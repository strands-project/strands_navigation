import rospy

import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction


from monitor_states import BumperMonitor, StuckOnCarpetMonitor, NavPreemptMonitor
from recover_states import RecoverNav,  RecoverBumper, RecoverStuckOnCarpet

#from logger import Loggable


from scitos_msgs.srv import EnableMotors


"""
A SimpleActionState that sends a goal to the navigation stack using move_base

input keys: 	goal_pose			- geometry_msgs/Pose, goal pose in /map
                                      frame
				n_move_base_fails	- number of failures so far, will be 
	                                  incremented  and returned if fail
                                      
output keys:	n_move_base_fails	- see input.
"""
#class MoveBaseActionState(smach_ros.SimpleActionState):
    #def __init__(self):
        
        #self.MOVE_BASE_EXEC_TIMEOUT = rospy.Duration(600.0)
        #self.MOVE_BASE_PREEMPT_TIMEOUT = rospy.Duration(30.0)
        
        
        #smach_ros.SimpleActionState.__init__(self,
                                             #'move_base',
                                             #MoveBaseAction,
                                             #goal_cb=self.move_base_goal_cb,
                                             #result_cb=self.move_base_result_cb,
                                             #input_keys=['goal_pose',
                                                         #'n_move_base_fails'],
                                             #output_keys=['n_move_base_fails'],
                                             #exec_timeout=self.MOVE_BASE_EXEC_TIMEOUT,
                                             #preempt_timeout=self.MOVE_BASE_PREEMPT_TIMEOUT
                                             #)                                 
        
    #""" callback that builds the move_base goal, from the input data """
    #def move_base_goal_cb(self, userdata, goal):
        #next_goal =userdata.goal_pose   
        #return next_goal
    
    
    #"""
    #called after the move_base state terminates. Incre                continue
#ases the number of
    #move_base fails or resets it to 0 according to the move_base result
    #"""
    #def move_base_result_cb(self, userdata, status, result):
        #if status == GoalStatus.ABORTED:
            #userdata.n_move_base_fails = userdata.n_move_base_fails + 1
        #elif status == GoalStatus.SUCCEEDED:
            #userdata.n_move_base_fails = 0
        #elif status==GoalStatus.PREEMPTED:
            #return 'preempted'
                
              
    
            
            
class NavActionState(smach.State):
       
    def __init__(self):
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['goal','n_nav_fails',],
                             output_keys=['goal','n_nav_fails'],
                             )

        
                                                  
    def execute(self, userdata):
        action_server_name=userdata.goal.action_server
        action_client= actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        action_client.wait_for_server()
        action_client.send_goal(userdata.goal)
        status= action_client.get_state()
        while status==GoalStatus.PENDING or status==GoalStatus.ACTIVE:   
            status= action_client.get_state()
            if self.preempt_requested():
                action_client.cancel_goal()
                self.service_preempt()
            action_client.wait_for_result(rospy.Duration(0.2))
        
        if status == GoalStatus.SUCCEEDED:
            userdata.n_nav_fails = 0
            return 'succeeded'
        elif status==GoalStatus.PREEMPTED:
            return 'preempted'
        else:
            userdata.n_nav_fails = userdata.n_nav_fails + 1
            return 'aborted'
        
                 


"""
Move the robot to a goal location, with some basic recovery attempts on failure.
Recovery is implemented in recover_states.RecoverMoveBase

outcomes: 	succeeded
			failed
            preempted
            
input keys:	goal_pose 
"""
class RecoverableNav(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded',
                                              'failure',
                                              'preempted'],
                                    input_keys=['goal'])

        self.userdata.n_nav_fails = 0
        self._nav_action = NavActionState()
        self._recover_nav =  RecoverNav()
        with self:
            smach.StateMachine.add('NAVIGATION',
                                   self._nav_action, 
                                   transitions={'succeeded': 'succeeded',
                                                'aborted':  'RECOVER_NAVIGATION',
                                                'preempted': 'preempted'}
                                   )
            smach.StateMachine.add('RECOVER_NAVIGATION',
                                   self._recover_nav,  
                                   transitions={'succeeded': 'NAVIGATION',
                                                'failure': 'failure',
                                                'preempted':'preempted'} )
            
    def execute(self, userdata=smach.UserData()):
        outcome = smach.StateMachine.execute(self, userdata)

            
        return outcome
        
    def set_nav_thresholds(self, max_nav_recovery_attempts):
        self._recover_nav.set_nav_thresholds(max_nav_recovery_attempts)         
            
"""


outcomes: 	bumper_pressed
            succeeded
            failure
            
input keys:	goal_pose
           
"""
class MonitoredRecoverableNav(smach.Concurrence):
    def __init__(self):
        self.enable_motors= rospy.ServiceProxy('enable_motors',
                                                  EnableMotors) 
        
        smach.Concurrence.__init__(self,
                                   outcomes=['bumper_pressed',
                                             'succeeded',
                                             'failure',
                                             'preempted',
                                             'stuck_on_carpet'],
                                   default_outcome='failure',
                                   child_termination_cb=self.child_term_cb,
                                   outcome_cb=self.out_cb,
                                   input_keys=['goal']
                                   )
        self._bumper_monitor = BumperMonitor()
        self._recoverable_nav = RecoverableNav()
        self._carpet_monitor = StuckOnCarpetMonitor()
        self._nav_preempt_monitor=NavPreemptMonitor()
        with self:
            smach.Concurrence.add('BUMPER_MONITOR', self._bumper_monitor)
            smach.Concurrence.add('STUCK_ON_CARPET_MONITOR', self._carpet_monitor)
            smach.Concurrence.add('NAV_PREEMPT_MONITOR', self._nav_preempt_monitor)
            smach.Concurrence.add('NAV_SM', self._recoverable_nav)
    
    def child_term_cb(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        if ( outcome_map['BUMPER_MONITOR'] == 'invalid' or
             outcome_map["STUCK_ON_CARPET_MONITOR"] == "invalid" or
             outcome_map["NAV_PREEMPT_MONITOR"] == "invalid" or             
             outcome_map["NAV_SM"] == "succeeded" or
             outcome_map['NAV_SM'] == "failure" or
             outcome_map['NAV_SM'] == "preempted"  ):
            return True
        return False
    
    def out_cb(self, outcome_map):
        if outcome_map['BUMPER_MONITOR'] == 'invalid':
            return 'bumper_pressed'
        if outcome_map["STUCK_ON_CARPET_MONITOR"] == "invalid":
            return "stuck_on_carpet"
        if outcome_map["NAV_PREEMPT_MONITOR"] == "invalid":
            return "preempted"
        if outcome_map["NAV_SM"] == "succeeded":
            return "succeeded"
        if outcome_map["NAV_SM"] == "failure":
            return "failure"
        if outcome_map["NAV_SM"] == "preempted":
            return "preempted"

        
    
    
    
    """ 
    Set the battery level thresholds.
    """
    def set_nav_thresholds(self,max_bumper_recovery_attempts,max_nav_recovery_attempts):
        self._recoverable_nav.set_nav_thresholds(max_nav_recovery_attempts)
    



"""
The highest level "goto position" state. This will use move_base to goto a goal
position, mean while checking the battery and the bumper. If move_base fails,
some recovary is attempted. If the bumper is pressed, but then released, it will
resume.

outcomes:	succeeded			- got to position
			bumper_failure		- bumper is in constant contact
            move_base_failure	- move_base fails, can't recover
            
input_keys:	goal_pose		- move_base_msgs.msg/MoveBaseGoal

"""
class HighLevelNav(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'nav_failure',
                                                    'bumper_failure',
                                                    'preempted'],
                                          input_keys=['goal'])
        self._monitored_recoverable_nav = MonitoredRecoverableNav()
        self._recover_bumper =  RecoverBumper()
        self._recover_carpet =  RecoverStuckOnCarpet()
        with self:
            smach.StateMachine.add('MONITORED_NAV',
                                   self._monitored_recoverable_nav,
                                   transitions={'bumper_pressed': 'RECOVER_BUMPER',
                                                'succeeded': 'succeeded',
                                                'failure': 'nav_failure',
                                                'preempted':'preempted',
                                                'stuck_on_carpet':'RECOVER_STUCK_ON_CARPET'})
            smach.StateMachine.add('RECOVER_BUMPER',
                                   self._recover_bumper,
                                   transitions={'succeeded': 'MONITORED_NAV',
                                                'failure':'bumper_failure'})
            smach.StateMachine.add('RECOVER_STUCK_ON_CARPET',
                                   self._recover_carpet,
                                   transitions={'succeeded': 'MONITORED_NAV',
                                                'failure': 'RECOVER_STUCK_ON_CARPET'})                                                
    
    
    """ 
    Set the battery level thresholds.
    """
    def set_nav_thresholds(self, max_bumper_recovery_attempts,max_nav_recovery_attempts):
        self._monitored_recoverable_nav.set_nav_thresholds(max_bumper_recovery_attempts,max_nav_recovery_attempts)
        self._recover_bumper.set_nav_thresholds(max_bumper_recovery_attempts)
    
