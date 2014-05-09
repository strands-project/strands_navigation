import rospy

import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import GoalStatus, GoalID
from move_base_msgs.msg import MoveBaseAction

from strands_navigation_msgs.msg import MonitoredNavigationResult

from nav_msgs.msg import Path

from monitor_states import BumperMonitor, StuckOnCarpetMonitor, NavPreemptMonitor
from recover_states import RecoverNavHelp, RecoverNavBacktrack,  RecoverBumper, RecoverStuckOnCarpet

#from logger import Loggable


from scitos_msgs.srv import EnableMotors



    
            
            
class NavActionState(smach.State):
       
    def __init__(self):
        
        smach.State.__init__(self,
                             outcomes=['succeeded', 'preempted','global_plan_failure','local_plan_failure'],
                             input_keys=['goal','n_nav_fails',],
                             output_keys=['goal','n_nav_fails'],
                             )
        
        self.global_plan=None
        self.last_global_plan_time=rospy.Time(0)
        self.last_action_cancel_time=rospy.Time(0)
        
        rospy.Subscriber("/move_base/NavfnROS/plan" , Path, self.global_planner_checker_cb)
        rospy.Subscriber("/monitored_navigation/cancel" , GoalID, self.cancel_checker_cb)
        
        
        
    def global_planner_checker_cb(self,msg):
        self.global_plan=msg
        self.last_global_plan_time=rospy.get_rostime()
        
        
    def cancel_checker_cb(self,msg):
        self.last_action_cancel_time=rospy.get_rostime()    
        
                                                  
    def execute(self, userdata):
        action_server_name=userdata.goal.action_server
        action_client= actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        action_client.wait_for_server()
        action_client.send_goal(userdata.goal)
        status= action_client.get_state()
        while status==GoalStatus.PENDING or status==GoalStatus.ACTIVE:   
            status= action_client.get_state()
            if self.preempt_requested():
                if rospy.get_rostime()-self.last_action_cancel_time< rospy.Duration(1):
                    action_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            action_client.wait_for_result(rospy.Duration(0.2))
        
        if status == GoalStatus.SUCCEEDED:
            userdata.n_nav_fails = 0
            return 'succeeded'
        elif status==GoalStatus.PREEMPTED:
            return 'preempted'
        else:            
            if (rospy.get_rostime()-self.last_global_plan_time < rospy.Duration(1)) and (self.global_plan.poses == []):
                userdata.n_nav_fails = 0
                return 'global_plan_failure'
            else:
                userdata.n_nav_fails = userdata.n_nav_fails + 1
                return 'local_plan_failure'
        
                 


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
                                              'local_plan_failure',
                                              'global_plan_failure',
                                              'preempted'],
                                    input_keys=['goal'])

        self.userdata.n_nav_fails = 0
        self._nav_action = NavActionState()
        self._recover_nav_backtrack =  RecoverNavBacktrack()
        self._recover_nav_help = RecoverNavHelp()
        with self:
            smach.StateMachine.add('NAVIGATION',
                                   self._nav_action, 
                                   transitions={'succeeded': 'succeeded',
                                                'local_plan_failure':  'RECOVER_NAVIGATION_BACKTRACK',
                                                'global_plan_failure':'global_plan_failure',
                                                'preempted': 'preempted'}
                                   )
            smach.StateMachine.add('RECOVER_NAVIGATION_BACKTRACK',
                                   self._recover_nav_backtrack,  
                                   transitions={'succeeded': 'NAVIGATION',
                                                'failure': 'RECOVER_NAVIGATION_HELP',
                                                'preempted':'preempted'})
            smach.StateMachine.add('RECOVER_NAVIGATION_HELP',
                                   self._recover_nav_help,  
                                   transitions={'succeeded': 'NAVIGATION',
                                                'failure': 'local_plan_failure',
                                                'preempted':'preempted'} )
            
    def execute(self, userdata=smach.UserData()):
        outcome = smach.StateMachine.execute(self, userdata)   
        return outcome
        
    def set_nav_thresholds(self, max_nav_recovery_attempts):
        self._recover_nav_help.set_nav_thresholds(max_nav_recovery_attempts)         
            
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
                                             'local_plan_failure',
                                             'global_plan_failure',
                                             'preempted',
                                             'stuck_on_carpet'],
                                   default_outcome='local_plan_failure',
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
             outcome_map['NAV_SM'] == "local_plan_failure" or
             outcome_map['NAV_SM'] == "global_plan_failure" or
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
        if outcome_map["NAV_SM"] == "local_plan_failure":
            return "local_plan_failure"
        if outcome_map["NAV_SM"] == "global_plan_failure":
            return "global_plan_failure"            
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
                                                    'nav_local_plan_failure',
                                                    'nav_global_plan_failure',
                                                    'bumper_failure',
                                                    'preempted'],
                                          input_keys=['goal'],
                                          output_keys=['result'])
        self._monitored_recoverable_nav = MonitoredRecoverableNav()
        self._recover_bumper =  RecoverBumper()
        self._recover_carpet =  RecoverStuckOnCarpet()
        
        self.register_termination_cb(self.termination_cb, cb_args=[])
        
        
        with self:
            smach.StateMachine.add('MONITORED_NAV',
                                   self._monitored_recoverable_nav,
                                   transitions={'bumper_pressed': 'RECOVER_BUMPER',
                                                'succeeded': 'succeeded',
                                                'local_plan_failure': 'nav_local_plan_failure',
                                                'global_plan_failure': 'nav_global_plan_failure',
                                                'preempted':'preempted',
                                                'stuck_on_carpet':'RECOVER_STUCK_ON_CARPET'})
            smach.StateMachine.add('RECOVER_BUMPER',
                                   self._recover_bumper,
                                   transitions={'succeeded': 'MONITORED_NAV',
                                                'preempted':'preempted',
                                                'failure':'bumper_failure'})
            smach.StateMachine.add('RECOVER_STUCK_ON_CARPET',
                                   self._recover_carpet,
                                   transitions={'succeeded': 'MONITORED_NAV',
                                                'failure': 'RECOVER_STUCK_ON_CARPET'})                                                
    
    def termination_cb(self,userdata, terminal_states, outcome):
        userdata.result=MonitoredNavigationResult()
        if outcome=='succeeded':
            userdata.result.sm_outcome=MonitoredNavigationResult().SUCCEEDED
        if outcome=='bumper_failure':
            userdata.result.sm_outcome=MonitoredNavigationResult().BUMPER_FAILURE
        if outcome=='nav_local_plan_failure':
            userdata.result.sm_outcome=MonitoredNavigationResult().LOCAL_PLANNER_FAILURE
        if outcome=='nav_global_plan_failure':
            userdata.result.sm_outcome=MonitoredNavigationResult().GLOBAL_PLANNER_FAILURE
        if outcome=='preempted':
            userdata.result.sm_outcome=MonitoredNavigationResult().PREEMPTED
        
    
    
    """ 
    Set the battery level thresholds.
    """
    def set_nav_thresholds(self, max_bumper_recovery_attempts,max_nav_recovery_attempts):
        self._monitored_recoverable_nav.set_nav_thresholds(max_bumper_recovery_attempts,max_nav_recovery_attempts)
        self._recover_bumper.set_nav_thresholds(max_bumper_recovery_attempts)
    
