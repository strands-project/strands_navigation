import rospy
import smach
import actionlib

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from strands_navigation_msgs.msg import MonitoredNavigationResult, MonitoredNavigationActionGoal

from recover_state_machine import RecoverStateMachine
from monitor_state import MonitorState



 
class NavActionState(smach.State):
       
    def __init__(self):   
        smach.State.__init__(self,
                            outcomes=['succeeded', 'preempted','aborted','planner_failure'],
                            input_keys=['goal'],
                            output_keys=['n_fails'],
                            )
        
        self.new_action_in = False
        self.last_new_action_server_name=''
        self.n_fails=0
        
        rospy.Subscriber("/monitored_navigation/goal" ,MonitoredNavigationActionGoal, self.new_goal_checker_cb)
 
    def new_goal_checker_cb(self,msg):
        self.new_action_in = True
        self.last_new_action_server_name=msg.goal.action_server

    def execute(self, userdata):
        self.new_action_in = False
        action_server_name=userdata.goal.action_server
        action_client= actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        server_active = action_client.wait_for_server(rospy.Duration(10))
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not server_active:
            rospy.logwarn("Action server " + action_server_name + " is not active. Aborting...")
            return "aborted"
        mb_goal = MoveBaseGoal(target_pose=userdata.goal.target_pose)
        action_client.send_goal(mb_goal)
        status= action_client.get_state()
        while status==GoalStatus.PENDING or status==GoalStatus.ACTIVE:   
            status= action_client.get_state()
            if self.preempt_requested():
                if (not self.new_action_in) or (action_server_name != self.last_new_action_server_name):
                    action_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
                
            action_client.wait_for_result(rospy.Duration(0.2))
            
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        if status == GoalStatus.SUCCEEDED:
            self.n_fails=0
            userdata.n_fails = self.n_fails
            return 'succeeded'
        elif status==GoalStatus.PREEMPTED:
            self.n_fails=0
            userdata.n_fails = self.n_fails
            return 'preempted'
        else:
            self.n_fails=self.n_fails+1
            userdata.n_fails = self.n_fails
            return 'planner_failure'
   

"""
Move the robot to a goal location, with some basic recovery attempts on failure.
Recovery is implemented in recover_states.RecoverMoveBase

outcomes: 	succeeded
			failed
            preempted
            
input keys:	goal_pose 
"""
class RecoverableNav:
    def __init__(self):
        self.nav_action_state = NavActionState()
        self.init_nav_sm(None)
    
    def init_nav_sm(self, recover_sm):
        self.nav_sm=smach.StateMachine(outcomes=['succeeded',
                                              'preempted',
                                              'recovered_with_help',
                                              'recovered_without_help',
                                              'not_recovered_with_help',
                                              'not_recovered_without_help'],
                                    input_keys=['goal'],
                                    output_keys=[])
        
        with self.nav_sm:
            if recover_sm is None:
                smach.StateMachine.add('NAVIGATION',
                                   self.nav_action_state, 
                                   transitions={'succeeded': 'succeeded',
                                                'preempted': 'preempted',
                                                'aborted':'not_recovered_without_help',
                                                'planner_failure':  'not_recovered_without_help'}
                                   )
            else:
                smach.StateMachine.add('NAVIGATION',
                                   self.nav_action_state, 
                                   transitions={'succeeded': 'succeeded',
                                                'preempted': 'preempted',
                                                'aborted':'not_recovered_without_help',
                                                'planner_failure':  'RECOVER_NAVIGATION'}
                                   )
                smach.StateMachine.add('RECOVER_NAVIGATION',
                                   recover_sm,  
                                   transitions={'preempted':'preempted',
                                                'recovered_with_help': 'recovered_with_help',
                                                'recovered_without_help': 'recovered_without_help',
                                                'not_recovered_with_help':'not_recovered_with_help',
                                                'not_recovered_without_help':'not_recovered_without_help'} )

 
    def set_nav_recovery(self,recover_sm):
        if not isinstance(recover_sm,RecoverStateMachine):
            rospy.logwarn("The navigation recovery state machine needs to be an instantiation of the RecoverStateMachine class")
            return
        self.init_nav_sm(recover_sm)
            
  
"""


outcomes: 	bumper_pressed
            succeeded
            failure
            
input keys:	goal_pose
           
"""
class MonitoredRecoverableNav:
    def __init__(self):
        self.recoverable_nav=RecoverableNav()
        self.monitor_list={}
        self.monitored_cc=None
        self.init_monitored_cc()
        
    def set_nav_recovery(self,recover_sm):
        self.recoverable_nav.set_nav_recovery(recover_sm)
        self.init_monitored_cc()
        
        
        
    def init_monitored_cc(self):
        outcomes=['succeeded',
                    'preempted',
                    'recovered_with_help',
                    'recovered_without_help',
                    'not_recovered_with_help',
                    'not_recovered_without_help']

        self.monitored_cc=smach.Concurrence(outcomes=outcomes,
                                   default_outcome='not_recovered_without_help',
                                   child_termination_cb=self.child_term_cb,
                                   outcome_cb=self.out_cb,
                                   input_keys=['goal', 'n_fails'],
                                   output_keys=['n_fails']
                                   )
        with self.monitored_cc:
            smach.Concurrence.add('NAV_SM', self.recoverable_nav.nav_sm)
            for name, monitor in self.monitor_list.iteritems():
                smach.Concurrence.add(name, monitor)
                self.monitored_cc.register_outcomes([name+"_fail"])
        

    def add_monitor(self, monitor_sm, name):
        if not isinstance(monitor_sm,MonitorState):
            rospy.logwarn("The monitor state needs to be an instantiation of the MonitorState class")
            return
        if name in self.monitor_list:
            rospy.logwarn("There already exists a monitor named " + name)
            return
        self.monitor_list[name]=monitor_sm
        self.init_monitored_cc()
        
        
    def del_monitor(self, name):
        del(self.monitor_list[name])
        self.init_monitored_cc()
        
        
    def set_monitors(self, monitor_list, name_list):
        self.monitor_list={}
        for name, monitor in zip(name_list, monitor_list):
            self.monitor_list[name]=monitor
        self.init_monitored_cc()
    
    def child_term_cb(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        for name, monitor in self.monitor_list.iteritems():
            if  outcome_map[name] == 'invalid':
                return True
        if ( outcome_map["NAV_SM"] == "succeeded" or
             outcome_map['NAV_SM'] == "preempted"  or
             outcome_map['NAV_SM'] == 'recovered_with_help'  or
             outcome_map['NAV_SM'] == 'recovered_without_help'  or
             outcome_map['NAV_SM'] == 'not_recovered_with_help' or
             outcome_map['NAV_SM'] == 'not_recovered_without_help'):
            return True
        return False
    
    def out_cb(self, outcome_map):
        for name, monitor in self.monitor_list.iteritems():
            if  outcome_map[name] == 'invalid':
                return name+"_fail"
        if outcome_map["NAV_SM"] == "succeeded":
            return "succeeded"         
        if outcome_map["NAV_SM"] == "preempted":
            return "preempted"
        if outcome_map["NAV_SM"] == 'recovered_with_help':
            return 'recovered_with_help'  
        if outcome_map["NAV_SM"] == 'recovered_without_help':
            return 'recovered_without_help' 
        if outcome_map["NAV_SM"] == 'not_recovered_with_help':
            return 'not_recovered_with_help'  
        if outcome_map["NAV_SM"] == 'not_recovered_without_help':
            return 'not_recovered_without_help'  




"""
The highest level "goto position" state. This will use move_base to goto a goal
position, mean while checking the battery and the bumper. If move_base fails,
some recovery is attempted. If the bumper is pressed, but then released, it will
resume.

outcomes:	succeeded			- got to position
			bumper_failure		- bumper is in constant contact
            move_base_failure	- move_base fails, can't recover
            
input_keys:	goal_pose		- move_base_msgs.msg/MoveBaseGoal

"""
class HighLevelNav:
    def __init__(self):
        self.monitored_recoverable_nav = MonitoredRecoverableNav()
        self.recovery_list={}
        self.high_level_sm=None
        self.init_high_level_sm()
        
        
        
   
    def init_high_level_sm(self):
        outcomes=['succeeded',
                    'preempted',
                    'recovered_with_help',
                    'recovered_without_help',
                    'not_recovered_with_help',
                    'not_recovered_without_help']
                    
        self.high_level_sm=smach.StateMachine(outcomes=outcomes,
                                                input_keys=['goal'],
                                                output_keys=['result'])
        

        with self.high_level_sm:
            nav_transitions={'succeeded': 'succeeded',
                               'preempted':'preempted',
                               'recovered_with_help':'recovered_with_help',
                               'recovered_without_help':'recovered_without_help',
                               'not_recovered_with_help':'not_recovered_with_help', 
                               'not_recovered_without_help':'not_recovered_without_help'}
            for name, recover_sm in self.recovery_list.iteritems():
                nav_transitions[name+"_fail"]=name+"_recover"
                smach.StateMachine.add(name+"_recover",
                                    recover_sm,
                                    transitions={'recovered_with_help':'recovered_with_help',
                                                'recovered_without_help':'MONITORED_NAV',
                                                'not_recovered_with_help':'not_recovered_with_help', 
                                                'not_recovered_without_help':'not_recovered_without_help',
                                                'preempted':'preempted'})
          
            smach.StateMachine.add('MONITORED_NAV',
                                   self.monitored_recoverable_nav.monitored_cc,
                                   transitions=nav_transitions)
                        
        self.high_level_sm.set_initial_state(["MONITORED_NAV"], userdata=smach.UserData())
        self.high_level_sm.register_start_cb(self.start_cb)
        self.high_level_sm.register_termination_cb(self.termination_cb, cb_args=[])
            
   
   
    def set_nav_recovery(self,recover_sm):
        if not isinstance(recover_sm,RecoverStateMachine):
            rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class")
            return  False
        self.monitored_recoverable_nav.set_nav_recovery(recover_sm)
        self.init_high_level_sm()
        return True
        
   
    def add_monitor_recovery_pair(self, monitor,recover_sm, name):
        if not isinstance(recover_sm,RecoverStateMachine):
            rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class")
            return  False
        if not isinstance(monitor,MonitorState):
            rospy.logwarn("The monitor state needs to be an instantiation of the MonitorState class")
            return False
        if name in self.recovery_list:
            rospy.logwarn("There already exists a recovery behaviour named " + name)
            return False
        self.monitored_recoverable_nav.add_monitor(monitor, name)
        self.recovery_list[name]=recover_sm
        self.init_high_level_sm()
        return True
        
    def del_monitor_recovery_pair(self, name):
        if name not in self.recovery_list:
            rospy.logwarn("There is no recovery state machine with name " + name)
            return False
        if name not in self.monitored_recoverable_nav.monitor_list:
            rospy.logwarn("There is no monitor state with name " + name)
            return False
        self.monitored_recoverable_nav.del_monitor(name)
        del(self.recovery_list[name])
        self.init_high_level_sm()
        return True
        
    def set_monitor_recovery_pairs(self, monitor_list, recovery_list, name_list):
        if (len(name_list) != len(monitor_list)) or (len(name_list) != len(recovery_list)):
            rospy.logwarn("There needs to be a one-to-one correspondence between names, monitors and recoveries.")
            return  False
        for monitor in monitor_list:
            if not isinstance(monitor,MonitorState):
                rospy.logwarn("The monitor state needs to be an instantiation of the MonitorState class")
                return  False
        for recovery in recovery_list:
            if not isinstance(recovery,RecoverStateMachine):
                rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class")
                return  False
        self.monitored_recoverable_nav.set_monitors(monitor_list, name_list)
        self.recovery_list={}
        for name, recovery in zip(name_list, recovery_list):
            self.recovery_list[name]=recovery
        self.init_high_level_sm()
        return  True
    
    def start_cb(self, userdata, initial_states):
        self.high_level_sm.userdata.n_fails=0

    def termination_cb(self,userdata, terminal_states, outcome):
        userdata.result=MonitoredNavigationResult()
        
        if outcome=='succeeded':
            userdata.result.outcome='succeeded'
        if outcome=='preempted':
            userdata.result.outcome='preempted'
        if outcome=='recovered_with_help':
            userdata.result.outcome = terminal_states[0]
            userdata.result.recovered=True
            userdata.result.human_interaction=True
        if outcome=='recovered_without_help':
            userdata.result.outcome= terminal_states[0]
            userdata.result.recovered=True
            userdata.result.human_interaction=False
        if outcome=='not_recovered_with_help':
            userdata.result.outcome= terminal_states[0]
            userdata.result.recovered=False
            userdata.result.human_interaction=True
        if outcome=='not_recovered_without_help':
            userdata.result.outcome= terminal_states[0]
            userdata.result.recovered=False
            userdata.result.human_interaction=False

