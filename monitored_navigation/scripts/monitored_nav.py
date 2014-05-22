#! /usr/bin/env python

import sys
import rospy


from smach_ros import ActionServerWrapper
#from move_base_msgs.msg import MoveBaseAction
from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationActionGoal

from dynamic_reconfigure.server import Server
from monitored_navigation.cfg import NavFailTresholdsConfig

from  monitored_navigation.navigation import HighLevelNav

   
class MonitoredNavigation(ActionServerWrapper):

    def __init__(self):
        
        # Create the main state machine
        self.nav_sm = HighLevelNav()
        self.mon_nav_sm=self.nav_sm.get_children()["MONITORED_NAV"]
        self.low_nav_sm=self.mon_nav_sm.get_children()["NAV_SM"]
        
        ActionServerWrapper.__init__(self,
                        'monitored_navigation', MonitoredNavigationAction, self.nav_sm,
                        ['succeeded'], ['nav_local_plan_failure','nav_global_plan_failure', 'bumper_failure'], ['preempted'],
                        goal_key = 'goal', result_key='result'
                        )
    
        self.current_action=''
        self.new_action=''
        
        ## Create a logger
        #logger =  PatrollLogger("autonomous_patrolling")
        #self.long_term_patrol_sm.set_logger(logger)
        
        # dynamic reconfiguration of failure tresholds
        self.srv = Server(NavFailTresholdsConfig, self.reconfigure_callback)
        
        rospy.Subscriber("/monitored_navigation/goal" , MonitoredNavigationActionGoal, self.new_goal_checker_cb)
        
        
        
    def check_nav_executing(self):
        if "MONITORED_NAV" not in  self.nav_sm.get_active_states() or "NAV_SM" not in self.mon_nav_sm.get_active_states() or "NAVIGATION" not in self.low_nav_sm.get_active_states():
            return False
        else:
            return True
        
    def new_goal_checker_cb(self,msg):
        self.last_new_action_time=rospy.get_rostime()
        self.new_action=msg.goal.action_server    

    def preempt_cb(self):
        wait_time=0
        if rospy.get_rostime()-self.last_new_action_time < rospy.Duration(1) and not self.current_action == self.new_action:
            while self.nav_sm.is_running() and self.check_nav_executing() and wait_time < 1000:
                rospy.sleep(0.1)
                wait_time=wait_time+1
        ActionServerWrapper.preempt_cb(self)
    
    def execute_cb(self,goal):
        self.current_action=goal.action_server
        ActionServerWrapper.execute_cb(self,goal)
        

     
     
    def reconfigure_callback(self, config, level):
        self.nav_sm.set_nav_thresholds(config.max_bumper_recovery_attempts,config.max_nav_recovery_attempts)
        return config
    
    
    
    
    def main(self):
        
        # Run the server in a background thread
        self.run_server()

        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('monitored_navigation')
 
    mon_nav =  MonitoredNavigation()
    mon_nav.main()
