#! /usr/bin/env python

import sys
import rospy

from dynamic_reconfigure.server import Server
from monitored_navigation.cfg import NavFailTresholdsConfig

from smach_ros import ActionServerWrapper, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction

from  monitored_navigation.navigation import HighLevelNav
from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationActionGoal

#from strands_recovery_behaviours.recover_bumper_no_help import RecoverBumperNoHelp
#from strands_recovery_behaviours.recover_bumper import RecoverBumper
#from strands_recovery_behaviours.monitor_bumper import MonitorBumper
#from strands_recovery_behaviours.monitor_stuck_on_carpet import MonitorStuckOnCarpet
#from strands_recovery_behaviours.recover_stuck_on_carpet_no_help import RecoverStuckOnCarpetNoHelp
#from strands_recovery_behaviours.recover_nav_no_help import RecoverNavNoHelp 
#from strands_recovery_behaviours.recover_nav import RecoverNav
   
class MonitoredNavigation(ActionServerWrapper):

    def __init__(self):
        
        # Create the main state machine
        self.high_level_nav = HighLevelNav()
        
        
        #TODO: initialize recovery behaviours via yaml file. Add services for changing recoveries at runtime
        #recover = RecoverBumper()
        #monitor=MonitorBumper()
        
        #self.high_level_nav.add_monitor_recovery_pair(monitor,recover,"bumper")
        
        #monitor=MonitorStuckOnCarpet()
        #recover=RecoverStuckOnCarpetNoHelp()
        
        #self.high_level_nav.add_monitor_recovery_pair(monitor,recover,"stuck_on_carpet")
        
        
        #recover=RecoverNav()
        #self.high_level_nav.set_nav_recovery(recover)
        
        self.sis = IntrospectionServer('monitored_navigation_sm', self.high_level_nav.high_level_sm, '/MON_NAV')
        self.sis.start()
        
        ActionServerWrapper.__init__(self,
                        'monitored_navigation', MonitoredNavigationAction, self.high_level_nav.high_level_sm,
                        ['succeeded'], ['recovered_with_help', 'recovered_without_help', 'not_recovered_with_help', 'not_recovered_without_help'], ['preempted'],
                        goal_key = 'goal', result_key='result'
                        )
    

    
    
    
    def main(self):
        
        # Run the server in a background thread
        self.run_server()

        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('monitored_navigation')
 
    mon_nav =  MonitoredNavigation()
    mon_nav.main()
