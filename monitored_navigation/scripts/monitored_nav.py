#! /usr/bin/env python

import sys
import yaml
import rospy

from dynamic_reconfigure.server import Server
from monitored_navigation.cfg import NavFailTresholdsConfig

from smach_ros import ActionServerWrapper, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction

from  monitored_navigation.navigation import HighLevelNav
from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationActionGoal

   
class MonitoredNavigation(ActionServerWrapper):

    def __init__(self, filename):
        
        # Create the main state machine
        self.high_level_nav = HighLevelNav()
        
        
        ##TODO:  Add services for changing recoveries at runtime
        
        if file_name is not None:
            stream = open(file_name, 'r')
            yaml_config=yaml.load(stream)
            nav_recovery_dict=yaml_config["nav_recovery"]
            self.high_level_nav.set_nav_recovery(self.create_recovery_object(nav_recovery_dict))

            monitor_recovery_pairs=yaml_config["monitor_recovery_pairs"]
            for monitor_recovery_pair in monitor_recovery_pairs:
                self.high_level_nav.add_monitor_recovery_pair(self.create_monitor_object(monitor_recovery_pair),self.create_recovery_object(monitor_recovery_pair),monitor_recovery_pair["name"])
        
        self.sis = IntrospectionServer('monitored_navigation_sm', self.high_level_nav.high_level_sm, '/MON_NAV')
        self.sis.start()
        
        ActionServerWrapper.__init__(self,
                        'monitored_navigation', MonitoredNavigationAction, self.high_level_nav.high_level_sm,
                        ['succeeded'], ['recovered_with_help', 'recovered_without_help', 'not_recovered_with_help', 'not_recovered_without_help'], ['preempted'],
                        goal_key = 'goal', result_key='result'
                        )
    

    
    def create_recovery_object(self, recovery_dict):
        mod = __import__(recovery_dict["package"]+ '.' + recovery_dict["recovery_file"], fromlist=[recovery_dict["recovery_class"]])
        recovery_class=getattr(mod, recovery_dict["recovery_class"])
        return recovery_class()
        
    def create_monitor_object(self, monitor_dict):
        mod = __import__(monitor_dict["package"]+ '.' + monitor_dict["monitor_file"], fromlist=[monitor_dict["monitor_class"]])
        monitor_class=getattr(mod, monitor_dict["monitor_class"])
        return monitor_class()
        
        
    
    def main(self):
        
        # Run the server in a background thread
        self.run_server()

        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('monitored_navigation')
    
    if len(sys.argv) < 1 or sys.argv[1]=="none":
        rospy.logwarn("No config yaml file provided. MonitoredNavigation state machine will be initialized without recovery behaviours. The strands config yaml file is located in monitored_navigation/config/strands.yaml")
        file_name=None
    else:
        file_name=sys.argv[1]

   
    mon_nav =  MonitoredNavigation(file_name)
    mon_nav.main()
