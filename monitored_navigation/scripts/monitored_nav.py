#! /usr/bin/env python

import sys
import rospy


from smach_ros import ActionServerWrapper
#from move_base_msgs.msg import MoveBaseAction
from strands_navigation_msgs.msg import MonitoredNavigationAction

from dynamic_reconfigure.server import Server
from monitored_navigation.cfg import NavFailTresholdsConfig


from  monitored_navigation.navigation import HighLevelNav


#import marathon_touch_gui.client
    
    
class MonitoredNavigation(object):
    """
    Constructor.
    :param waypoints_name: str, name of waypoints as appears in the datacentre
    :param is_random: bool, should the waypoints be visited in random order
    :param n_iterations: int, how many times to visit all the waypoints
    """
    def __init__(self):
    
        # Create the main state machine
        self.nav_sm = HighLevelNav()
        
        ## Create a logger
        #logger =  PatrollLogger("autonomous_patrolling")
        #self.long_term_patrol_sm.set_logger(logger)
        
        # dynamic reconfiguration of battery tresholds
        self.srv = Server(NavFailTresholdsConfig, self.reconfigure_callback)
    

     
     
    def reconfigure_callback(self, config, level):
        self.nav_sm.set_nav_thresholds(config.max_bumper_recovery_attempts,config.max_nav_recovery_attempts)
        return config
    
    
    """ The Main start point for Long Term Patroller """
    def main(self):
        asw = ActionServerWrapper(
                        'monitored_navigation', MonitoredNavigationAction, self.nav_sm,
                        ['succeeded'], ['move_base_failure'], ['preempted'],
                        goal_key = 'goal',
                        result_key = 'move_result'
                        )
        
        # Run the server in a background thread
        asw.run_server()

        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('monitored_navigation')
    

        
    mon_nav =  MonitoredNavigation()
    mon_nav.main()
