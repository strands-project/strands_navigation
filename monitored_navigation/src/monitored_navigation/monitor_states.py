import rospy

import smach
import smach_ros
from smach import *
from smach_ros import *

from scitos_msgs.msg import MotorStatus
from strands_navigation_msgs.msg import NavStatus






"""
A smach_ros  MonitorState that monitors the robot's bumper. If the bumper get
pressed, this state to exit with outcome 'invalid'.
"""
class BumperMonitor(smach_ros.MonitorState):
    def __init__(self,):
        smach_ros.MonitorState.__init__(self, "/motor_status",
                                        MotorStatus,
                                        self._callback)
    
    """ Test the message and decide exit or not """
    def _callback(self,  ud,  msg):
        # using msg.bumper_pressed does not work properly because sometimes the
        # bumper is pressed but no change of state is published
        if msg.motor_stopped and not msg.free_run:
            #self.get_logger().log_bump()
            return False
        else:
            return True


class StuckOnCarpetMonitor(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, "/monitored_navigation/nav_status",
                                        NavStatus,
                                        self._callback)
    
    """ Test the message and decide exit or not """
    def _callback(self,  ud,  msg):
        if  msg.carpet_stuck:
            #self.get_logger().log_carpet_stuck()
            return False
        else:
            return True


            
class NavPreemptMonitor(smach_ros.MonitorState):
    def __init__(self):
        
           
        
        smach_ros.MonitorState.__init__(self, "/monitored_navigation/nav_status",
                                        NavStatus,
                                        self._callback)
    
    """ Test the message and decide exit or not """
    def _callback(self,  ud,  msg):
        if  msg.is_preempted:
            return False
        else:
            return True
       


