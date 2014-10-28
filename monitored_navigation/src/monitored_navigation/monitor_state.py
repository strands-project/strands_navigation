import rospy

import smach
import smach_ros




class MonitorState(smach_ros.MonitorState):
    def __init__(self,topic, msg_type, cb):
        self.name=None
        smach_ros.MonitorState.__init__(self, topic, msg_type, cb)
