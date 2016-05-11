import rospy

import smach
import smach_ros




class MonitorState(smach_ros.MonitorState):
    def __init__(self,topic, msg_type, cb, input_keys=[], output_keys=[]):
        smach_ros.MonitorState.__init__(self, topic, msg_type, cb, -1, input_keys, output_keys)
