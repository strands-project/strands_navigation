#! /usr/bin/env python

import rospy


from strands_navigation_msgs.srv import AskHelp
from strands_navigation_msgs.srv import Register
    
class HelpManager(object):

    def __init__(self):
    
        self.registration_service=rospy.Service('/monitored_navigation/human_help/register', Register, self.registration_callback)
        self.unregistration_service=rospy.Service('/monitored_navigation/human_help/unregister', Register, self.unregistration_callback)
        self.forward_service=rospy.Service('/monitored_navigation/human_help/manager', AskHelp, self.forward_callback)
        
        self.registered_helpers={}

        
    def registration_callback(self,req):
        self.registered_helpers[req.id]=rospy.ServiceProxy(req.service_name, AskHelp)
        return []
        
    def unregistration_callback(self,req):
        del(self.registered_helpers[req.id])
        return []
        
        
    def forward_callback(self,req):
        for key in self.registered_helpers:
            rospy.logwarn(key)
            self.registered_helpers[key](req)
        return "help!!"
        
        
    def main(self):
        
        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    
    rospy.init_node('human_help_manager')  
    manager =  HelpManager()
    manager.main()
