#! /usr/bin/env python

import rospy
from threading import Thread

from strands_navigation_msgs.srv import AskHelp
from ui_helper import UIHelper
    
    
class HelpManager(object):

    def __init__(self):
        self.forward_service=rospy.Service('/monitored_navigation/human_help', AskHelp, self.forward_callback)
        self.helpers={}

    def add_helper(self, helper, name):
        if not isinstance(helper,UIHelper):
            rospy.logwarn("The interface for asking human help must be an instantiation of UIHelper")
            return False
        if name in self.helpers:
            rospy.logwarn("There already exists a helper named " + name)
            return False
        self.helpers[name]=helper
        return True
        
    def del_helper(self, name):
        if name in self.helpers:
            del(self.helpers[name])
            return True
        else:
            rospy.logwarn("No helper named " + name)
            return False
    
    def set_helpers(self, helper_list, name_list):
        if len(helper_list) == len(name_list):
            for helper, name in zip(helper_list, name_list):
                if not isinstance(helper,UIHelper):
                    rospy.logwarn("The interface for asking human help with name " + name + " is not an instantiation of UIHelper.")
                    return False
            self.helpers={}
            for helper, name in zip(helper_list, name_list):
                self.helpers[name]=helper
            return True
        else:
             rospy.logwarn("There needs to be a one-to-one correspondence between helpers and names")
             return False
        
 
    def forward_callback(self,req):
        for helper in self.helpers.values():
            t = Thread(target=helper.process_help_request, args = (req, ))
            t.start()
        return []
 