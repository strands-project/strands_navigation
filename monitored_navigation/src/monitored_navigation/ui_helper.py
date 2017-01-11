#! /usr/bin/env python

import rospy

from strands_navigation_msgs.srv import AskHelpRequest

 
class UIHelper:   
    def __init__(self):            
        self.previous_interaction=None
        
    def process_help_request(self,req):
        if not self.previous_interaction == req.interaction_status:
            self.previous_interaction=req.interaction_status
            if req.interaction_status==AskHelpRequest.ASKING_HELP:
                self.ask_help(req.failed_component, req.interaction_service,req.n_fails)
            elif  req.interaction_status==AskHelpRequest.BEING_HELPED:
                self.being_helped(req.failed_component, req.interaction_service,req.n_fails)
            elif req.interaction_status==AskHelpRequest.HELP_FINISHED:
                self.help_finished(req.failed_component, req.interaction_service,req.n_fails)
            elif  req.interaction_status==AskHelpRequest.HELP_FAILED:
                self.help_failed(req.failed_component, req.interaction_service,req.n_fails)
    
    
    def ask_help(self, failed_component, interaction_service, n_fails):
        rospy.logwarn("This should be overriden by the helper.")
        
    def being_helped(self, failed_component, interaction_service, n_fails):
        rospy.logwarn("This should be overriden by the helper.")
        
    def help_finished(self, failed_component, interaction_service, n_fails):
        rospy.logwarn("This should be overriden by the helper.")
        
    def help_failed(self, failed_component, interaction_service, n_fails):
        rospy.logwarn("This should be overriden by the helper.")


