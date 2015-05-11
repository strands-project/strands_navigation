#!/usr/bin/env python

import rospy
import rosservice
import actionlib
from threading import Timer

from std_msgs.msg import String
import std_srvs.srv
import strands_navigation_msgs.srv
from strands_navigation_msgs.msg import TopologicalMap
import strands_emails.msg
import topological_navigation.msg
from strands_navigation_msgs.msg import MonitoredNavigationAction
from topological_navigation.msg import GotoNodeAction
from strands_navigation_msgs.msg import ExecutePolicyModeAction


class SafetyServer(object):

    def __init__(self, name) :
        rospy.on_shutdown(self._on_node_shutdown)
        #rospy.on_shutdown(self._on_node_shutdown)
        self.closest_node = "Unknown"
        self.stop_services=["/enable_motors", "/emergency_stop", "/task_executor/set_execution_status"]
        self.activate_services=["/enable_motors", "/reset_motorstop", "/task_executor/set_execution_status"]  
        self.info=''
        self.service_called=''
        self.monit_nav_cli = False
        self.top_nav_cli = False
        self.exec_pol_cli = False
        self.pre_active = False
        self.safety_stop = False
        self.notificate_to = rospy.get_param("/admin_email",'henry.strands@hanheide.net')


        #Waiting for Topological Map        
        self._map_received=False
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)      
        rospy.loginfo("Waiting for Topological map ...")        
        while not self._map_received :
            pass
        rospy.loginfo(" ...done")

        self.create_action_clients()
        self.update_service_list()
        
        
        #Subscribing to Localisation Topics
        rospy.loginfo("Subscribing to Localisation Topics")
        rospy.Subscriber('/closest_node', String, self.cNodeCallback)
        rospy.loginfo(" ...done")

        #This service returns any given map
        self.get_map_srv=rospy.Service('/go_to_safety_point', std_srvs.srv.Empty, self.goto_safety_cb)

        self.safety_stop_srv=rospy.Service('/safety_stop', std_srvs.srv.Empty, self.safety_stop_cb)
        self.reset_safety_stop_srv=rospy.Service('/reset_safety_stop', std_srvs.srv.Empty, self.reset_safety_stop_cb)


        self._killall_timers=False
        t = Timer(1.0, self.timer_callback)
        t.start()

        rospy.loginfo("All Done ...")
        rospy.spin()


    def update_service_list(self):
        self.av_stop_services=[]
        self.av_activate_services=[]        
        #print rosservice.get_service_list()
        self.service_names = rosservice.get_service_list()#[x[0] for x in rosgraph.masterapi.Master('/no_go_nodes_stop').getSystemState()[2]]
        #print service_names
        for i in self.stop_services:
            if i in self.service_names:
                self.av_stop_services.append(i)
        
        for i in self.activate_services :
            if i in self.service_names :
                self.av_activate_services.append(i)



    def create_action_clients(self):
        if not self.monit_nav_cli:
            rospy.loginfo("Creating monitored navigation client.")
            self.monNavClient= actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
            self.monit_nav_cli=self.monNavClient.wait_for_server(timeout = rospy.Duration(1))
            if self.monit_nav_cli:
                rospy.loginfo(" ...done")
            else:
                rospy.logwarn("monitored navigation client could not be created will retry afterwards")
        if not self.top_nav_cli:
            rospy.loginfo("Creating topological navigation client.")
            self.topNavClient = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
            self.top_nav_cli = self.topNavClient.wait_for_server(timeout = rospy.Duration(1))
            if self.top_nav_cli:
                rospy.loginfo(" ...done")
            else:
                rospy.logwarn("topological navigation client could not be created will retry afterwards")
        if not self.exec_pol_cli:
            rospy.loginfo("Creating execute policy client.")
            self.execPolClient = actionlib.SimpleActionClient('/topological_navigation/execute_policy_mode',ExecutePolicyModeAction)
            self.exec_pol_cli = self.execPolClient.wait_for_server(timeout = rospy.Duration(1))
            if self.exec_pol_cli:
                rospy.loginfo(" ...done")
            else:
                rospy.logwarn("execute policy client could not be created will retry afterwards")    



    def safety_stop_cb(self, req):
        self.service_called='*Safety Stop*'
        self.safety_stop = True

    def reset_safety_stop_cb(self, req):
        self.safety_stop = False

    """
     Get Safety_Nodes
     
     This function gets the list of No go nodes
    """
    def goto_safety_cb(self, req):
        self.service_called='*Go to Safety Point*'
        self.update_service_list()
        #nodes=self.get_safety_nodes()
        self.info=''
        #if nodes:
        #    print nodes[0]
        rospy.loginfo("Pause task executor")
        self.info= self.info + ' Pause task executor\n'
        self.start_stop_scheduler(False)
        self.cancel_all_goals()
        self.go_to_node(self.closest_node)
#            if not self.go_to_node(nodes[0]):
#                cinfo = "Navigation Failed stuck somewhere close to %s" %self.closest_node
#                rospy.loginfo(cinfo)
#                self.info= self.info + ' ' +cinfo + '\n'
#            else:
        cinfo = "Stop at %s" %self.closest_node
        rospy.loginfo(cinfo)
        self.info= self.info + ' ' +cinfo + '\n'
        self.info= self.info + ' Set Emergency Stop\n'
        rospy.loginfo("Set Emergency Stop")
        self.set_emergency_stop()
        self.info= self.info + ' Set Free Run\n'
        rospy.loginfo("Set Free Run")
        self.set_free_run(False)
#        else :
#            self.info= self.info + ' Can\'t Find any Safety Points no action taken\n'
#            rospy.logerr("Can't Find any Safety Points no action taken")
        self.send_email()
        return []




    def get_safety_nodes(self):
        try:
            rospy.wait_for_service('/topological_localisation/get_nodes_with_tag')
            get_nodes = rospy.ServiceProxy('/topological_localisation/get_nodes_with_tag', strands_navigation_msgs.srv.GetTaggedNodes)
            resp1 = get_nodes('Safety_Point')
            #print resp1
            return resp1.nodes
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return []



    def go_to_node(self, node):
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
           
        self.client.wait_for_server()
        doing = "Requesting Navigation to %s" %node
        self.info= self.info + ' ' +doing+'\n'
        rospy.loginfo(doing)
               
        navgoal = topological_navigation.msg.GotoNodeGoal()       
        #print "Requesting Navigation to %s" %node
        navgoal.target = node
        
        self.client.send_goal(navgoal)
        self.client.wait_for_result()
        
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        return ps.success


    """
     Update Map CallBack
     
     This Function updates the Topological Map everytime it is called
    """
    def MapCallback(self, msg) :
        self.lnodes = msg
        self._map_received=True


    """
     Current Node CallBack
     
    """
    def cNodeCallback(self, msg):
        self.closest_node = msg.data
 


    def set_free_run(self, val):
        if "/enable_motors" in self.av_stop_services:
            try:
                rospy.wait_for_service('/enable_motors', timeout=0.5)
                sfrun = rospy.ServiceProxy('/enable_motors', rosservice.get_service_class_by_name('/enable_motors'))
                sfrun(val)
                print "Free run %s" %val
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
        else:
            rospy.logwarn('Couldn\'t Find Scheduler Free Run service')

        
    def start_stop_scheduler(self, val):
        if "/task_executor/set_execution_status" in self.av_stop_services:
            try:
                rospy.wait_for_service('/task_executor/set_execution_status', timeout=0.5)
                schstop = rospy.ServiceProxy('/task_executor/set_execution_status', rosservice.get_service_class_by_name('/task_executor/set_execution_status'))
                schstop(val)
                print "Schedule Execute %s" %val
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
        else:
            rospy.logwarn('Couldn\'t Find Scheduler Start/Stop service')            

    def set_emergency_stop(self):
        if '/emergency_stop' in self.av_stop_services:
            try:
                rospy.wait_for_service('/emergency_stop', timeout=0.5)
                stop = rospy.ServiceProxy('/emergency_stop', rosservice.get_service_class_by_name('/emergency_stop'))
                stop()
                print "stop"
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
        else:
            rospy.logwarn('Couldn\'t Find Emergency Stop service')
    
    def release_emergency_stop(self):
        if '/reset_motorstop' in self.av_activate_services :
            try:
                rospy.wait_for_service('/reset_motorstop', timeout=0.5)
                reset = rospy.ServiceProxy('/reset_motorstop', rosservice.get_service_class_by_name('/reset_motorstop'))
                reset()
                print "reset"
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
        else:
            rospy.logwarn('Couldn\'t Find Release Emergency Stop service')


    def send_email(self):
        client = actionlib.SimpleActionClient('strands_emails', strands_emails.msg.SendEmailAction)
        client.wait_for_server()
        goal = strands_emails.msg.SendEmailGoal()

        if self.service_called == '*Safety Stop*':
            ps='I recommend calling an expert to resume Normal operation a */reset_safety_stop* service call is needed'
        if self.service_called == '*Go to Safety Point*':
            ps='I recommend calling an expert to assess the situation and resume normal operation if possible'

        goal.text = "Hello,\nMy %s safety behaviour has been triggered I have taken the following actions:\n%s\n%s" %(self.service_called, self.info, ps)
        goal.to_address = self.notificate_to
        goal.subject = 'Robot Safety Behaviour Triggered'
        
        client.send_goal(goal)
        self.info = ''
        #print "sending email"
        
        # Prints out the result of executing the action
        #return client.get_result()  # A FibonacciResult

    def cancel_all_goals(self):
        if self.top_nav_cli:
            self.topNavClient.cancel_all_goals()
        if self.monit_nav_cli:
            self.monNavClient.cancel_all_goals()
        if self.exec_pol_cli:
            self.execPolClient.cancel_all_goals()


    def timer_callback(self):
        if self.safety_stop :
            self.set_emergency_stop()
            if not self.pre_active :
                self.update_service_list()
                self.set_free_run(False)
                self.start_stop_scheduler(False)
                self.cancel_all_goals()
                self.send_email()
                self.create_action_clients()
            self.pre_active = True
        else:
            if self.pre_active :
                self.update_service_list()
                self.release_emergency_stop()
                self.set_free_run(True)
                self.start_stop_scheduler(True)            
            self.pre_active = False
            
        if not self._killall_timers :
            t = Timer(1.0, self.timer_callback)
            t.start()


    def _on_node_shutdown(self):
        self._killall_timers=True


if __name__ == '__main__':
    rospy.init_node('safety_trigger')
    server = SafetyServer(rospy.get_name())
