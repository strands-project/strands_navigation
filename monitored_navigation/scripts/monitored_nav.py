#! /usr/bin/env python
import sys
import yaml
import rospy

from smach_ros import ActionServerWrapper, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction

from monitored_navigation.navigation import HighLevelNav
from monitored_navigation.recover_state_machine import RecoverStateMachine
from monitored_navigation.monitor_state import MonitorState

from monitored_navigation.help_manager import HelpManager
from monitored_navigation.ui_helper import UIHelper

from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationActionGoal

from strands_navigation_msgs.srv import AddMonitorRecoveryPair, DelMonitorRecoveryPair, SetMonitorRecoveryPairs, SetNavRecovery, AddHelper, DelHelper, SetHelpers

   
class MonitoredNavigation:

    def __init__(self, filename, smach_viewer=True):
        
        self.smach_viewer=smach_viewer
        
        # Create the main state machine
        self.high_level_nav = HighLevelNav()        
        self.add_monitor_recovery_pair_srv=rospy.Service('/monitored_navigation/add_monitor_recovery_pair', AddMonitorRecoveryPair, self.add_monitor_recovery_pair_cb)
        self.add_monitor_recovery_pair_srv=rospy.Service('/monitored_navigation/del_monitor_recovery_pair', DelMonitorRecoveryPair, self.del_monitor_recovery_pair_cb)
        self.add_monitor_recovery_pair_srv=rospy.Service('/monitored_navigation/set_monitor_recovery_pairs', SetMonitorRecoveryPairs, self.set_monitor_recovery_pairs_cb)
        self.add_monitor_recovery_pair_srv=rospy.Service('/monitored_navigation/set_nav_recovery', SetNavRecovery, self.set_nav_recovery_cb)
        
        self.human_help=HelpManager()
        self.add_helper_srv=rospy.Service('/monitored_navigation/add_helper', AddHelper, self.add_helper_cb)
        self.del_helper_srv=rospy.Service('/monitored_navigation/del_helper', DelHelper, self.del_helper_cb)
        self.set_helperd_srv=rospy.Service('/monitored_navigation/set_helpers', SetHelpers, self.set_helpers_cb)
        
        
        if file_name is not None:
            self.load_yaml_config(file_name)
            
        
        if self.smach_viewer:
            self.sis = IntrospectionServer('monitored_navigation_sm', self.high_level_nav.high_level_sm, '/MON_NAV')
            self.sis.start()
        
        self.as_wrapper=ActionServerWrapper('monitored_navigation', MonitoredNavigationAction, self.high_level_nav.high_level_sm,
                        ['succeeded'], ['recovered_with_help', 'recovered_without_help', 'not_recovered_with_help', 'not_recovered_without_help'], ['preempted'],
                        goal_key = 'goal', result_key='result'
                        )
        self.as_wrapper.run_server()
        
    def load_yaml_config(self, file_name):
        try:
            stream = open(file_name, 'r')
            yaml_config=yaml.load(stream)
        except Exception, e:
            rospy.logwarn("Error loading yaml config. MonitoredNavigation state machine will be initialized without recovery behaviours. To provide a config file do 'rosrun monitored_navigation monitored_nav.py path_to_config_file'. The strands config yaml file is located in monitored_navigation/config/strands.yaml")
            return
        
        #set nav recovery
        if "nav_recovery" in yaml_config:
            recovery_dict=yaml_config["nav_recovery"]
            recovery=self.create_object(recovery_dict["package"], recovery_dict["recovery_file"], recovery_dict["recovery_class"])
            if isinstance(recovery,RecoverStateMachine):
                self.high_level_nav.set_nav_recovery(recovery)
            else:
                rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Nav Recovery will not be added to state machine.")
        else:
            rospy.logwarn("No nav recovery provided.")
        
        #set monitor/recovery pairs
        if "monitor_recovery_pairs" in yaml_config:
            monitor_recovery_pairs=yaml_config["monitor_recovery_pairs"]
            for monitor_recovery_pair in monitor_recovery_pairs:
                monitor=self.create_object(monitor_recovery_pair["package"], monitor_recovery_pair["monitor_file"], monitor_recovery_pair["monitor_class"])
                recovery=self.create_object(monitor_recovery_pair["package"], monitor_recovery_pair["recovery_file"], monitor_recovery_pair["recovery_class"])
                if not isinstance(monitor,MonitorState):
                    rospy.logwarn("The monitor state needs to be an instantiation of the MonitorState class. Monitor/Recovery pair with name " + monitor_recovery_pair["name"] + " will not be added to state machine.")
                elif not isinstance(recovery,RecoverStateMachine):
                    rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Monitor/Recovery pair with name " + monitor_recovery_pair["name"] + " will not be added to state machine.")
                else:
                    self.high_level_nav.add_monitor_recovery_pair(monitor, recovery, monitor_recovery_pair["name"])
        else:
            rospy.logwarn("No monitor/recovery pairs provided.")
                
        #set helpers
        if "human_help" in yaml_config:
            helpers=yaml_config["human_help"]
            for helper_def in helpers:
                helper=self.create_object(helper_def["package"], helper_def["helper_file"], helper_def["helper_class"])
                if not isinstance(helper, UIHelper):
                    rospy.logwarn("The helper needs to be an instantiation of the UIHelper class. Helper with name " + helper["name"] + " will not be added.")
                else:
                    self.human_help.add_helper(helper, helper_def["name"])
        else:
            rospy.logwarn("No interfaces for human help provided.")
        

    def create_object(self, package, filename, class_name):
        try:
            mod = __import__(package+ '.' + filename, fromlist=[class_name])
            klass=getattr(mod, class_name)
            return klass()
        except (ImportError, AttributeError, ValueError), e:
            rospy.logwarn("Import error: " + str(e))
            return None

    
    def add_helper_cb(self, req):
        helper=self.create_object(req.package, req.helper_file, req.helper_class)
        if not isinstance(helper,UIHelper):
            rospy.logwarn("The helper needs to be an instantiation of the UIHelper class. Helper with name " + req.name + " will not be added.")
            return False
        return self.human_help.add_helper(helper, req.name)
        
    def del_helper_cb(self, req):
        return self.human_help.del_helper(req.name)
        
    def set_helpers_cb(self, req):
        size=len(req.names)
        if size != len(req.packages):
            rospy.logwarn("Length of names list and packages list does not match. Human help interfaces will remain the same.")
            return False
        if size != len(req.helper_files):
            rospy.logwarn("Length of names list and helper files list does not match. Human help interfaces will remain the same.")
            return False
        if size != len(req.helper_classes):
            rospy.logwarn("Length of names list and helper classes list does not match.  Human help interfaces will remain the same.")
            return False
        succeeded=True
        name_list=[]
        helper_list=[]
        for name, package, helper_file, helper_class in zip(req.names, req.packages, req.helper_files, req.helper_classes):
            helper=self.create_object(package, helper_file, helper_class)
            if not isinstance(helper, UIHelper):
                rospy.logwarn("The helper needs to be an instantiation of the UIHelper class. Human help interface with name " + name + " will not be added.")
                succeeded=False
            else:
                if name in name_list:
                    rospy.logwarn("There is already a human help interface with name " + name + " to be added. Skipping...")
                    succeeded=False
                else:
                    name_list.append(name)
                    helper_list.append(helper)
        added=self.human_help.set_helpers(helper_list, name_list)
        return succeeded and added
    
 
    def add_monitor_recovery_pair_cb(self, req):
        if self.as_wrapper.wrapped_container.is_running():
            rospy.logwarn("Cannot edit monitored navigation state machine while it is running. Skipping...")
            return False
        monitor=self.create_object(req.package, req.monitor_file, req.monitor_class)
        recovery=self.create_object(req.package, req.recovery_file, req.recovery_class)
        if not isinstance(monitor,MonitorState):
            rospy.logwarn("The monitor state needs to be an instantiation of the MonitorState class. Monitor/Recovery pair with name " + req.name + " will not be added to state machine.")
            return False
        elif not isinstance(recovery,RecoverStateMachine):
            rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Monitor/Recovery pair with name " + req.name + " will not be added to state machine.")
            return False
        else:
            added=self.high_level_nav.add_monitor_recovery_pair(monitor, recovery, req.name)
            if added:
                self.reinit_as()
            return added

        
    def del_monitor_recovery_pair_cb(self, req):
        if self.as_wrapper.wrapped_container.is_running():
            rospy.logwarn("Cannot edit monitored navigation state machine while it is running. Skipping...")
            return False
        removed=self.high_level_nav.del_monitor_recovery_pair(req.name)
        if removed:
            self.reinit_as()
        return removed
        
    def set_monitor_recovery_pairs_cb(self, req):
        if self.as_wrapper.wrapped_container.is_running():
            rospy.logwarn("Cannot edit monitored navigation state machine while it is running. Skipping...")
            return False
        size=len(req.names)
        if size != len(req.packages):
            rospy.logwarn("Length of names list and packages list does not match. Monitor and recovery behaviours will remain the same.")
            return False
        if size != len(req.monitor_files):
            rospy.logwarn("Length of names list and monitor files list does not match. Monitor and recovery behaviours will remain the same.")
            return False
        if size != len(req.monitor_classes):
            rospy.logwarn("Length of names list and monitor classes list does not match. Monitor and recovery behaviours will remain the same.")
            return False
        if size != len(req.recovery_files):
            rospy.logwarn("Length of names list and recovery files list does not match. Monitor and recovery behaviours will remain the same.")
            return False
        if size != len(req.recovery_classes):
            rospy.logwarn("Length of names list and recovery classes list does not match. Monitor and recovery behaviours will remain the same.")
            return False
        succeeded=True
        name_list=[]
        monitor_list=[]
        recovery_list=[]
        for name, package, monitor_file, monitor_class, recovery_file, recovery_class in zip(req.names, req.packages, req.monitor_files, req.monitor_classes, req.recovery_files, req.recovery_classes):
            monitor=self.create_object(package, monitor_file, monitor_class)
            recovery=self.create_object(package, recovery_file, recovery_class)
            if not isinstance(monitor,MonitorState):
                rospy.logwarn("The monitor state needs to be an instantiation of the MonitorState class. Monitor/Recovery pair with name " + name + " will not be added to state machine.")
                succeeded=False
            elif not isinstance(recovery,RecoverStateMachine):
                rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Monitor/Recovery pair with name " + name + " will not be added to state machine.")
                succeeded=False
            else:
                if name in name_list:
                    rospy.logwarn("There is already a Monitor/Recovery pair with name " + name + " to be added to the monitored navigation state machine. Skipping...")
                    succeeded=False
                else:
                    name_list.append(name)
                    monitor_list.append(monitor)
                    recovery_list.append(recovery)       
        added=self.high_level_nav.set_monitor_recovery_pairs(monitor_list, recovery_list, name_list)
        self.reinit_as()
        succeeded=succeeded and added
        return succeeded
            

    def set_nav_recovery_cb(self, req):
        if self.as_wrapper.wrapped_container.is_running():
            rospy.logwarn("Cannot edit monitored navigation state machine while it is running. Skipping...")
            return False
        recovery=self.create_object(req.package, req.recovery_file, req.recovery_class)
        if not isinstance(recovery,RecoverStateMachine):
            rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Nav Recovery will not be set.")
            return False
        added=self.high_level_nav.set_nav_recovery(recovery)
        if added:
            self.reinit_as()
        return added

    
    def reinit_as(self):
        if self.smach_viewer:
            self.sis.stop()
            self.sis = IntrospectionServer('monitored_navigation_sm', self.high_level_nav.high_level_sm, '/MON_NAV')
            self.sis.start()
        self.as_wrapper.wrapped_container=self.high_level_nav.high_level_sm
        
    
    def main(self):
        # Wait for control-c
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('monitored_navigation')
    
    filtered_argv=rospy.myargv(argv=sys.argv)
    
    if len(filtered_argv)<2:
        rospy.logwarn("No config yaml file provided. MonitoredNavigation state machine will be initialized without recovery behaviours. To provide a config file do 'rosrun monitored_navigation monitored_nav.py path_to_config_file'. The strands config yaml file is located in monitored_navigation/config/strands.yaml")
        file_name=None
    elif len(filtered_argv)>2:
        rospy.logwarn("Too many arguments. MonitoredNavigation state machine will be initialized without recovery behaviours. To provide a config file do 'rosrun monitored_navigation monitored_nav.py path_to_config_file'. The strands config yaml file is located in monitored_navigation/config/strands.yaml")
        file_name=None
    else:
        file_name=filtered_argv[1]

   
    mon_nav =  MonitoredNavigation(file_name)
    mon_nav.main()
