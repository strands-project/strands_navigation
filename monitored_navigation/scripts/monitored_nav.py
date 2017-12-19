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

from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationActionGoal, DynClassLoaderDef

from strands_navigation_msgs.srv import AddMonitorRecoveryPair, DelMonitorRecoveryPair, SetMonitorRecoveryPairs, SetNavRecovery, AddHelper, DelHelper, SetHelpers, GetMonitorRecoveryPairs, GetNavRecovery, GetHelpers,GetMonitorRecoveryPairsResponse, GetNavRecoveryResponse, GetHelpersResponse

   
class MonitoredNavigation:

    def __init__(self, filename, smach_viewer=True):
        
        self.smach_viewer=smach_viewer
        
        # Create the main state machine
        self.high_level_nav = HighLevelNav()        
        self.add_monitor_recovery_pair_srv=rospy.Service('/monitored_navigation/add_monitor_recovery_pair', AddMonitorRecoveryPair, self.add_monitor_recovery_pair_cb)
        self.del_monitor_recovery_pair_srv=rospy.Service('/monitored_navigation/del_monitor_recovery_pair', DelMonitorRecoveryPair, self.del_monitor_recovery_pair_cb)
        self.set_monitor_recovery_pairs_srv=rospy.Service('/monitored_navigation/set_monitor_recovery_pairs', SetMonitorRecoveryPairs, self.set_monitor_recovery_pairs_cb)
        self.get_monitor_recovery_pairs_srv=rospy.Service('/monitored_navigation/get_monitor_recovery_pairs', GetMonitorRecoveryPairs, self.get_monitor_recovery_pairs_cb)
        
        self.set_nav_recovery_srv=rospy.Service('/monitored_navigation/set_nav_recovery', SetNavRecovery, self.set_nav_recovery_cb)
        self.get_nav_recovery_srv=rospy.Service('/monitored_navigation/get_nav_recovery', GetNavRecovery, self.get_nav_recovery_cb)
        
        self.human_help=HelpManager()
        self.add_helper_srv=rospy.Service('/monitored_navigation/add_helper', AddHelper, self.add_helper_cb)
        self.del_helper_srv=rospy.Service('/monitored_navigation/del_helper', DelHelper, self.del_helper_cb)
        self.set_helpers_srv=rospy.Service('/monitored_navigation/set_helpers', SetHelpers, self.set_helpers_cb)
        self.get_helpers_srv=rospy.Service('/monitored_navigation/get_helpers', GetHelpers, self.get_helpers_cb)

        self.current_monitor_recovery_pair_names=[]
        self.current_monitor_defs=[]
        self.current_recovery_defs=[]
        self.current_nav_recovery_def=None
        self.current_helper_names=[]
        self.current_helper_defs=[]
        
        self.created_dyn_defs=[]
        self.created_dyn_objects=[]
        
        if file_name is not None:
            self.load_yaml_config(file_name)
            
        
        if self.smach_viewer:
            self.sis = IntrospectionServer('monitored_navigation_sm', self.high_level_nav.high_level_sm, '/MON_NAV')
            self.sis.start()
        
        self.as_wrapper=ActionServerWrapper(rospy.get_namespace() + 'monitored_navigation', MonitoredNavigationAction, self.high_level_nav.high_level_sm,
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
            recovery_def=DynClassLoaderDef(recovery_dict["package"], recovery_dict["recovery_file"], recovery_dict["recovery_class"])
            recovery=self.create_object(recovery_def)
            if isinstance(recovery,RecoverStateMachine):
                self.high_level_nav.set_nav_recovery(recovery)
                self.set_nav_recovery_def(recovery_def)
            else:
                rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Nav Recovery will not be added to state machine.")
        else:
            rospy.logwarn("No nav recovery provided.")
        
        #set monitor/recovery pairs
        if "monitor_recovery_pairs" in yaml_config:
            monitor_recovery_pairs=yaml_config["monitor_recovery_pairs"]
            for monitor_recovery_pair in monitor_recovery_pairs:
                name=monitor_recovery_pair["name"]
                monitor_def=DynClassLoaderDef(monitor_recovery_pair["package"], monitor_recovery_pair["monitor_file"], monitor_recovery_pair["monitor_class"])
                recovery_def=DynClassLoaderDef(monitor_recovery_pair["package"], monitor_recovery_pair["recovery_file"], monitor_recovery_pair["recovery_class"])
                monitor=self.create_object(monitor_def)
                recovery=self.create_object(recovery_def)
                if not isinstance(monitor,MonitorState):
                    rospy.logwarn("The monitor state needs to be an instantiation of the MonitorState class. Monitor/Recovery pair with name " + name + " will not be added to state machine.")
                elif not isinstance(recovery,RecoverStateMachine):
                    rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Monitor/Recovery pair with name " + name + " will not be added to state machine.")
                else:
                    self.high_level_nav.add_monitor_recovery_pair(monitor, recovery, name)
                    self.add_monitor_recovery_def(monitor_def,recovery_def,name)
        else:
            rospy.logwarn("No monitor/recovery pairs provided.")
                
        #set helpers
        if "human_help" in yaml_config:
            helpers=yaml_config["human_help"]
            for named_helper_def in helpers:
                helper_def=DynClassLoaderDef(named_helper_def["package"], named_helper_def["helper_file"], named_helper_def["helper_class"])
                name=named_helper_def["name"]
                helper=self.create_object(helper_def)
                if not isinstance(helper, UIHelper):
                    rospy.logwarn("The helper needs to be an instantiation of the UIHelper class. Helper with name " + helper["name"] + " will not be added.")
                else:
                    self.human_help.add_helper(helper, name)
                    self.add_helper_def(helper_def,name)
        else:
            rospy.logwarn("No interfaces for human help provided.")
        

    def create_object(self, dyn_class_loader_def):
        if dyn_class_loader_def in self.created_dyn_defs:
            return self.created_dyn_objects[self.created_dyn_defs.index(dyn_class_loader_def)]
        else:
            package=dyn_class_loader_def.object_package
            filename=dyn_class_loader_def.object_file
            class_name=dyn_class_loader_def.object_class
            try:
                mod = __import__(package+ '.' + filename, fromlist=[class_name])
                klass=getattr(mod, class_name)
                class_instance=klass()
                self.created_dyn_defs.append(dyn_class_loader_def)
                self.created_dyn_objects.append(class_instance)
                return class_instance
            except (ImportError, AttributeError, ValueError), e:
                rospy.logwarn("Import error: " + str(e))
                return None

    
    def add_helper_cb(self, req):
        helper=self.create_object(req.ui_helper)
        if not isinstance(helper,UIHelper):
            rospy.logwarn("The helper needs to be an instantiation of the UIHelper class. Helper with name " + req.name + " will not be added.")
            return False
        added=self.human_help.add_helper(helper, req.name)
        if added:
            self.add_helper_def(req.ui_helper, req.name)
        return added
        
    def del_helper_cb(self, req):
        deleted= self.human_help.del_helper(req.name)
        if deleted:
            self.del_helper_def(req.name)
        return deleted
        
    def set_helpers_cb(self, req):
        size=len(req.names)
        if size != len(req.ui_helpers):
            rospy.logwarn("Length of names list and help classes def list does not match. Human help interfaces will remain the same.")
            return False
        succeeded=True
        name_list=[]
        helper_list=[]
        for name, helper_def  in zip(req.names, req.ui_helpers):
            helper=self.create_object(helper_def)
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
        if succeeded and added:
            self.set_helper_defs(req.ui_helpers,req.names)
        return succeeded and added
    
 
    def add_monitor_recovery_pair_cb(self, req):
        if self.as_wrapper.wrapped_container.is_running():
            rospy.logwarn("Cannot edit monitored navigation state machine while it is running. Skipping...")
            return False
        monitor=self.create_object(req.monitor_state)
        recovery=self.create_object(req.recover_state_machine)
        if not isinstance(monitor,MonitorState):
            rospy.logwarn("The monitor state needs to be an instantiation of the MonitorState class. Monitor/Recovery pair with name " + req.name + " will not be added to state machine.")
            return False
        elif not isinstance(recovery,RecoverStateMachine):
            rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Monitor/Recovery pair with name " + req.name + " will not be added to state machine.")
            return False
        else:
            added=self.high_level_nav.add_monitor_recovery_pair(monitor, recovery, req.name)
            if added:
                self.add_monitor_recovery_def(req.monitor_state,req.recover_state_machine, req.name)
                self.reinit_as()
            return added

        
    def del_monitor_recovery_pair_cb(self, req):
        if self.as_wrapper.wrapped_container.is_running():
            rospy.logwarn("Cannot edit monitored navigation state machine while it is running. Skipping...")
            return False
        removed=self.high_level_nav.del_monitor_recovery_pair(req.name)
        if removed:
            self.del_monitor_recovery_def(req.name)
            self.reinit_as()
        return removed
        
    def set_monitor_recovery_pairs_cb(self, req):
        if self.as_wrapper.wrapped_container.is_running():
            rospy.logwarn("Cannot edit monitored navigation state machine while it is running. Skipping...")
            return False
        size=len(req.names)
        if size != len(req.monitor_states):
            rospy.logwarn("Length of names list and monitor defs list does not match. Monitor and recovery behaviours will remain the same.")
            return False
        if size != len(req.recovery_state_machines):
            rospy.logwarn("Length of names list and recovery sm defs list does not match. Monitor and recovery behaviours will remain the same.")
            return False
        succeeded=True
        name_list=[]
        monitor_list=[]
        recovery_list=[]
        for name, monitor_def, recovery_def in zip(req.names, req.monitor_states, req.recovery_state_machines):
            monitor=self.create_object(monitor_def)
            recovery=self.create_object(recovery_def)
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
        if succeeded:
            self.set_monitor_recovery_defs(req.monitor_states, req.recovery_state_machines, req.names)
        return succeeded
            

    def set_nav_recovery_cb(self, req):
        if self.as_wrapper.wrapped_container.is_running():
            rospy.logwarn("Cannot edit monitored navigation state machine while it is running. Skipping...")
            return False
        recovery=self.create_object(req.recovery_state_machine)
        if not isinstance(recovery,RecoverStateMachine):
            rospy.logwarn("The recovery state machine needs to be an instantiation of the RecoverStateMachine class. Nav Recovery will not be set.")
            return False
        added=self.high_level_nav.set_nav_recovery(recovery)
        if added:
            self.set_nav_recovery_def(req.recovery_state_machine)
            self.reinit_as()
        return added


    def add_monitor_recovery_def(self, monitor_def,recovery_def,name):
        self.current_monitor_defs.append(monitor_def)
        self.current_recovery_defs.append(recovery_def)
        self.current_monitor_recovery_pair_names.append(name)
        
    def del_monitor_recovery_def(self, name):
        index=self.current_monitor_recovery_pair_names.index(name)
        del self.current_monitor_defs[index]
        del self.current_recovery_defs[index]
        del self.current_monitor_recovery_pair_names[index]
        
    def set_monitor_recovery_defs(self, monitor_defs,recovery_defs,names):
        self.current_monitor_defs=monitor_defs
        self.current_recovery_defs=recovery_defs
        self.current_monitor_recovery_pair_names=names
        
    def set_nav_recovery_def(self,nav_recovery_def):
        self.current_nav_recovery_def=nav_recovery_def
        
        
    def add_helper_def(self, helper_def, name):
        self.current_helper_defs.append(helper_def)
        self.current_helper_names.append(name)
        
    def del_helper_def(self, name):
        index=self.current_helper_names.index(name)
        del self.current_helper_defs[index]
        del self.current_helper_names[index]
        
    def set_helper_defs(self, helper_defs,names):
        self.current_helper_defs=helper_defs
        self.current_helper_names=names
        
    def get_monitor_recovery_pairs_cb(self,req):
        return GetMonitorRecoveryPairsResponse(names=self.current_monitor_recovery_pair_names, monitor_states=self.current_monitor_defs, recovery_state_machines=self.current_recovery_defs)
    
    def get_nav_recovery_cb(self,req):
        return GetNavRecoveryResponse(recovery_state_machine=self.current_nav_recovery_def)
    
    def get_helpers_cb(self,req):
        return GetHelpersResponse(names=self.current_helper_names, ui_helpers=self.current_helper_defs)



    
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
