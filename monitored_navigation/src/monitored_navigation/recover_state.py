import rospy
import smach
from mongo_logger import MonitoredNavEventClass

#A special state that can bet  activated/deactivated using the param server, and knows how to log monitored nav events to mongo. 
#Not all RecoverStateMachine states need to be subclasses of this one, but if you want to log stuff and/or activate/deactivate the 
#recover behaviour being implemented by the state, you should implement a subclass of this.
class RecoverState(smach.State):
    def __init__(self, name, outcomes=[], input_keys=[], output_keys=[], io_keys=[], is_active=True, max_recovery_attempts=1):
            outcomes.append("not_active")
            smach.State.__init__(self,
                                 outcomes=outcomes,
                                 input_keys=input_keys,
                                 output_keys=output_keys,
                                 io_keys=io_keys
                                 )
            self.name=name
            self.was_helped=False #in case of being a ask help instate, is true if a human interacted with the robot
            current_params=rospy.get_param("/monitored_navigation/recover_states", {}) #dict of the form {name: (is_active, max_recovery_attempts)}
            if not current_params.has_key(name):
                current_params[name]=(is_active, max_recovery_attempts)
                rospy.set_param("monitored_navigation/recover_states", current_params)
            self.n_tries=1
            
    def execute(self, userdata):
        self.was_helped=False
        #This method *cannot* be overriden by the child class
        (is_active, max_recovery_attempts)=rospy.get_param("monitored_navigation/recover_states")[self.name]
        if "n_fails" in self.get_registered_input_keys():
            self.n_tries=userdata.n_fails
        else:
            self.n_tries=1
        if is_active and self.n_tries<=max_recovery_attempts:
            nav_stat=MonitoredNavEventClass()
            if self.n_tries==1:
                nav_stat.initialize(recovery_mechanism=self.name, log_costmaps=True)
            else:
                nav_stat.initialize(recovery_mechanism=self.name, log_costmaps=False)
            result = self.active_execute(userdata)
            nav_stat.finalize(was_helped=self.was_helped,n_tries=self.n_tries)
            nav_stat.insert()
            return result
        else:
            rospy.logwarn("Recover state " + self.name + " is not active. Not executing.")
            return "not_active"
        
    def active_execute(self, userdata):
        #this method *needs* to be overriden by the child class
        rospy.logerr("The 'active_execute' method needs to be implemented by class " + self.name + ". This replaces the normal Smach.State 'execute' method.")
        return None
    

