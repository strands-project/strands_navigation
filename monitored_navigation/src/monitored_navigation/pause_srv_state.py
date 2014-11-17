import rospy
import smach

from monitor_srv_pause import MonitorSrvPause
from recover_state_machine import RecoverStateMachine

from mongo_logger import MonitoredNavEventClass 




class PauseSrvState(RecoverStateMachine):
    def __init__(self):
        
        self.nav_stat=None
        
        RecoverStateMachine.__init__(self)
        self.resume_srv_monitor=MonitorSrvPause(is_paused=True)
        
        with self:
            smach.StateMachine.add('PAUSE_SRV_STATE',
                                   self.resume_srv_monitor,
                                   transitions={'preempted':'preempted',
                                                'invalid':'recovered_without_help',
                                                'valid':'recovered_without_help'})
                                                
    def execute(self, userdata):
        self.nav_stat=MonitoredNavEventClass()
        self.nav_stat.initialize(recovery_mechanism="srv_paused")
        result=smach.StateMachine.execute(self, userdata)
        self.nav_stat.finalize(was_helped=False,n_tries=0)
        self.nav_stat.insert()
        return result
 
