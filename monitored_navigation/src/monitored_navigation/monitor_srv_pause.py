import rospy

from monitor_state import MonitorState

from std_msgs.msg import Bool
from strands_navigation_msgs.srv import PauseResumeNav



class MonitorSrvPause(MonitorState):
    def __init__(self, is_paused=False):
        
        #if is_paused monitors for resume, else monitors for pause
        self.is_paused=is_paused
        
        self.pub=rospy.Publisher("monitored_navigation/srv_pause_requested", Bool, queue_size=1)        
       
        MonitorState.__init__(self, "/monitored_navigation/srv_pause_requested", Bool,  self.monitor_cb)
        
        
    def execute(self, userdata):
        pause_service = rospy.Service('/monitored_navigation/pause_nav', PauseResumeNav, self.pause_service_cb)
        result=MonitorState.execute(self, userdata)
        pause_service.shutdown()
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return result
    
    
    def monitor_cb(self,  ud,  msg):
        if self.is_paused:
            return msg.data
        else:
            return not msg.data

 
    def pause_service_cb(self, req):
        self.service_paused=req.pause
        self.pub.publish(self.service_paused)
        return []

