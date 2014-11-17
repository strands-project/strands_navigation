import rospy

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalID, GoalStatusArray

from strands_navigation_msgs.msg import MonitoredNavEvent
from mongodb_store.message_store import MessageStoreProxy


#string failure #'global_plan_fail, local_plan_fail, bump_fail, stuck_on_carpet
#bool was_helped
#int8 n_help_requests
#geometry_msgs/PoseStamped event_start_pose
#geometry_msgs/PoseStamped pose event_end_pose
#actionlib_msgs/GoalID goal_id

class MonitoredNavEventClass:
    def __init__(self):
        self.nav_event=None
        self.pub=rospy.Publisher('/monitored_navigation/monitored_nav_event', MonitoredNavEvent)
        
    def initialize(self, recovery_mechanism):
        self.nav_event=MonitoredNavEvent(recover_mechanism=recovery_mechanism, event_start_time=rospy.get_rostime())
        self.nav_event.event_start_pose=rospy.wait_for_message("/robot_pose", Pose , timeout=10.0)
        status_msg=rospy.wait_for_message("/monitored_navigation/status", GoalStatusArray , timeout=10.0)
        self.nav_event.goal_id=status_msg.status_list[0].goal_id

 
    def finalize(self, was_helped, n_tries):
        self.nav_event.was_helped=was_helped
        self.nav_event.event_end_time=rospy.get_rostime()
        self.nav_event.event_end_pose=rospy.wait_for_message("/robot_pose", Pose , timeout=10.0)
        self.nav_event.n_help_requests=n_tries
    
    def insert(self):
        log_to_db=rospy.get_param('log_mon_nav_events',False)
        if log_to_db:
            message_proxy=MessageStoreProxy(collection='monitored_nav_events')
            message_proxy.insert(self.nav_event)
        self.pub.publish(self.nav_event)



