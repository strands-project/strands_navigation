import rospy

from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalID, GoalStatusArray
from  movebase_state_service.srv import MovebaseStateService, MovebaseStateServiceRequest

from strands_navigation_msgs.msg import MonitoredNavEvent
from mongodb_store.message_store import MessageStoreProxy


#string failure #'global_plan_fail, local_plan_fail, bump_fail, stuck_on_carpet
#bool was_helped
#int8 n_help_requests
#geometry_msgs/PoseStamped event_start_pose
#geometry_msgs/PoseStamped pose event_end_pose
#actionlib_msgs/GoalID goal_id
#sensor_msgs/Image global_costmap_image
#sensor_msgs/Image local_costmap_image
#sensor_msgs/Image camera_image

class MonitoredNavEventClass:
    def __init__(self):
        self.nav_event=None
        self.pub=rospy.Publisher('monitored_navigation/monitored_nav_event', MonitoredNavEvent, queue_size=1)
        
        got_service=False
        while not got_service:
            try:
                rospy.wait_for_service('movebase_state_service', 1)
                got_service=True
            except rospy.ROSException,e:
                rospy.loginfo("Monited nav event logger is waiting for the movebase_state_service")
            if rospy.is_shutdown():
                return
        
        self.get_costmaps=rospy.ServiceProxy('movebase_state_service',  MovebaseStateService)
        
    def initialize(self, recovery_mechanism,log_costmaps=False):
        self.nav_event=MonitoredNavEvent(recover_mechanism=recovery_mechanism, event_start_time=rospy.get_rostime())
        self.nav_event.event_start_pose=rospy.wait_for_message("robot_pose", Pose , timeout=10.0)
        status_msg=rospy.wait_for_message("monitored_navigation/status", GoalStatusArray , timeout=10.0)
        self.nav_event.goal_id=status_msg.status_list[0].goal_id
        if log_costmaps:
            try:
                costmap_images=self.get_costmaps(MovebaseStateServiceRequest(save_to_disk=False))
                self.nav_event.global_costmap_image=costmap_images.global_costmap_image
                self.nav_event.local_costmap_image=costmap_images.local_costmap_image
                self.nav_event.camera_image=costmap_images.camera_image
            except Exception, e:
                rospy.logwarn("Error while getting costmap images for monitored nav event logging: " + str(e))

 
    def finalize(self, was_helped, n_tries):
        try:
            self.nav_event.was_helped=was_helped
            self.nav_event.event_end_time=rospy.get_rostime()
            self.nav_event.event_end_pose=rospy.wait_for_message("robot_pose", Pose , timeout=10.0)
            self.nav_event.n_help_requests=n_tries
        except Exception, e:
            rospy.logwarn("Error finalising mon nav event: " + str(e))
    
    def insert(self):
        log_to_db=rospy.get_param('log_mon_nav_events',True)
        try:
            if log_to_db:
                message_proxy=MessageStoreProxy(collection='monitored_nav_events')
                message_proxy.insert(self.nav_event)
            self.pub.publish(self.nav_event)
        except Exception, e:
            rospy.logwarn("Error inserting mon nav event in mongo: " + str(e))



