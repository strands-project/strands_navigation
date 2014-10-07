import rospy


from strands_navigation_msgs.msg import MonitoredNavEvent
from mongodb_store.message_store import MessageStoreProxy


#string failure #'global_plan_fail, local_plan_fail, bump_fail, stuck_on_carpet
#bool was_helped
#int8 n_help_requests
#geometry_msgs/PoseStamped event_start_pose
#geometry_msgs/PoseStamped pose event_end_pose
#actionlib_msgs/GoalID goal_id



def add_event(event):
    message_proxy=MessageStoreProxy(collection='monitored_nav_events')
    message_proxy.insert(event)
    



