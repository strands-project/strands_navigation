#! /usr/bin/env python

import rospy

#import smach
#import smach_ros

#from scitos_msgs.srv import ResetMotorStop
#from scitos_msgs.srv import EnableMotors
#from std_srvs.srv import Empty
#from scitos_msgs.msg import MotorStatus
#from geometry_msgs.msg import Twist

from nav_msgs.msg import Path
from move_base_msgs.msg import *
import dynamic_reconfigure.client
from scitos_ptu.msg import *
from previous_positions_service.srv import PreviousPosition
from republish_pointcloud_service.srv import RepublishPointcloud
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose

import actionlib

from strands_navigation_msgs.srv import AskHelp, AskHelpRequest
from backtrack_behaviour.msg import BacktrackAction, BacktrackResult, BacktrackFeedback

#from logger import Loggable

# this file has the recovery states that will be used when some failures are
# detected. There is a recovery behaviour for move_base and another for when the
# bumper is pressed

class BacktrackServer(object):
    def __init__(self):
        rospy.init_node('backtrack_actionserver')

        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        self.move_base_action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_reconfig_client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
        
        self.global_plan = None
        self.last_global_plan_time = None
        rospy.Subscriber("/move_base/NavfnROS/plan" , Path, self.global_planner_checker_cb)
        
        self.feedback = BacktrackFeedback()
        self.result = BacktrackResult()
        
        self.server = actionlib.SimpleActionServer('do_backtrack', BacktrackAction, self.execute, False)
        self.server.start()
        
    def global_planner_checker_cb(self, msg):
        self.global_plan = msg
        self.last_global_plan_time = rospy.get_rostime()
    
    def stop_republish(self):
        try:
            republish_pointcloud = rospy.ServiceProxy('republish_pointcloud', RepublishPointcloud)
            republish_pointcloud(False, '', '', 0.0)
        except rospy.ServiceException, e:
            rospy.logwarn("Couldn't stop republish pointcloud service, returning failure.")
            return False
        return True
    
    def start_republish(self):
        try:
            republish_pointcloud = rospy.ServiceProxy('republish_pointcloud', RepublishPointcloud)
            republish_pointcloud(True, '/head_xtion/depth/points', '/move_base/head_subsampled', 0.05)
        except rospy.ServiceException, e:
            rospy.logwarn("Couldn't get republish pointcloud service, returning failure.")
            return False
        return True

    def pose_cb(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        if self.first_pose:
            self.first_x = self.x
            self.first_y = self.y
            self.first_pose = False

    def gone_too_far(self, meters_back):
        xdiff = self.x - self.first_x
        ydiff = self.y - self.first_y
        if xdiff*xdiff + ydiff*ydiff > (meters_back+0.2)*(meters_back+0.2):
            return True
        return False

    def reset_ptu(self):
        ptu_goal = PtuGotoGoal();
        ptu_goal.pan = 0
        ptu_goal.tilt = 0
        ptu_goal.pan_vel = 20
        ptu_goal.tilt_vel = 20
        self.ptu_action_client.send_goal(ptu_goal)
        #self.ptu_action_client.wait_for_result()

    def reset_move_base_pars(self):
        params = { 'max_vel_x' : self.max_vel_x, 'min_vel_x' : self.min_vel_x }
        config = self.move_base_reconfig_client.update_configuration(params)
                                                  
    def execute(self, goal):
        #back track in elegant fashion     
        try:
            previous_position = rospy.ServiceProxy('previous_position', PreviousPosition)
            meter_back = previous_position(goal.meters_back)
        except rospy.ServiceException, e:
            rospy.logwarn("Couldn't get previous position service, returning failure.")
            self.server.set_aborted()
            return # 'failure'
            
        print "Got the previous position: ", meter_back.previous_pose.pose.position.x, ", ", meter_back.previous_pose.pose.position.y, ", ",  meter_back.previous_pose.pose.position.z

        self.move_base_action_client.cancel_all_goals() # begin by cancelling others
        self.first_pose = True
        pose_sub = rospy.Subscriber("/robot_pose" , Pose, self.pose_cb)
        
        self.feedback.status = "Moving the PTU"
        self.server.publish_feedback(self.feedback)
        ptu_goal = PtuGotoGoal();
        ptu_goal.pan = 179
        ptu_goal.tilt = 30
        ptu_goal.pan_vel = 20
        ptu_goal.tilt_vel = 20
        self.ptu_action_client.send_goal(ptu_goal)
        self.ptu_action_client.wait_for_result()
        
        if self.server.is_preempt_requested():
            self.reset_ptu()
            pose_sub.unregister()
            self.service_preempt()
            return
            
        if self.start_republish() == False:
            pose_sub.unregister()
            self.server.set_aborted()
            return # 'failure'
            
        print "Managed to republish pointcloud."
        
        self.max_vel_x = rospy.get_param("/move_base/DWAPlannerROS/max_vel_x")
        self.min_vel_x = rospy.get_param("/move_base/DWAPlannerROS/min_vel_x")
        params = { 'max_vel_x' : 0.2, 'min_vel_x' : -0.2 }
        config = self.move_base_reconfig_client.update_configuration(params)
        
        self.feedback.status = "Moving the robot"
        self.server.publish_feedback(self.feedback)
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose = meter_back.previous_pose.pose
        move_goal.target_pose.header.frame_id = meter_back.previous_pose.header.frame_id
        #rospy.sleep(rospy.Duration.from_sec(1))
        #print movegoal
        self.move_base_action_client.send_goal(move_goal)
        status = self.move_base_action_client.get_state()
        while status == GoalStatus.PENDING or status == GoalStatus.ACTIVE:   
            status = self.move_base_action_client.get_state()
            if self.server.is_preempt_requested():
                self.stop_republish()
                self.reset_move_base_pars()
                self.reset_ptu()
                pose_sub.unregister()
                self.service_preempt()
                return
            if self.gone_too_far(goal.meters_back):
                self.stop_republish()
                self.reset_move_base_pars()
                self.reset_ptu()
                self.move_base_action_client.cancel_all_goals()
                pose_sub.unregister()
                self.server.set_aborted()
                return
            self.move_base_action_client.wait_for_result(rospy.Duration(0.2))
        
        self.reset_move_base_pars()
        pose_sub.unregister()
        
        if self.server.is_preempt_requested():
            self.stop_republish()
            self.service_preempt()
            return

        if self.stop_republish() == False:
            self.server.set_aborted()
            return # 'failure'
        
        self.reset_ptu()

        print "Reset PTU, move_base parameters and stopped republishing pointcloud."
        
        if self.server.is_preempt_requested():
            self.service_preempt()
            return
            
        if self.move_base_action_client.get_state() == GoalStatus.SUCCEEDED: #set the previous goal again
            self.result.status = 'reached_point'
            self.server.set_succeeded(self.result)
            return
        elif status == GoalStatus.PREEMPTED:
            self.service_preempt()
            return
        else:
            if (rospy.get_rostime() - self.last_global_plan_time < rospy.Duration.from_sec(1)) and (self.global_plan.poses == []):
                self.server.set_aborted()
                return # 'failure' # 'global_plan_failure'
            else: # 'local_plan_failure'
                self.result.status = 'local_plan_failure' # 'succeeded'
                self.server.set_succeeded(self.result)
                return      

    def service_preempt(self):
        #check if preemption is working
        self.move_base_action_client.cancel_all_goals()
        self.server.set_preempted()

if __name__ == '__main__':
    server = BacktrackServer()
    rospy.spin()
