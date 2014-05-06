import rospy

import smach
import smach_ros

from scitos_msgs.srv import ResetMotorStop
from scitos_msgs.srv import EnableMotors
from std_srvs.srv import Empty
from scitos_msgs.msg import MotorStatus
from geometry_msgs.msg import Twist

from move_base_msgs.msg import *
import dynamic_reconfigure.client
from flir_pantilt_d46.msg import *
from previous_positions_service.srv import PreviousPosition
from republish_pointcloud_service.srv import RepublishPointcloud
from actionlib_msgs.msg import *

import actionlib

from strands_navigation_msgs.srv import AskHelp, AskHelpRequest

#from logger import Loggable



# this file has the recovery states that will be used when some failures are
# detected. There is a recovery behaviour for move_base and another for when the
# bumper is pressed

class RecoverNavBacktrack(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failure', 'preempted'],
                             input_keys=['goal','n_nav_fails'],
                             output_keys=['goal','n_nav_fails'],
                             )

        self.ptu_action_client = actionlib.SimpleActionClient('/SetPTUState', PtuGotoAction)
        self.move_base_action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_reconfig_client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
               
                                                  
    def execute(self, userdata):

        print "Failures: ", userdata.n_nav_fails
        if userdata.n_nav_fails < 2:
            #back track in elegant fashion
            
            try:
                previous_position = rospy.ServiceProxy('previous_position', PreviousPosition)
                meter_back = previous_position(1.0)
            except rospy.ServiceException, e:
                rospy.logwarn("Couldn't get previous position service, returning failure.")
                return 'failure'
                
            print "Got the previous position: ", meter_back.previous_pose.pose.position.x, ", ", meter_back.previous_pose.pose.position.y, ", ",  meter_back.previous_pose.pose.position.z
                
            try:
                republish_pointcloud = rospy.ServiceProxy('republish_pointcloud', RepublishPointcloud)
                republish_pointcloud(True, '/head_xtion/depth/points', '/move_base/head_subsampled', 0.05)
            except rospy.ServiceException, e:
                rospy.logwarn("Couldn't get republish pointcloud service, returning failure.")
                return 'failure'
                
            print "Managed to republish pointcloud."
                      
            params = { 'max_vel_x' : -0.1, 'min_vel_x' : -0.9 }
            config = self.move_base_reconfig_client.update_configuration(params)
            
            ptu_goal = PtuGotoGoal();
            ptu_goal.pan = -179
            ptu_goal.tilt = 50
            ptu_goal.pan_vel = 1
            ptu_goal.tilt_vel = 1
            self.ptu_action_client.send_goal(ptu_goal)
            self.ptu_action_client.wait_for_result()
            
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            move_goal = MoveBaseGoal()
            move_goal.target_pose.pose = meter_back.previous_pose.pose
            move_goal.target_pose.header.frame_id = meter_back.previous_pose.header.frame_id
            self.move_base_action_client.cancel_all_goals()
            rospy.sleep(rospy.Duration.from_sec(1))
            #print movegoal
            self.move_base_action_client.send_goal(move_goal)
            self.move_base_action_client.wait_for_result()
            
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            params = { 'max_vel_x' : 0.9, 'min_vel_x' : 0.1 }
            config = self.move_base_reconfig_client.update_configuration(params)
            
            ptu_goal = PtuGotoGoal();
            ptu_goal.pan = 0
            ptu_goal.tilt = 0
            ptu_goal.pan_vel = 1
            ptu_goal.tilt_vel = 1
            self.ptu_action_client.send_goal(ptu_goal)
            self.ptu_action_client.wait_for_result()
            
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            try:
                republish_pointcloud = rospy.ServiceProxy('republish_pointcloud', RepublishPointcloud)
                republish_pointcloud(False, '', '', 0.0)
            except rospy.ServiceException, e:
                rospy.logwarn("Couldn't stop republish pointcloud service, returning failure.")
                return 'failure'
                
            print "Reset PTU, move_base parameters and stopped republishing pointcloud."
                
            if self.move_base_action_client.get_state() != GoalStatus.SUCCEEDED: #set the previous goal again
                return 'failure'
            else:
                return 'succeeded'
        else:
            return 'failure'
        
      
            
    
    def service_preempt(self):
        #check if preemption is working
        smach.State.service_preempt(self)



class RecoverNavHelp(smach.State):
    def __init__(self,max_nav_recovery_attempts=5):
        smach.State.__init__(self,
                             # we need the number of move_base fails as
                             # incoming data from the move_base action state,
                             # because it is not possible for this recovery
                             # behaviour to check if it was succeeded
                             outcomes=['succeeded', 'failure', 'preempted'],
                             input_keys=['goal','n_nav_fails'],
                             output_keys=['goal','n_nav_fails'],
                             )

        self.set_nav_thresholds(max_nav_recovery_attempts)
        

        self.enable_motors= rospy.ServiceProxy('enable_motors',
                                                  EnableMotors)
                                                  
        self.ask_help=rospy.ServiceProxy('/monitored_navigation/human_help/manager', AskHelp)
        self.service_msg=AskHelpRequest()
        self.service_msg.failed_component=AskHelpRequest.NAVIGATION
                                                  
        #self.clear_costmap
                                                  

        
        self.being_helped=False
        self.help_finished=False      
                                                  

   
    def help_offered(self, req):
        self.being_helped=True
        return []
    
        
    def nav_recovered(self,req):
        self.being_helped=False
        self.help_finished=True
        return []
        
                                                  
    def execute(self, userdata):

        
        self.isRecovered=False
            
        self.enable_motors(False)    
        rospy.sleep(0.2)
        # since there is no way to check if the recovery behaviour was
        # successful, we always go back to the move_base action state with
        # 'succeeded' until the number of failures treshold is reached
        if userdata.n_nav_fails < self.MAX_NAV_RECOVERY_ATTEMPTS:
            #self.get_logger().log_navigation_recovery_attempt(success=True,
             #                                                 attempts=userdata.n_move_base_fails)
                                                              
                                                              
            self.help_offered_monitor=rospy.Service('/monitored_navigation/help_offered', Empty, self.help_offered)
            self.help_done_monitor=rospy.Service('/monitored_navigation/help_finished', Empty, self.nav_recovered)            
            
            
            
            
            self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
            self.service_msg.interaction_service='help_offered'
            try:
                self.ask_help(self.service_msg)
            except rospy.ServiceException, e:
                rospy.logwarn("No means of asking for human help available.")
            
            for i in range(0,40):
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                if self.being_helped:
                    self.service_msg.interaction_status=AskHelpRequest.BEING_HELPED
                    self.service_msg.interaction_service='help_finished'
                    try:
                        self.ask_help(self.service_msg)
                    except rospy.ServiceException, e:
                        rospy.logwarn("No means of asking for human help available.")
                    break
                rospy.sleep(1)       
            
            if self.being_helped:
                self.being_helped=False
                for i in range(0,60):
                    if self.preempt_requested():
                        self.service_preempt()
                        return 'preempted'
                    if self.help_finished:
                        #self.get_logger().log_helped("navigation")
                        self.help_finished=False
                        break
                    rospy.sleep(1)   
            
            self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
            self.service_msg.interaction_service='none'
            try:
                self.ask_help(self.service_msg)
            except rospy.ServiceException, e:
                rospy.logwarn("No means of asking for human help available.")
            self.help_offered_monitor.shutdown()
            self.help_done_monitor.shutdown()
            return 'succeeded'
        else:
            userdata.n_nav_fails=0
           # self.get_logger().log_navigation_recovery_attempt(success=False,
            #                                                  attempts=userdata.n_move_base_fails)
            return 'failure'


            
    def set_nav_thresholds(self, max_nav_recovery_attempts):
        if max_nav_recovery_attempts is not None:
            self.MAX_NAV_RECOVERY_ATTEMPTS = max_nav_recovery_attempts        
            
    
    def service_preempt(self):
        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
        self.service_msg.interaction_service='none'
        try:
            self.ask_help(self.service_msg)
        except rospy.ServiceException, e:
            rospy.logwarn("No means of asking for human help available.")
        self.help_offered_monitor.shutdown()
        self.help_done_monitor.shutdown()
        smach.State.service_preempt(self)
            
class RecoverBumper(smach.State):
    def __init__(self,max_bumper_recovery_attempts=5):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failure', 'preempted']
                             )
        self.reset_motorstop = rospy.ServiceProxy('reset_motorstop',
                                                  ResetMotorStop)
        self.enable_motors= rospy.ServiceProxy('enable_motors',
                                                  EnableMotors)                                          
        self.being_helped = False
        self.help_finished=False
        self.motor_monitor = rospy.Subscriber("/motor_status",
                                              MotorStatus,
                                              self.bumper_monitor_cb)
        
        self.ask_help=rospy.ServiceProxy('/monitored_navigation/human_help/manager', AskHelp)
        self.service_msg=AskHelpRequest()
        self.service_msg.failed_component=AskHelpRequest.BUMPER
        
        
        
        self.set_nav_thresholds(max_bumper_recovery_attempts)
        
        
        
    
    def help_offered(self, req):
        self.being_helped=True
        return []
    
        
    def bumper_recovered(self,req):
        self.being_helped=False
        self.help_finished=True
        return []
        
    def bumper_monitor_cb(self, msg):
        self.isRecovered = not msg.bumper_pressed

    # restarts the motors and check to see of they really restarted.
    # Between each failure the waiting time to try and restart the motors
    # again increases. This state can check its own success
    def execute(self, userdata):
        self.help_offered_monitor=rospy.Service('/monitored_navigation/help_offered', Empty, self.help_offered)
        self.help_done_monitor=rospy.Service('/monitored_navigation/help_finished', Empty, self.bumper_recovered)
        n_tries=1
        while True:
            
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if self.being_helped:
                #ver se isto ta bem
                self.service_msg.interaction_status=AskHelpRequest.BEING_HELPED
                self.service_msg.interaction_service='help_finished'
                try:
                    self.ask_help(self.service_msg)
                except rospy.ServiceException, e:
                    rospy.logwarn("No means of asking for human help available.")
                for i in range(0,60):
                    if self.preempt_requested():
                        self.service_preempt()
                        return 'preempted'
                    if self.help_finished:
                        break
                    rospy.sleep(1)
                if not self.help_finished:
                    self.being_helped=False
                    self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
                    self.service_msg.interaction_service='help_offered'
                    try:
                        self.ask_help(self.service_msg) 
                    except rospy.ServiceException, e:
                        rospy.logwarn("No means of asking for human help available.")
            elif self.help_finished:
                self.help_finished=False
                self.reset_motorstop()    
                rospy.sleep(0.1)
                if self.isRecovered:
                   # self.get_logger().log_helped("bumper")
                    self.help_done_monitor.shutdown()
                    self.help_offered_monitor.shutdown()
                    #self.get_logger().log_bump_count(n_tries)
                    self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
                    self.service_msg.interaction_service='none'
                    try:
                        self.ask_help(self.service_msg)
                    except rospy.ServiceException, e:
                        rospy.logwarn("No means of asking for human help available.")
                    return 'succeeded' 
                else:
                    self.service_msg.interaction_status=AskHelpRequest.HELP_FAILED
                    self.service_msg.interaction_service='help_offered'
                    try:
                        self.ask_help(self.service_msg)
                    except rospy.ServiceException, e:
                        rospy.logwarn("No means of asking for human help available.")
            else:  
                for i in range(0,4*n_tries):
                    if self.being_helped:
                        break
                    self.enable_motors(False)
                    self.reset_motorstop()
                    rospy.sleep(0.1)
                    if self.isRecovered:
                        self.help_done_monitor.shutdown()
                        self.help_offered_monitor.shutdown()
                        #self.get_logger().log_bump_count(n_tries)
                        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
                        self.service_msg.interaction_service='none'
                        try:
                            self.ask_help(self.service_msg)
                        except rospy.ServiceException, e:
                            rospy.logwarn("No means of asking for human help available.")
                        return 'succeeded' 
                    rospy.sleep(1)
                    if n_tries>self.MAX_BUMPER_RECOVERY_ATTEMPTS:
                        return 'failure'
                n_tries += 1
            
       
            
                if n_tries>1:
                    self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
                    self.service_msg.interaction_service='help_offered'
                    try:
                        self.ask_help(self.service_msg)
                    except rospy.ServiceException, e:
                        rospy.logwarn("No means of asking for human help available.")
	
            
            
   
            
            
    def set_nav_thresholds(self, max_bumper_recovery_attempts):
        if max_bumper_recovery_attempts is not None:
            self.MAX_BUMPER_RECOVERY_ATTEMPTS = max_bumper_recovery_attempts
                

    def service_preempt(self):
        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
        self.service_msg.interaction_service='none'
        try:
            self.ask_help(self.service_msg)
        except rospy.ServiceException, e:
            rospy.logwarn("No means of asking for human help available.")
        self.help_offered_monitor.shutdown()
        self.help_done_monitor.shutdown()
        smach.State.service_preempt(self)           
                
                
                
                
                
class RecoverStuckOnCarpet(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded','failure'])
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        self._vel_cmd = Twist()
        
        


    def execute(self,userdata):
        #small forward vel to unstuck robot
        self._vel_cmd.linear.x=0.8
        self._vel_cmd.angular.z=0.4
        for i in range(0,4): 
            self.vel_pub.publish(self._vel_cmd)
            self._vel_cmd.linear.x=self._vel_cmd.linear.x-0.2
            self._vel_cmd.angular.z=self._vel_cmd.angular.z-0.2  
   #         if self.preempt_requested():
   #             self.service_preempt()
   #             return 'preempted'
            rospy.sleep(0.2)
        self._vel_cmd.linear.x=0.0
        self._vel_cmd.angular.z=0.0
        self.vel_pub.publish(self._vel_cmd)
        

        
        #check if behaviour was successful       
        if True:     
            return 'succeeded'
        else:
            return 'failure'
