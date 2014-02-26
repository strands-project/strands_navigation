# TODO: Add speaking back in here
import rospy

import smach
import smach_ros

from scitos_msgs.srv import ResetMotorStop
from scitos_msgs.srv import EnableMotors
from std_srvs.srv import Empty
from scitos_msgs.msg import MotorStatus
from geometry_msgs.msg import Twist

import actionlib

from strands_navigation_msgs.srv import AskHelp, AskHelpRequest

#from logger import Loggable

#import marathon_touch_gui.client


# this file has the recovery states that will be used when some failures are
# detected. There is a recovery behaviour for move_base and another for when the
# bumper is pressed



class RecoverMoveBase(smach.State):
    def __init__(self,max_move_base_recovery_attempts=5):
        smach.State.__init__(self,
                             # we need the number of move_base fails as
                             # incoming data from the move_base action state,
                             # because it is not possible for this recovery
                             # behaviour to check if it was succeeded
                             outcomes=['succeeded', 'failure', 'preempted'],
                             input_keys=['n_move_base_fails'],
                             output_keys=['n_move_base_fails'],
                             )
        #self.vel_pub = rospy.Publisher('/cmd_vel', Twist)
        #self._vel_cmd = Twist()
        #self._vel_cmd.linear.x = -0.1
        self.set_patroller_thresholds(max_move_base_recovery_attempts)
        

        self.enable_motors= rospy.ServiceProxy('enable_motors',
                                                  EnableMotors)
                                                  
        self.ask_help=rospy.ServiceProxy('monitored_navigation/ask_help', AskHelp)
        self.service_msg=AskHelpRequest()
        self.service_msg.failed_component=AskHelpRequest.NAVIGATION
                                                  
        #self.clear_costmap
                                                  
        #self.speaker=actionlib.SimpleActionClient('/speak', maryttsAction)
        #self.speak_goal= maryttsGoal()
        #self.speaker.wait_for_server()
        
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
        # move slowly backwards a bit. A better option might be to save the
        # latest messages received in cmd_vel and reverse them
        #for i in range(0, 20):
            #self.vel_pub.publish(self._vel_cmd)
            #if self.preempt_requested():
                #self.service_preempt()
                #return 'preempted'
            #rospy.sleep(0.2)
        
        self.isRecovered=False
            
        self.enable_motors(False)    
        rospy.sleep(0.2)
        # since there is no way to check if the recovery behaviour was
        # successful, we always go back to the move_base action state with
        # 'succeeded' until the number of failures treshold is reached
        if userdata.n_move_base_fails < self.MAX_MOVE_BASE_RECOVERY_ATTEMPTS:
            #self.get_logger().log_navigation_recovery_attempt(success=True,
             #                                                 attempts=userdata.n_move_base_fails)
                                                              
                                                              
            help_offered_monitor=rospy.Service('/monitored_navigation/help_offered', Empty, self.help_offered)
            help_done_monitor=rospy.Service('/monitored_navigation/help_finished', Empty, self.nav_recovered)            
            
            
            
            
            self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
            self.service_msg.interaction_service='/monitored_navigation/help_offered'
            try:
                self.ask_help(self.service_msg)
            except rospy.ServiceException, e:
                rospy.logwarn("No means of asking for human help available.")
            
            for i in range(0,40):
                if self.being_helped:
                    self.service_msg.interaction_status=AskHelpRequest.BEING_HELPED
                    self.service_msg.interaction_service='/monitored_navigation/help_finished'
                    try:
                        self.ask_help(self.service_msg)
                    except rospy.ServiceException, e:
                        rospy.logwarn("No means of asking for human help available.")
                    break
                rospy.sleep(1)       
            
            if self.being_helped:
                self.being_helped=False
                for i in range(0,60):
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
            help_offered_monitor.shutdown()
            help_done_monitor.shutdown()
            return 'succeeded'
        else:
            userdata.n_move_base_fails=0
           # self.get_logger().log_navigation_recovery_attempt(success=False,
            #                                                  attempts=userdata.n_move_base_fails)
            return 'failure'


            
    def set_patroller_thresholds(self, max_move_base_recovery_attempts):
        if max_move_base_recovery_attempts is not None:
            self.MAX_MOVE_BASE_RECOVERY_ATTEMPTS = max_move_base_recovery_attempts        
            
            
            
class RecoverBumper(smach.State):
    def __init__(self,max_bumper_recovery_attempts=5):
        smach.State.__init__(self,
                             outcomes=['succeeded']
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
        
        self.ask_help=rospy.ServiceProxy('monitored_navigation/ask_help', AskHelp)
        self.service_msg=AskHelpRequest()
        self.service_msg.failed_component=AskHelpRequest.BUMPER
        
        
        #self.speaker=actionlib.SimpleActionClient('/speak', maryttsAction)
        #self.speak_goal= maryttsGoal()
        #self.speaker.wait_for_server()
        
        
        self.set_patroller_thresholds(max_bumper_recovery_attempts)
        
        
        
    
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
        help_offered_monitor=rospy.Service('/monitored_navigation/help_offered', Empty, self.help_offered)
        help_done_monitor=rospy.Service('/monitored_navigation/help_finished', Empty, self.bumper_recovered)
        #displayNo = rospy.get_param("~display", 0)
        n_tries=1
        while True:
            if self.being_helped:
                #ver se isto ta bem
                self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
                self.service_msg.interaction_service='/monitored_navigation/help_offered'
                try:
                    self.ask_help(self.service_msg)
                except rospy.ServiceException, e:
                    rospy.logwarn("No means of asking for human help available.")
                for i in range(0,60):
                    if self.help_finished:
                        break
                    rospy.sleep(1)
                if not self.help_finished:
                    self.being_helped=False
                    self.service_msg.interaction_status=AskHelpRequest.BEING_HELPED
                    self.service_msg.interaction_service='/monitored_navigation/help_finished'
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
                    #self.speak_goal.text='Thank you! I will be on my way.'
                    #self.speaker.send_goal(self.speak_goal)
                    #self.speaker.wait_for_result()
                    help_done_monitor.shutdown()
                    help_offered_monitor.shutdown()
                    #self.get_logger().log_bump_count(n_tries)
                    #marathon_touch_gui.client.display_main_page(displayNo)
                    self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
                    self.service_msg.interaction_service='none'
                    try:
                        self.ask_help(self.service_msg)
                    except rospy.ServiceException, e:
                        rospy.logwarn("No means of asking for human help available.")
                    return 'succeeded' 
                else:
                    #self.speak_goal.text='Something is still wrong. Are you sure I am in a clear area?'
                    #self.speaker.send_goal(self.speak_goal)
                    #self.speaker.wait_for_result()
                    #on_completion = 'help_offered'
                    #service_prefix = '/patroller'
                    #marathon_touch_gui.client.bumper_stuck(displayNo, service_prefix, on_completion)
                    self.service_msg.interaction_status=AskHelpRequest.HELP_FAILED
                    self.service_msg.interaction_service='none'
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
                        help_done_monitor.shutdown()
                        help_offered_monitor.shutdown()
                        #self.get_logger().log_bump_count(n_tries)
                        self.service_msg.interaction_status=AskHelpRequest.HELP_FINISHED
                        self.service_msg.interaction_service='none'
                        try:
                            self.ask_help(self.service_msg)
                        except rospy.ServiceException, e:
                            rospy.logwarn("No means of asking for human help available.")
                        return 'succeeded' 
                    rospy.sleep(1)
                    #if n_tries>self.MAX_BUMPER_RECOVERY_ATTEMPTS:
                    #send email
                n_tries += 1
            
                #if n_tries==2:
                    #on_completion = 'help_offered'
                    #service_prefix = '/patroller'
                    #marathon_touch_gui.client.bumper_stuck(displayNo, service_prefix, on_completion)            
            
                if n_tries>1:
                    #self.speak_goal.text='My bumper is being pressed. Please release it so I can move on!'
                    #self.speaker.send_goal(self.speak_goal)
                    #self.speaker.wait_for_result()
                    self.service_msg.interaction_status=AskHelpRequest.ASKING_HELP
                    self.service_msg.interaction_service='/monitored_navigation/help_offered'
                    try:
                        self.ask_help(self.service_msg)
                    except rospy.ServiceException, e:
                        rospy.logwarn("No means of asking for human help available.")
	
            
            
   
            
            
    def set_patroller_thresholds(self, max_bumper_recovery_attempts):
        if max_bumper_recovery_attempts is not None:
            self.MAX_BUMPER_RECOVERY_ATTEMPTS = max_bumper_recovery_attempts
                

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
   #     os.system("rosservice call /ros_mary 'Recovering carpet stuck'")
        self._vel_cmd.linear.x=0.0
        self._vel_cmd.angular.z=0.0
        self.vel_pub.publish(self._vel_cmd)
        

        
        #check if behaviour was successful       
        if True:     
            return 'succeeded'
        else:
            return 'failure'
