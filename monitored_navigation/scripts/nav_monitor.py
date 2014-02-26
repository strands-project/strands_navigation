#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from strands_navigation_msgs.msg import NavStatus
from std_srvs.srv import Empty




class NavMonitor(object):
    def __init__(self):
        self.goal_z=0
        self.current_z=0
        
        self.n_fails=0
        self.MAX_FAILS=100
        
        
        rospy.init_node('nav_monitor')
        
        #stuck in carpet
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)   
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        #preempt_nav
        self.pad_preempted=False
        rospy.Subscriber("/teleop_joystick/joy",Joy,self.pad_callback)
        
        self.pub = rospy.Publisher('/monitored_navigation/nav_status', NavStatus)
        self.pub_msg=NavStatus()
        
        #preempt service
        self.service_preempted=False        
        self.preempt_service = rospy.Service('/monitored_navigation/preempt_navigation', Empty, self.preempt_service_handler)



    def vel_callback(self,msg):
        self.goal_z=msg.angular.z
        if self.goal_z != 0 and self.current_z==0:
            self.n_fails=self.n_fails+1
        else:
            self.n_fails=0
        if self.n_fails>self.MAX_FAILS:
            self.pub_msg.carpet_stuck=True
        else:
            self.pub_msg.carpet_stuck=False
            

    def odom_callback(self,msg):
        self.current_z=msg.twist.twist.angular.z
        
        
    def pad_callback(self,msg):
        if msg.buttons[4]==0:
            self.pad_preempted=False
        else:
            self.pad_preempted=True
            
    def preempt_service_handler(self, req):
        self.service_preempted=True
        return []

            
    

    def publisher(self):
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.pub_msg.is_preempted=self.pad_preempted or self.service_preempted
            self.service_preempted=False
            self.pad_preempted=False
            self.pub.publish(self.pub_msg)
            r.sleep()
            
            
    

    
    
    
if __name__ == '__main__':


    monitor=NavMonitor()
    
    monitor.publisher()
    
    
    rospy.spin()    
