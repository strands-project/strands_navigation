#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 12:08:14 2015

@author: cdondrup
"""

import rospy
import rosparam
from std_msgs.msg import String, Bool
from topological_logging_manager.msg import LoggingManager
import thread


class Manager(object):

    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        self.current_edge = 'none'
        self.current_node = 'none'
        self.alive = False

        white_list_file = rospy.get_param("~white_list_file", '')
        if white_list_file == '':
            rospy.logfatal("No white list for logging specified")
            raise rospy.ROSException("No white list for logging specified")
            return
        self.white_list = rosparam.load_file(white_list_file)[0][0]

        self.node_sub = rospy.Subscriber(
            rospy.get_param("~node_topic", "/current_node"),
            String,
            self.node_callback
        )
        self.edge_sub = rospy.Subscriber(
            rospy.get_param("~edge_topic", "/current_edge"),
            String,
            self.edge_callback
        )

        self.pub_bool = rospy.Publisher(
            rospy.get_param("~bool_publisher_topic", "~log"),
            Bool,
            queue_size=10
        )
        self.pub_bool_stamped = rospy.Publisher(
            rospy.get_param("~bool_publisher_topic", "~log_stamped"),
            LoggingManager,
            queue_size=10
        )

        heart_beat = rospy.Rate(rospy.get_param("~check_topic_rate", 1))
        thread.start_new_thread(self.check_topics,(self.node_sub, self.edge_sub, heart_beat))

        pub_rate = rospy.Rate(rospy.get_param("~publishing_rate", 30))
        thread.start_new_thread(self.publisher_thread,(pub_rate,))

    def node_callback(self, msg):
        self.current_node = msg.data

    def edge_callback(self, msg):
        self.current_edge = msg.data

    def check_topics(self, sub1, sub2, rate):
        while not rospy.is_shutdown():
            self.alive = True if sub1.get_num_connections() and sub2.get_num_connections() else False
            if not self.alive:
                rospy.logwarn("Subscribed topics are not being published.")
            rate.sleep()

    def publisher_thread(self, rate):
        if not self.white_list:
            rospy.logfatal("No white list for logging specified")
            raise rospy.ROSException("No white list for logging specified")
            return

        while not rospy.is_shutdown():
            res = LoggingManager()
            res.header.stamp = rospy.Time.now()
            res.log = False
            if self.alive:
                if self.current_edge or self.current_node:
                    if type(self.white_list) == dict:
                        if [x for x in self.white_list["nodes"] if self.current_node in x or "ALL" in x] \
                            or [x for x in self.white_list["edges"] if self.current_edge in x or "ALL" in x]:
                                res.log = True
            else:
                if not self.current_edge == "none" or self.current_node == "none":
                    self.current_edge = "none"
                    self.current_node = "none"

            self.pub_bool.publish(res.log)
            self.pub_bool_stamped.publish(res)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("logging_manager")
    lm = Manager(rospy.get_name())
    rospy.spin()