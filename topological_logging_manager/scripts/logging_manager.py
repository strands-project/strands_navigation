#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 12:08:14 2015

@author: cdondrup
"""

import rospy
from std_msgs.msg import String, Bool
from topological_logging_manager.msg import LoggingManager
import thread


class Manager(object):

    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        self.current_edge = None
        self.current_node = None
        self.white_list = rospy.get_param("~white_list", None)
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

        self.rate = rospy.Rate(rospy.get_param("~publishing_rate", 30))
        thread.start_new_thread(self.publisher_thread,(self.rate,))

    def node_callback(self, msg):
        self.current_node = msg.data if not msg.data == "none" else None

    def edge_callback(self, msg):
        self.current_edge = msg.data if not msg.data == "none" else None

    def publisher_thread(self, rate):
        if not self.white_list:
            rospy.logfatal("No white list for logging specified")
            raise rospy.ROSException("No white list for logging specified")
            return

        while not rospy.is_shutdown():
            res = LoggingManager()
            res.header.stamp = rospy.Time.now()
            if self.current_edge or self.current_node:
                self.pub_bool.publish(True)
                res.log = True
                self.pub_bool_stamped.publish(res)
            else:
                self.pub_bool.publish(False)
                res.log = False
                self.pub_bool_stamped.publish(res)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("logging_manager")
    lm = Manager(rospy.get_name())
    rospy.spin()