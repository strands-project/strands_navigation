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
from strands_navigation_msgs.srv import GetEdgesBetweenNodes, GetEdgesBetweenNodesRequest
from strands_navigation_msgs.msg import TopologicalMap
import thread


class Manager(object):

    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        self.current_edge = 'none'
        self.current_node = 'none'
        self.alive = False
        self.white_list_edges = []
        white_list_file = rospy.get_param("~white_list_file", '')
        if white_list_file == '':
            rospy.logfatal("No white list for logging specified")
            raise rospy.ROSException("No white list for logging specified")
            return
        white_list = rosparam.load_file(white_list_file)[0][0]

        rospy.loginfo("Getting topological map name...")
        self.map_name = None
        self.map_sub = rospy.Subscriber(
            rospy.get_param("~topo_map_topic", "topological_map"),
            TopologicalMap,
            self.get_map_name
        )
        while not self.map_name and not rospy.is_shutdown():
            rospy.logwarn("Topologial map is not being published. Waiting...")
            rospy.sleep(1)
        rospy.loginfo(" ... got topological map: %s" % self.map_name)

        rospy.loginfo("Getting allowed nodes from file...")
        self.white_list_nodes = white_list["nodes"]
        rospy.loginfo(" ... got nodes: %s" % self.white_list_nodes)
        if not [x for x in self.white_list_nodes if "ALL" in x]:
            while not rospy.is_shutdown():
                try:
                    rospy.loginfo("Creating edge look-up service...")
                    s = rospy.ServiceProxy(
                        '/topological_map_manager/get_edges_between_nodes',
                        GetEdgesBetweenNodes
                    )
                    rospy.loginfo("... waiting for edge look-up service")
                    s.wait_for_service()
                    rospy.loginfo("... found edge look-up service.")
                    for idx, nodea in enumerate(self.white_list_nodes):
                        for nodeb in self.white_list_nodes[idx+1:]:
                            rospy.logdebug("%s <-> %s" % (nodea, nodeb))
                            req = GetEdgesBetweenNodesRequest()
                            req.nodea = nodea
                            req.nodeb = nodeb
                            resp = s(req)
                            if resp.ids_a_to_b and resp.ids_b_to_a:
                                self.white_list_edges.append(str(resp.ids_a_to_b[0])+'--'+self.map_name)
                                self.white_list_edges.append(str(resp.ids_b_to_a[0])+'--'+self.map_name)
                    break
                except rospy.ServiceException, e:
                    rospy.logwarn("Service call failed: %s. Retrying." % e)
                    rospy.sleep(1)
        else:
            self.white_list_edges = ['ALL']

        self.node_sub = rospy.Subscriber(
            rospy.get_param("~node_topic", rospy.get_namespace() +  "closest_node"),
            String,
            self.node_callback
        )
        self.edge_sub = rospy.Subscriber(
            rospy.get_param("~edge_topic", rospy.get_namespace() + "current_edge"),
            String,
            self.edge_callback
        )

        self.pub_bool = rospy.Publisher(
            rospy.get_param("~bool_publisher_topic", "~log"),
            Bool,
            queue_size=10
        )
        self.pub_bool_stamped = rospy.Publisher(
            rospy.get_param("~bool_stamped_publisher_topic", "~log_stamped"),
            LoggingManager,
            queue_size=10
        )

        rospy.sleep(1)
        heart_beat = rospy.Rate(rospy.get_param("~check_topic_rate", 1))
        thread.start_new_thread(self.check_topics,(self.node_sub, self.edge_sub, heart_beat))

        pub_rate = rospy.Rate(rospy.get_param("~publishing_rate", 30))
        thread.start_new_thread(self.publisher_thread,(pub_rate,))

    def get_map_name(self, msg):
        self.map_name = msg.name
        self.map_sub.unregister()

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
        if not self.white_list_edges:
            rospy.logfatal("No white list for logging specified")
            raise rospy.ROSException("No white list for logging specified")
            return

        while not rospy.is_shutdown():
            res = LoggingManager()
            res.header.stamp = rospy.Time.now()
            res.log = False
            if self.alive:
                if self.current_edge or self.current_node:
                    if type(self.white_list_nodes) == list:
                        if [x for x in self.white_list_nodes if self.current_node == x or "ALL" == x] \
                            or [x for x in self.white_list_edges if self.current_edge == x or "ALL" == x]:
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