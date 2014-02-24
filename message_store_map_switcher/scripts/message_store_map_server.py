#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import ros_datacentre_msgs.srv as dc_srv
import ros_datacentre.util as dc_util
from ros_datacentre.message_store import MessageStoreProxy
import StringIO
from optparse import OptionParser
from nav_msgs.msg import OccupancyGrid

class MessageStoreMapServer:
    def __init__(self):
        self.msg_store = MessageStoreProxy()
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, latch=True)

    def serve_map(self, name):
        try:
            map = self.msg_store.query_named(name, OccupancyGrid._type)
            self.map_publisher.publish(map)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node("message_store_map_server")

    parser = OptionParser()
    parser.add_option("-d", "--default", dest="map_name", help="name of default map to publish")
    (options, args) = parser.parse_args()
    map_name = rospy.get_param("default_map", options.map_name)

    if map_name is None :
        parser.error("Default map name must be set by either -d/--default or via the default_map ros param")


    map_server = MessageStoreMapServer()
    map_server.serve_map(map_name)
    rospy.spin()
