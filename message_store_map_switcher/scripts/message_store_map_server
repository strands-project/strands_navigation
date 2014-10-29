#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
import StringIO
from optparse import OptionParser
from nav_msgs.msg import OccupancyGrid
from message_store_map_switcher.srv import SwitchMap

class MessageStoreMapServer:
    def __init__(self):
        self.msg_store = MessageStoreProxy()
        self.map_publisher = rospy.Publisher('/map', OccupancyGrid, latch=True)
        self.publish_map_serv = rospy.Service('/switch_map', SwitchMap, self.serve_map_srv)

    def serve_map_srv(self, req):
        return self.serve_map(req.map_name)         

    def serve_map(self, name):
        try:
            query_result = self.msg_store.query_named(name, OccupancyGrid._type, single=True)
            if query_result[0] is None:
                rospy.logwarn("No map found with name \"%s\"", name)
                return False
            else:
                self.map_publisher.publish(query_result[0])
                return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

if __name__ == '__main__':
    rospy.init_node("message_store_map_server")

    parser = OptionParser()
    parser.add_option("-d", "--default", dest="map_name", help="name of default map to publish")
    (options, args) = parser.parse_args()
    map_name = rospy.get_param("default_map", options.map_name)


    map_server = MessageStoreMapServer()

    if map_name is None :
        rospy.logwarn("No default map set via -d/--default command line, or ros param \"default_map\", so no map being published")
    else:
        # serve the default map
        map_server.serve_map(map_name)
    
    rospy.spin()
