#!/usr/bin/env python

import sys
import rospy
import math
import tf
import actionlib

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *

from strands_navigation_msgs.msg import TopologicalNode
from topological_navigation.topological_map import *

import topological_navigation.msg


class go_to_controllers(object):

    def __init__(self, map_name) :
        self.in_feedback=False
        #self.update_needed=False
        #self.topo_map = topological_map(map_name)
        self._goto_server = InteractiveMarkerServer(map_name+"_go_to")
        #self.update_map(map_name)
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")


    def update_map(self, map_name) :

        self.topo_map = topological_map(map_name)
        self._goto_server.clear()

        for i in self.topo_map.nodes :
            self.create_marker(i.name, i._get_pose(), i.name)


    def makeEmptyMarker(self, dummyBox=True ) :
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.scale = 1
        return int_marker


    def makeBox(self, msg ):
        marker = Marker()

        marker.type = Marker.ARROW
        marker.scale.x = msg.scale * 0.5
        marker.scale.y = msg.scale * 0.25
        marker.scale.z = msg.scale * 0.15
        marker.lifetime.secs = 3
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 1.0
        return marker


    def create_marker(self, marker_name, pos, marker_description="goto marker") :
        # create an interactive marker for our server
        
        marker = self.makeEmptyMarker()
        marker.name = marker_name
        #marker.description = marker_description

        control = InteractiveMarkerControl()
    
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        control.markers.append( self.makeBox( marker ) )
        marker.controls.append(control)
    
        self._goto_server.insert(marker, self.feedback_cb)
        self._goto_server.applyChanges()

        if pos is not None:
            pos.position.z=pos.position.z+0.15
            self._goto_server.setPose( marker.name, pos )
            self._goto_server.applyChanges()

       
    def reset_update(self) :
        self.reset_feedback()
        self.update_needed=False


    def reset_feedback(self) :
        self.in_feedback=False


    def feedback_cb(self, feedback):
        if not self.in_feedback and not self.update_needed :
            self.in_feedback=True
            print 'GOTO: '+str(feedback.marker_name)
            self.client.cancel_all_goals()
            navgoal = topological_navigation.msg.GotoNodeGoal()                
            navgoal.target = feedback.marker_name
            #navgoal.origin = orig
            # Sends the goal to the action server.
            self.client.send_goal(navgoal)
#            self.update_needed=True

    
    def clear():
        self._goto_server.clear()
        self._goto_server.applyChanges()

        
    def __del__(self):
        del self._goto_server
