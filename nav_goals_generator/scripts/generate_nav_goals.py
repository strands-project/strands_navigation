#!/usr/bin/env python
# import roslib; roslib.load_manifest('nav_goals_generator')
import rospy
import tf

from nav_goals_generator.srv import NavGoals, NavGoalsResponse
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import random
import math
import numpy
import sys

################################################################
# Ray-casting algorithm
#
# adapted from http://rosettacode.org/wiki/Ray-casting_algorithm
################################################################
_eps = 0.00001

def ray_intersect_seg(p, a, b):
    ''' takes a point p and an edge of two endpoints a,b of a line segment returns boolean
    '''
    if a.y > b.y:
        a,b = b,a
    if p.y == a.y or p.y == b.y:
        p = Point32(p.x, p.y + _eps, 0)

    intersect = False

    if (p.y > b.y or p.y < a.y) or (
        p.x > max(a.x, b.x)):
        return False

    if p.x < min(a.x, b.x):
        intersect = True
    else:
        if abs(a.x - b.x) > sys.float_info.min:
            m_red = (b.y - a.y) / float(b.x - a.x)
        else:
            m_red = sys.float_info.max
        if abs(a.x - p.x) > sys.float_info.min:
            m_blue = (p.y - a.y) / float(p.x - a.x)
        else:
            m_blue = sys.float_info.max
        intersect = m_blue >= m_red
    return intersect

def is_odd(x): return x%2 == 1

def is_inside(p, poly):
    ln = len(poly)
    num_of_intersections = 0
    for i in range(0,ln):
        num_of_intersections += ray_intersect_seg(p, poly[i], poly[(i + 1) % ln])

    return is_odd(num_of_intersections)

################################################################
# Navigation goal generator
################################################################

class NavGoalsGenerator():
	"A class for generation random poses for the robot"

	def __init__(self):
            rospy.init_node('generate_nav_goals')
            rospy.loginfo("Started nav_goals_generator service")

            # subscribing to a map
            self.map_frame = rospy.get_param('~map_frame', '/map')
            rospy.loginfo("Sampling goals in %s", self.map_frame)
            self.is_costmap = rospy.get_param('~is_costmap', 'false')

            #rospy.Subscriber(self.map_frame, OccupancyGrid, self.map_callback)

            # setting up the service
            self.ser = rospy.Service('/nav_goals', NavGoals, self.nav_goals_service)

            # visualizing nav goals in RVIZ
            self.pub = rospy.Publisher('nav_goals', PoseArray)

            # visualizing nav goals in RVIZ
            self.pubmarker = rospy.Publisher('visualization_marker_array', MarkerArray)
            self.marker_len = 0

            rospy.spin()
            rospy.loginfo("Stopped nav_goals_generator service")

        def map_callback(self,data):

            # get map data
            self.resolution = data.info.resolution
            self.width = data.info.width
            self.height = data.info.height
            self.origin = data.info.origin
            self.data = data.data

            self.map_min_x = self.origin.position.x
            self.map_max_x = self.origin.position.x + self.width * self.resolution
            self.map_min_y = self.origin.position.y
            self.map_max_y = self.origin.position.y + self.height * self.resolution


        def nav_goals_service(self,req):
            rospy.loginfo('Incoming service request: %s', req)

            # get arguments
            self.n = req.n
            self.inflation_radius = req.inflation_radius
            self.roi = req.roi

            res = NavGoalsResponse()

            try:
                msg = rospy.wait_for_message(self.map_frame, OccupancyGrid , timeout=10.0)
                self.map_callback(msg)
            except rospy.ROSException, e:
                rospy.logwarn("Failed to get %s" % self.map_frame)
                return res

            # process arguments
            self.process_arguments()

            # generate response
            res.goals.header.frame_id = '/map' # self.map_frame
            res.goals.poses = []

            self.delete_markers()
            markerArray = MarkerArray()

            # generate random goal poses
            # todo: learn upperbound
            upperbound = self.n  * ( 2 +  self.inflation_radius / 0.01)
            count = 0
            while len(res.goals.poses) < self.n and count < upperbound:
                count += 1
                cell_x = int(random.uniform(self.cell_min_x, self.cell_max_x))
                cell_y = int(random.uniform(self.cell_min_y, self.cell_max_y))

                pose = Pose()
                pose.position.x = cell_x * self.resolution + self.origin.position.x
                pose.position.y = cell_y * self.resolution + self.origin.position.y

                # if the point lies within ROI and is not in collision
                if self.in_roi(pose.position.x,pose.position.y) and not self.in_collision(cell_x,cell_y):

                    yaw = random.uniform(0, 2*math.pi)

                    q = list(tf.transformations.quaternion_about_axis(yaw, (0,0,1)))

                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]

                    #rospy.loginfo("pose x: %s, y: %s", pose.position.x, pose.position.y)
                    res.goals.poses.append(pose)

                    self.create_marker(markerArray,len(res.goals.poses)-1, pose)



            self.pub.publish(res.goals)

            self.marker_len =  len(markerArray.markers)
            #self.pubmarker.publish(markerArray)

            return res

        def create_marker(self,markerArray, marker_id, pose):
            marker1 = Marker()
            marker1.id = marker_id
            marker1.header.frame_id = "/map"
            marker1.type = marker1.TRIANGLE_LIST
            marker1.action = marker1.ADD
            marker1.scale.x = 1
            marker1.scale.y = 1
            marker1.scale.z = 1
            marker1.color.a = 0.1
            marker1.color.r = 1.0
            marker1.color.g = 0.0
            marker1.color.b = 0.0
            marker1.pose.orientation = pose.orientation
            marker1.pose.position = pose.position
            marker1.points = [Point(0,0,0),Point(3,-1.5,0),Point(3,1.5,0)]

            markerArray.markers.append(marker1)


        def delete_markers(self):
            markerArray = MarkerArray()
            for i in range(0,self.marker_len):
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.id = i
                marker.action = marker.DELETE
                markerArray.markers.append(marker)
            self.pubmarker.publish(markerArray)





        def process_arguments(self):
            # n must be positive
            if self.n < 0:
                self.n = 0

            # inflation radius must be positive
            if self.inflation_radius < 0:
                self.inflation_raidus = 0.5

            self.inflated_footprint_size = int(self.inflation_radius / self.resolution) + 1

            # region of interest (roi) must lie inside the map boundaries
            # if roi is empty, the whole map is treated as roi by default
            if len(self.roi.points) == 0:
                self.bbox_min_x = self.map_min_x
                self.bbox_max_x = self.map_max_x
                self.bbox_min_y = self.map_min_y
                self.bbox_max_y = self.map_max_y
                rospy.loginfo('no ROI specified, full map is used.')

            else:
                # if the roi is outside the map, adjust to map boundaries
                # determine bbox of roi
                self.bbox_min_x = float('inf')
                self.bbox_max_x = float('-inf')
                self.bbox_min_y = float('inf')
                self.bbox_max_y = float('-inf')
                for p in self.roi.points:
                    if p.x < self.map_min_x:
                        p.x = self.map_min_x
                    if p.x > self.map_max_x:
                        p.x = self.map_max_x

                    if p.x < self.bbox_min_x:
                        self.bbox_min_x = p.x
                    if p.x > self.bbox_max_x:
                        self.bbox_max_x = p.x

                    if p.y < self.map_min_y:
                        p.y = self.map_min_y
                    if p.y > self.map_max_y:
                        p.y = self.map_max_y

                    if p.y < self.bbox_min_y:
                        self.bbox_min_y = p.y
                    if p.y > self.bbox_max_y:
                        self.bbox_max_y = p.y

            # calculate bbox for cell array
            self.cell_min_x = int((self.bbox_min_x - self.origin.position.x) / self.resolution)
            self.cell_max_x = int((self.bbox_max_x - self.origin.position.x) / self.resolution)
            self.cell_min_y = int((self.bbox_min_y - self.origin.position.y) / self.resolution)
            self.cell_max_y = int((self.bbox_max_y - self.origin.position.y) / self.resolution)

            rospy.loginfo('ROI bounding box (meters): (%s,%s) (%s,%s)', \
                              self.bbox_min_x,self.bbox_min_y,self.bbox_max_x,self.bbox_max_y)

            rospy.loginfo('ROI bounding box (cells): (%s,%s) (%s,%s)', \
                          self.cell_min_x,self.cell_min_y,self.cell_max_x,self.cell_max_y)



        def cell(self, x,y):
                if x < 0 or y <0 or x >= self.width or y >= self.height:
                    #rospy.loginfo("out of bounds! x: %s, y: %s", x, y)
                    # return 'unknown' if out of bounds
                    return -1

                return self.data[x +  self.width * y]

        def in_roi(self,x,y):
            if (len(self.roi.points)==0):
                return True
            p = Point32(x,y,0)
            return is_inside(p, self.roi.points)

        def in_collision(self,x,y):

            if self.is_costmap:
                if (self.cell(x,y) != 0):
                    return True
                return False

            x_min = x - self.inflated_footprint_size
            x_max = x + self.inflated_footprint_size
            y_min = y - self.inflated_footprint_size
            y_max = y + self.inflated_footprint_size
            for i in range(x_min,x_max):
                for j in range(y_min,y_max):
                    if (self.cell(i,j) != 0):
                        return True
            return False


if __name__ == '__main__':
    NavGoalsGenerator()
