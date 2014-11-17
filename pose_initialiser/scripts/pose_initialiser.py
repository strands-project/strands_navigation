#!/usr/bin/env python
import rospy
import sys
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped

def initializer():
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
    rospy.init_node('mht_init')
    rospy.sleep(1.0)
    initialPose = PoseWithCovarianceStamped()
    initialPose.header.seq = 0
    initialPose.header.stamp.secs = 0
    initialPose.header.stamp.nsecs = 0
    initialPose.header.frame_id = '' 
    initialPose.pose.pose.position.x = float(sys.argv[1])
    initialPose.pose.pose.position.y = float(sys.argv[2])
    initialPose.pose.pose.position.z = 0.0
    initialPose.pose.pose.orientation.x = 0.0
    initialPose.pose.pose.orientation.y = 0.0
    initialPose.pose.pose.orientation.z = float(sys.argv[3])
    initialPose.pose.pose.orientation.w = float(sys.argv[4])
    p_cov = np.array([0.0]*36)
    initialPose.pose.covariance = tuple(p_cov.ravel().tolist())
    count = 0
    #while (count < 9):
    #    print 'The count is:', count
    #    count = count + 1
    print initialPose
    pub.publish(initialPose)
    print "Done"


if __name__ == '__main__':
    try:
        initializer()
    except rospy.ROSInterruptException:
        pass

