#!/usr/bin/env python

import rospy

import topological_navigation.map_marker


class SimpleVisualiseMap(object):
    _killall_timers=False

    def __init__(self, name) :
        rospy.on_shutdown(self._on_node_shutdown)

        self.map_markers = topological_navigation.map_marker.TopologicalVis()
        rospy.loginfo("All Done ...")

    def _on_node_shutdown(self):
        print "bye"


if __name__ == '__main__':
    rospy.init_node('simlpe_topological_visualisation')
    server = SimpleVisualiseMap(rospy.get_name())
    rospy.spin()
