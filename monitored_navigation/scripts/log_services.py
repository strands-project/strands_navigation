#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import roslib
import rospy

from ap_msgs.srv import *
from std_msgs.msg import String

import waypoint_patroller.log_util

class RosInterface(object):
    def __init__(self):
        self._gen = waypoint_patroller.log_util.StatGenerator()

        # set up services
        for s in dir(self):
            if s.endswith("_srv_cb"):
                rospy.Service("/ap_log_server/"+s[:-7],
                              getattr(self,s).type, getattr(self,s))
                

    def get_episode_names_srv_cb(self, req):
        r=GetEpisodeNamesResponse()
        r.episodes=self._gen.get_episode_names()
        return r
    get_episode_names_srv_cb.type=GetEpisodeNames

    def get_latest_episode_name_srv_cb(self, req):
        r=GetLatestEpisodeNameResponse()
        r.name = self._gen.get_latest_run_name()
        return r
    get_latest_episode_name_srv_cb.type=GetLatestEpisodeName

    def get_episode_summary_srv_cb(self, req):
        r=GetEpisodeSummaryResponse()
        r.summary = self._gen.get_episode(req.episode_name).get_json_summary()
        return r
    get_episode_summary_srv_cb.type=GetEpisodeSummary
    
    def get_episode_complete_srv_cb(self, req):
        r=GetEpisodeCompleteResponse()
        r.complete = self._gen.get_episode(req.episode_name).get_json_complete()
        return r
    get_episode_complete_srv_cb.type=GetEpisodeComplete
    
    
    
if __name__ == '__main__':
    rospy.init_node("ap_log_server")
    dm = RosInterface()
    rospy.spin()
