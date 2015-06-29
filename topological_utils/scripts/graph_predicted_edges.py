#! /usr/bin/env python

import rospy
import yaml
import sys

#from sensor_msgs.msg import Image
import cv2
import cv

from datetime import tzinfo, timedelta, datetime
import pytz
#from pytz import UTC

import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.pyplot as plt



import numpy as np


from geometry_msgs.msg import Point

from mongodb_store.message_store import MessageStoreProxy
from topological_navigation.tmap_utils import *
from strands_navigation_msgs.msg import TopologicalMap
from strands_navigation_msgs.msg import TopologicalNode
from nav_msgs.msg import OccupancyGrid
from strands_navigation_msgs.srv import *


def usage():
    print "\n ---------- Topological Predictions Visualisation Tool ---------------"
    print "\nFor one image of the current predicted map:"
    print "\t> rosrun topological_utils draw_predicted_map.py"
    print "\nFor one image of one specific timestamp:"
    print "\t> rosrun topological_utils draw_predicted_map.py -time epoch"
    print "For images in a range use (every two hours):"
    print "\t> rosrun topological_utils draw_predicted_map.py -range from_epoch to_epoch"
    print "\n\n Other options:"
    print "\t-rotate <deg> \t\t | rotate the map image by X degrees"
    print "\t-scale <scale factor> \t\t | scale the map image by X scaling factor"
    print "\t-tz <time zone> \t\t | create the image labels for a specific time zone" 
    print "\t\t\t\t | time zone codes wiki https://en.wikipedia.org/wiki/List_of_tz_database_time_zones"
        


def predict_edges(epoch):
    rospy.wait_for_service('/topological_prediction/predict_edges')
    try:
        get_prediction = rospy.ServiceProxy('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState)
        print "Requesting prediction for %s"%epoch
        resp1 = get_prediction(epoch)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class GraphStats(object):

    def __init__(self, epoch, props):
        self.norm = mpl.colors.Normalize(vmin=0, vmax=100)
        self.cmap = cm.jet
        self.colmap = cm.ScalarMappable(norm=self.norm, cmap=self.cmap)

        
        self.epoch = np.arange(epoch[0].secs, epoch[1].secs, 3600)

        #get list of edges
        est = predict_edges(epoch[0])
        edg_ids = est.edge_ids
        print edg_ids
        probs=[]
        durs=[]
        for i in edg_ids:
            probs.append([])
            durs.append([])
        
        for i in self.epoch:
            est = predict_edges(rospy.Time.from_sec(i))
            for j in est.edge_ids:
                lindex = est.edge_ids.index(j)
                probs[lindex].append(est.probs[lindex])
                durs[lindex].append(est.durations[lindex].secs)
            #print est edge_ids probs: durations:


        print edg_ids[80], probs[80], self.epoch
        for i in edg_ids:
            print i
            lindex = edg_ids.index(i)
            #print len(probs[0]), len(self.epoch)
            filename = 'AAF_'+str(i)+'.txt'
            fh = open(filename, "w")
            #count=0
            for j in range(0, len(self.epoch)):
                s_output = "%d\t%f\t%d\n" %(self.epoch[j],probs[lindex][j],durs[lindex][j])
                fh.write(s_output)
            fh.close

        for i in edg_ids:
            print i
            lindex = edg_ids.index(i)
            plt.plot(probs[lindex])
            plt.axis([0, 170, -0.1, 1.1])
            ax=plt.gca()
            ax.grid(True)
            cb=np.arange(0, 168,24)
            cmin=np.arange(12, 168,12)
            plt.xticks(cb)
            plt.xticks(cmin)
            plt.ylabel(edg_ids[lindex])
            figname=edg_ids[lindex]+'.png'
            plt.savefig(figname)
            plt.clf()

            #durs[lindex].append(est.durations)
            
#        while self.epoch.secs <= epoch[1].secs:
#            print self.epoch.secs
#            self.max_vel=0.01
#            est = predict_edges(self.epoch)
#            topmapimg = self.draw_top_map(points, map2d.info, map2dimg, est)
#            self.save_images(map2dimg, topmapimg)144
#            self.epoch = self.epoch + rospy.Duration(freq)
#        #print est.edge_ids
        

if __name__ == '__main__':
    rospy.init_node('graph_stats')
    epochs=[]
    props={}
    #if len(sys.argv) < 2:
    if '-h' in sys.argv or '--help' in sys.argv:
        usage()
        sys.exit(1)
    else:
        if '-range' in sys.argv:
            ind = sys.argv.index('-range')
            epochs.append(rospy.Time.from_sec(float(sys.argv[ind+1])))
            epochs.append(rospy.Time.from_sec(float(sys.argv[ind+2])))
            print epochs
        else:
            epochs.append(rospy.Time.now())
            epochs.append(rospy.Time.now()+rospy.Duration(3600*7*24))        
        
    server = GraphStats(epochs, props)
