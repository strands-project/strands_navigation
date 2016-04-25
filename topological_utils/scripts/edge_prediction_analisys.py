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

#import matplotlib as mpl
#import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np


from geometry_msgs.msg import Point

#from mongodb_store.message_store import MessageStoreProxy
#from topological_navigation.tmap_utils import *
#from strands_navigation_msgs.msg import TopologicalMap
#from strands_navigation_msgs.msg import TopologicalNode
#from nav_msgs.msg import OccupancyGrid
from strands_navigation_msgs.srv import *




def usage():
    print "\nFor one image of the current map:"
    print "\t rosrun topological_utils draw_predicted_map.py"
    print "\nFor one image of one specific timestamp:"
    print "\t rosrun topological_utils draw_predicted_map.py -time epoch"
    print "For images in a range use (every two hours):"
    print "\t rosrun topological_utils draw_predicted_map.py -range from_epoch to_epoch"
    print "\n\n"
    #print "For all the stats from a date until now use:"
    #print "\t rosrun topological_navigation topological_prediction.py -range from_epoch -1"
    #print "For all the stats until one date:"
    #print "\t rosrun topological_navigation topological_prediction.py -range 0 to_epoch"
    #https://en.wikipedia.org/wiki/List_of_tz_database_time_zones


def predict_edges(epoch):
    rospy.wait_for_service('/topological_prediction/predict_edges')
    try:
        get_prediction = rospy.ServiceProxy('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState)
        print "Requesting prediction for %s"%epoch
        resp1 = get_prediction(epoch)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class predict_map(object):

    def __init__(self, epoch, props):
        if 'time_diff' in props:
            freq = props['time_diff']
        else:
            freq = 300
        #points = self.loadTMap()
        #points.nodes.sort(key=lambda node: node.name)        

        if len(epoch) <2:
            self.epoch = epoch[0]
            self.max_vel=0.01

            est = predict_edges(self.epoch)
            self.print_results(self.epoch, est, props)
            
        else :
            print epoch
            self.epoch = epoch[0]
            first_epoch = epoch[0]

            ep=[]
            trav=[]
            times=[]
            
            while self.epoch.secs <= epoch[1].secs:
                print self.epoch.secs
                self.max_vel=0.01
                est = predict_edges(self.epoch)
                self.print_results(self.epoch, est, props)
                if 'edge' in props:
                    for i in range(len(est.edge_ids)):                    
                        if est.edge_ids[i] == props['edge']:
                            ep.append(self.epoch.secs-first_epoch.secs)
                            trav.append(est.probs[i])
                            times.append(est.durations[i].secs)
                self.epoch = self.epoch + rospy.Duration.from_sec(freq)
                
                
            if 'edge' in props:
                self.create_graphs(ep, times, trav)
                    
    def create_graphs(self, ep, times, trav):
       x = np.asarray(ep)
       y1= np.asarray(trav)
       y2= np.asarray(times)
       print x
       print y1
       print y2
       
#       plt.plot(x, y1)
#       plt.savefig('trav.png') 
#       
#       plt.plot(x, y2)
#       plt.savefig('times.png')
   
       fig = plt.figure()

       ax1 = fig.add_subplot(211)
       ax1.plot(x, y1, 'r-')

       ax2 = fig.add_subplot(212)
       ax2.plot(x, y2, 'k-')

       plt.tight_layout()
       plt.show()       
       

    def print_results(self, epoch, est, props): 
        print "PREDICTION FOR TIME %d" %self.epoch.secs
        if len(est.edge_ids) == len(est.probs) and len(est.edge_ids) == len(est.durations):
            print "Good Answer!!! %d" %len(est.probs)
            for i in range(len(est.edge_ids)):
                if 'edge' in props:
                    if est.edge_ids[i] == props['edge']:
                        print epoch.secs,est.edge_ids[i], est.probs[i], est.durations[i].secs
                else:               
                    print est.edge_ids[i], est.probs[i], est.durations[i].secs
        else:
            print "something failed"
            print len(est.edge_ids), len(est.probs), len(est.durations)

        #print est.edge_ids
        
#    def load_t_map(self) :
#        try:
#            msg = rospy.wait_for_message('/topological_map', TopologicalMap, timeout=10.0)
#            return msg
#        except rospy.ROSException :
#            rospy.logwarn("Failed to get topological map")
#            return


if __name__ == '__main__':
    rospy.init_node('analise_prediction')
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
        elif '-time' in sys.argv:
            ind = sys.argv.index('-time')
            epochs.append(rospy.Time.from_sec(float(sys.argv[ind+1])))
            print epochs
        else:
            epochs.append(rospy.Time.now())
        if '-edge' in sys.argv:
            ind = sys.argv.index('-edge')
            props['edge'] = sys.argv[ind+1]
        if '-time_diff' in sys.argv:
            ind = sys.argv.index('-time_diff')
            props['time_diff'] = int(sys.argv[ind+1])

#        if '-rotate' in sys.argv:
#            ind = sys.argv.index('-rotate')
#            props['rotate'] = float(sys.argv[ind+1])
#        if '-scale' in sys.argv:
#            ind = sys.argv.index('-scale')
#            props['scale'] = float(sys.argv[ind+1])
#        if '-tz' in sys.argv:
#            ind = sys.argv.index('-tz')
#            props['tz'] = str(sys.argv[ind+1])
    
    server = predict_map(epochs, props)

