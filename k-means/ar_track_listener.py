#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from baxter_tasker.msg import BlockColour
from baxter_tasker.msg import BlockPosition
import std_msgs.msg
from std_msgs.msg import String
import numpy as np
from collections import Counter

class LandmarkMonitor(object):
    def __init__(self,pub):
        self.__pub = pub
        self.__landmarks = {}
        self.__blocks = {}
        self.__target_locations = {'A':(0.225, 0.01),'M':(0.07,-0.06),'D':(-0.084, -0.135)}

    def publisher(self,msg):
        self.__pub.publish(str(msg))
        rospy.loginfo(msg)


    def callback(self,data):
        red = BlockColour()
        red.name = 'red'

        colours = {'red','yellow','blue'}
        ids = range(9,18)
        m_colours = dict(zip(ids,colours))


        # landmark positions relative to the camera
        x = float("%.3f"%data.pose.position.x)
        y = float("%.3f"%data.pose.position.y)
        z = float("%.2f"%data.pose.position.z)

        self.__landmarks[data.id] = (x,y)
        rospy.loginfo('landmarks = {}'.format(self.__landmarks))

        print "-------------------------"
        # get all distances
        distances = [np.linalg.norm(i) for i in self.__landmarks.values()]
        #rospy.loginfo(distances)
        # get closest marker
        (closest_distance,closest_id) = min((np.linalg.norm(self.__landmarks[x]),x) for x in self.__landmarks)
        #closest_name = m_colours[closest_id]
        if closest_distance < 0.02:
            ld = 'closest marker = {}:{}'.format(closest_id,closest_distance)
            self.publisher(ld)

        
        # capture snapshot from 10 messages    
        if not(self.__blocks.has_key(data.id)):
            # if it is a new key then initialise to array
            self.__blocks[data.id] = []
        elif min(len(self.__blocks[i]) for i in self.__blocks) > 9:
            # if we recorded 10 instances or any observation, stop
            rospy.loginfo('10 observations found: {}'.format(self.__blocks))
            snapshot = [{x:Counter(self.__blocks[x]).most_common(1)[0][0]} for x in self.__blocks]
            
            rospy.loginfo('snapshot: {}'.format(snapshot))
            locations = [self.get_block_position(i,self.__target_locations) for i in snapshot]
            rospy.loginfo('Assigned locations: {}'.format(locations))
            self.publisher(locations)
            #rospy.signal_shutdown('World snapshot taken')
        else:
            # otherwise keep recording
            self.__blocks[data.id].append((x,y))
            

    def get_block_position(self,landmarks,positions):
        """
            Calculates the distances for each landmark to each position.
            Returns a dictionary with landmarks and closest positions
        """
        locations = {}
        #dist_matrix = [(b, p, np.linalg.norm( np.array(self.__landmarks[b]) - np.array(positions[p]) )) for b in self.__landmarks for p in positions] 
        
        for ld in landmarks:
            l = np.array(landmarks[ld])
            dist_matrix = [(np.linalg.norm( l - np.array(positions[p])),p) for p in positions]
            #rospy.loginfo('{} : {}'.format(ld,dist_matrix))
            (dist,pos) = min((np.linalg.norm( l - np.array(positions[p])),p) for p in positions)
            locations[ld] = pos

        #rospy.loginfo('Assigned locations: {}'.format(locations))
        return locations

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ar_listener', anonymous=True)

    pub = rospy.Publisher("/ar_snapshots", String, queue_size=1)

    monitor = LandmarkMonitor(pub)
    sub = rospy.Subscriber('/visualization_marker', Marker, monitor.callback)
    #    rospy.Subscriber('/robot/sonar/head_sonar/state', PointCloud, callback)
    # spin() simply keeps python from exiting until this node is stopped
    
    while not rospy.is_shutdown():
    #rospy.spinOnce()
        rospy.sleep(1)
    rospy.loginfo('Shutting down...')
    

if __name__ == '__main__':
    #rospy.loginfo("Enter the action name: ")
    #action = sys.stdin.readline().strip()

    #rospy.loginfo("Press Enter to start demonstration of '{}' action: ".format(action))
    listener()
    #stop = sys.stdin.readline().strip()