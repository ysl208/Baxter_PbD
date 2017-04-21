#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
import std_msgs.msg
from std_msgs.msg import String
import numpy as np

class LandmarkMonitor(object):
    def __init__(self,pub,landmarks):
        self.__pub = pub
        self.__landmarks = landmarks
        

    def callback(self,data):
        target_locations = {'A':(0.6,-0.419),'M':(0.795,-0.2),'D':(0.6, -0.018)}
        # landmark positions relative to the camera
        x = float("%.2f"%data.pose.position.x)
        y = float("%.2f"%data.pose.position.y)
        z = float("%.2f"%data.pose.position.z)

        self.__landmarks[data.id] = (x,y)
        rospy.loginfo('landmarks = {}'.format(self.__landmarks))

        print "-------------------------"
        # get all distances
        distances = [np.linalg.norm(i) for i in self.__landmarks.values()]
        rospy.loginfo(distances)
        # get closest marker
        (closest_distance,closest_id) = min((np.linalg.norm(self.__landmarks[x]),x) for x in self.__landmarks)
        #closest_name = m_colours[closest_id]
        ld = 'closest marker = {}:{}'.format(closest_id,closest_distance)
        locations = self.get_position(self.__landmarks,target_locations)
        #rospy.loginfo('assigned locations: {}'.format(locations))

        self.__pub.publish(str(self.__landmarks))
        if closest_distance < 0.02:
            rospy.loginfo('i am near {}'.format(ld))

    def get_position(self,landmarks,positions):
        """
            Calculates the distances for each landmark to each position.
            Returns a dictionary with landmarks and closest positions
        """
        locations = {}
        #dist_matrix = [(b, p, np.linalg.norm( np.array(self.__landmarks[b]) - np.array(positions[p]) )) for b in self.__landmarks for p in positions] 
        
        for ld in landmarks:
            l = np.array(landmarks[ld])
            dist_matrix = [(np.linalg.norm( l - np.array(positions[p])),p) for p in positions]
            rospy.loginfo('{} : {}'.format(ld,dist_matrix))
            (dist,pos) = min((np.linalg.norm( l - np.array(positions[p])),p) for p in positions)
            locations[ld] = pos
        rospy.loginfo('assigned locations: {}'.format(locations))

        return locations

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ar_listener', anonymous=True)

    landmarks = {}
    colours = {'red','yellow','blue','','','','','','','','','',''}
    ids = range(9,18)
    m_colours = dict(zip(ids,colours))

    pub = rospy.Publisher("/ar_closest_marker", String, queue_size=1)
    monitor = LandmarkMonitor(pub,landmarks)
    rospy.Subscriber('/visualization_marker', Marker, monitor.callback)
#    rospy.Subscriber('/robot/sonar/head_sonar/state', PointCloud, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
