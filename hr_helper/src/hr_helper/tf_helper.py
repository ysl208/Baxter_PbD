#!/usr/bin/env python


########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 

# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.
#
#############################################################################
import rospy
from copy import deepcopy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from post_threading import Post
from threading import Lock
import math
import tf
from std_msgs.msg import Header
from singleton import Singleton
import numpy

FRAME_ORIGIN="base"

def sanitizePS(ps):
    """
        Converts Pose to PoseStamped()
        Converts lists of positions and orientations to the correct datatype
        
        :param ps: pose to repair
        :type ps: geometry.msgs.msg.Pose or geometry.msgs.msg.PoseStamped
        :return: repaired pose
        :rtype: geometry.msgs.msg.PoseStamped
    """
    _ps=deepcopy(ps)    
    if type(_ps)==Pose:
        ps=PoseStamped()
        ps.header.stamp=rospy.Time().now()
        ps.header.frame_id=FRAME_ORIGIN
        ps.pose=_ps
    elif type(_ps)==PoseStamped:
        ps=_ps
    else:
        raise Exception,"Unhandled data type for sanitizePS: %s"%str(type(_ps))
    if type(ps.pose.position)!=Point: # Assume this is a list or numpy array
        ps.pose.position=Point(*ps.pose.position)
    if type(ps.pose.orientation)!=Quaternion: # Assume this is a list or numpy array
        ps.pose.orientation=Quaternion(*ps.pose.orientation)
    return ps
    

def PS(origin,position=[0,0,0],orientation=[0,0,0,1]):
    """
        Creates a PoseStamped()
        
        :param origin: frame id
        :type: origin: str
        :param position: position (default [0,0,0])
        :type: position: list
        :param oriention: orientation (default [0,0,0,1])
        :type: orientation: list
        :return: the created PoseStamped()
        :rtype: PoseStamped
    """
    h=Header()
    h.frame_id=origin
    h.stamp=rospy.Time().now()
    
    p=Pose()
    p.position=Point(*position)
    p.orientation=Quaternion(*orientation)
        
    return PoseStamped(h,p)
 
class TFHelper():
    __metaclass__=Singleton
    """
        Helps broadcasting static tf frames
    """
    def __init__(self,frequency=50):
        self.frequency=frequency
        
        
        self.post = Post(self)
        self.mutex = Lock()
        self.tb = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.transforms={}
        self.thread=None
        self._stop_broadcast=False
        self.last_keys=None
        
        self.medians = {}

    def __del__(self):
        self.stop()
        
        
    def start(self):
        """ Start broadcasting thread,  blocking """
        with self.mutex:            
            if self.thread:
                self.stop()
            self._stop_broadcast=False
            self.thread=self.post.__broadcastLoop()        

    def stop(self):
        """ Stop broadcasting thread,  blocking """
        with self.mutex:
            if self.thread:
                self._stop_broadcast=True
                self.thread.join()
                self.thread=None

    def __broadcastLoop(self): 
        """
            Loop that broadcasts all transformations. Started by (BaxterFrame)
        """       
        with self.mutex:
            self.last_keys=None
        rate=rospy.Rate(self.frequency)
        while not rospy.is_shutdown() and not self._stop_broadcast:
            with self.mutex:
                if not self.last_keys or str(self.last_keys)!=str(self.transforms.keys()):
                    rospy.loginfo("Broadcasting %d frames: %s"%(len(self.transforms.items())  , self.transforms.keys() ) )
                    self.last_keys=str(self.transforms.keys())
                for id,t in self.transforms.items():
                    if not rospy.is_shutdown():
                        try:
                            self.tb.sendTransform(self.pos2list(t.pose.position),self.quat2list(t.pose.orientation),rospy.Time.now(), "/"+id, "/"+t.header.frame_id)
                        except rospy.ROSException,e:
                            rospy.logwarn("Could not publish on TF topic, closing broadcaster: "+str(e))
                            break
            rate.sleep()

    def getPose(self,target_frame,source_frame=FRAME_ORIGIN):
        """
            Get a pose between two frames of the tf tree
            
            :param target_frame: The frame you want to transfrom to
            :type target_frame: str 
            :param source_frame: The origin of the pose (default: "base")
            :type source_frame: str
            :return: returns a concatenated list of the translation and rotation or None, if the tf is not available
            :rtype: list
        """
        try:
            latest = rospy.Time(0)
            if not self.waitUntilFrameUpdate(target_frame,source_frame):
                return None
            (trans,rot) = self.listener.lookupTransform(target_frame,source_frame,latest)
            return list(trans)+list(rot)
        except Exception,e:
            rospy.logerr("%s"%str(e))
            return None

    def setTF(self,name,tf):
        """
            Adds a new transformation to the broadcasting loop or updates an exisiting one
        """
        #~ rospy.loginfo("Requested to broadcast new tf with name %s"%(name))
        with self.mutex:
            ps=sanitizePS(tf)
            self.transforms[str(name)]=ps

    def unsetTF(self,name):
        """
            Removes a transformation from the loop
        """
        with self.mutex:
            try:
                del self.transforms[name]
            except Exception,e:
                rospy.logwarn("unsetTF does not exist: %s"%name)

    def getTF(self,name):
        """
            Returns a transformation for the given name or None, if the entry does not exist.
        """
        with self.mutex:
            try:
                return deepcopy(self.transforms[name])
            except:
                return (None,None)

    def waitUntilFrameUpdate(self,frame_id_1,frame_id_2=FRAME_ORIGIN,timeout=5):       
        """
            Waits until a frame transformation is broadcasted
            
            :param frame_id_1: target frame
            :type frame_id_1: str
            :param frame_id_2: source frame (default: "base")
            :type frame_id_2: str
            :param timeout: time to wait, until an exception is thrown (default: 5secs)
            :type timeout: float
            :return: True is tf is available
            :rtype: bool
            :raise exception: If transformation is not available after the timeout  
        """
        now = rospy.Time.now()
        end = rospy.Time.now()+rospy.Duration(timeout)
        while not rospy.is_shutdown() and now < end:
            now = rospy.Time.now()
            try:
                self.listener.waitForTransform(frame_id_2, frame_id_1, now, rospy.Duration(0.5))
                return True
            except Exception, e:#(tf.Exception, tf.LookupException, tf.ConnectivityException):
                pass
        raise Exception,"Transform %s -> %s never appeared"%(frame_id_1,frame_id_2)
        return False    

    def quaternion_from_euler(self, roll,pitch,yaw,axes='sxyz'):
        """
            Returns a geometry_msgs.msg.Quaterion, computed of 3 angles
        """
        return tf.transformations.quaternion_from_euler(roll,pitch,yaw,axes)
        
    def euler_from_quaternion(self,quat,axes='sxyz'):
        """
            Returns [roll, yaw, pitch], computed on the given quaternion
        """
        return tf.transformations.euler_from_quaternion(quat,axes)
        
    def invertPose(self,new_source_frame,pose):
        """
            Inverts the given pose stamped and sets the parent frame id to the given frame
            
            :param new_source_frame: The new frame_id
            :type: new_source_frame: str
            :param pose: the pose which should be inverted
            :type pose: geometry_msgs/PoseStamped
            :return: The inverted pose
            :rtype: geometry_msgs/PoseStamped
        """
        euler_angles = tf.transformations.euler_from_quaternion(self.quat2list(pose.pose.orientation))
        tfmatrix = tf.transformations.compose_matrix(scale=None,shear=None,angles=euler_angles,translate=self.pos2list(pose.pose.position),perspective=None)
        tfmatrix_inv = tf.transformations.inverse_matrix(tfmatrix)
        return PS(new_source_frame,tf.transformations.translation_from_matrix(tfmatrix_inv),tf.transformations.quaternion_from_matrix(tfmatrix_inv))

    def computeYaw(self,points):
        """
            Computes the yaw angle between two points in the base system
        """
        p = deepcopy(points)
        p[1][2] = p[0][2]
        v_12 = p[1]-p[0]
        p_yaw = deepcopy(p[0])
        p_yaw[0]+=10
        v_yaw = p_yaw - p[0]
        angle = self.computeAngle(v_12, v_yaw)
        if v_12[1] < 0:
            angle = -angle
        rospy.loginfo("yaw in deg %f"%self.rad2deg(angle))
        return angle
    
    def computePitch(self,points):
        """
            Computes the pitch angle between two points in the base system
        """
        p = deepcopy(points)
        v_12 = p[1]-p[0]
        p1z0 = p[1]
        p1z0[2] = p[0][2]
        v_pitch = p1z0 - p[0]
        angle = self.computeAngle(v_12, v_pitch)
        if v_12[2] < 0:
            angle = -angle
        rospy.loginfo("pitch in deg %f"%self.rad2deg(angle))
        return -angle
        
    def computeLength(self,v):
        """
            Computes the length of a vector
        """
        return numpy.sqrt(v.dot(v))
    
    def rad2deg(self,rad):
        """
            Returns degrees that correspond to the given radians
        """
        return rad*180/math.pi
    
    def deg2rad(self,deg):
        """
            Returns radians that correspond to the given degrees
        """
        return deg/180*math.pi
    
    def computeAngle(self,v1,v2):
        """
            Return the angles between two vectors
        """
        top = (v1.dot(v2))
        bottom = self.computeLength(v1)*self.computeLength(v2) 
        if math.fabs(bottom)<0.01:
            rospy.loginfo("Error: Point are to close to each other, increase the distance between the points")
            return 0
        return numpy.arccos(top/bottom)
    
    def vector(self,coords):
        """
            Returns the numpy.array for a given list
        """
        return numpy.array(coords) 
    
    def __initNewFilter(self,filter_size):
        storage = numpy.zeros((filter_size,7)) - 999
        storage_element = 0
        return [storage_element, storage]
    
    def __updateFilter(self, pose, filter_size):
        if not pose.header.frame_id in self.medians.keys():
            self.medians[pose.header.frame_id] = self.__initNewFilter(filter_size)
        
        i = (self.medians[pose.header.frame_id][0] +1) % filter_size
        self.medians[pose.header.frame_id][0] = i
        self.medians[pose.header.frame_id][1][i] = numpy.array(self.pos2list(pose.pose.position) + self.quat2list(pose.pose.orientation))
    
    def getMedianPose(self,pose, filter_size=10):
        if filter_size <=1:
            return pose
        self.__updateFilter(pose, filter_size)
        
        for i in xrange(filter_size):
            if self.medians[pose.header.frame_id][1][i,0]== -999:
                #rospy.loginfo("not enough values to filter yet")
                return pose 
        print "filtering.."
        filtered_pose =  numpy.zeros((7))
        
        for i in xrange(7):
            values = sorted(self.medians[pose.header.frame_id][1][:,i])
            if i <=1 :
                print values
            filtered_pose[i] = values[int(filter_size/2)]
        filtered_pose = PS(pose.header.frame_id,filtered_pose[0:3], filtered_pose[3:7])        
        return filtered_pose
    
    def quat2list(self,quat):
        """ 
            Converts a geometry.msgs.Quaternion to a list
            
            :param quat: Quaterion to convert
            :type quat: geometry.msgs.msgs.Quaternion
            :return: list containing a Quaternion
            :rtype: list
            
        """
        return [quat.x,quat.y,quat.z,quat.w]
           
    def pos2list(self,pos):
        """
            Converts a geometry.msgs.Point to a list
            
            :param pos: Point to convert
            :type pos: geometry.msgs.Point
            :return: list containing a Point
            :rtype: list  
            
        """
        return [pos.x,pos.y,pos.z]
        

if __name__ == '__main__':  
    rospy.init_node('th_helper',anonymous=True)

    origin="/base"
    tfh=TFHelper()
    tfh.start()
     
#     print tfh.getPose("wako","base")
#     from baxter_helper import *
#     baxter=BaxterRobot(True)
#     baxter.loadFrame()
#     baxter.loadBaxterBehaviors()
#     side='left'
#     baxter.bb.predefined_box='cover_door'
#     pose = PS('base',[0.1666624398327384, 1.0099178330491019, -0.017062058603457095],[-0.0066044035893055306, -9.9674102351208056e-05, -0.9998643224110102, 0.015090019478927071])
#     print "set tf"
#     baxter.frame.setTF('cover_door_'+side,pose)
#     print "wait until update"
#     baxter.frame.waitUntilFrameUpdate('cover_door_'+side)
#     print "create relative boxes"
#     baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)
#     print "done"
#     rospy.sleep(1)
#     rospy.spin()
    rospy.loginfo("open your rviz and add the tf to see the broadcasted frames")
    tfh.setTF("test",PS(origin,[2,0,0]) )    
    tfh.setTF("test2",PS(origin,[1,0,0]) )
    rospy.sleep(5)

    tfh.unsetTF("test2")
    rospy.sleep(5)
    tfh.setTF("test2",PS("test",[1,0,0]) )
    rospy.sleep(2)

    tfh.setTF("test2",PS("test",[1,1,0]) )
    rospy.sleep(2)
    
    
    