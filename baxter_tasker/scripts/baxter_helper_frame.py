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
import roslib
import rospy
from hr_helper.post_threading import Post
from threading import Lock,Thread
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    
)
from std_msgs.msg import Header,Empty
import tf
import numpy
import math
from copy import deepcopy,copy
from hr_helper.tf_helper import *

"""
    Computes a new transformation frame based on 2 points and a reference frame.
    Also contains useful transformation functions.
"""

class BaxterFrame(TFHelper):
    def __init__(self,baxter):
        TFHelper.__init__(self)
        self.baxter = baxter
        self.post = Post(self)
        self.coords= [{},{}]
        self.box_index = 0
        self.num_points = 2
        self.start()
        
    def addPoint(self,side,step):
        """
            Saves the current position of the gripper_base in relation to the base
            :param side: selects the arm that gripper position should be saved
            :type side: str
            :param step: selects if the current position is the first or second point
            :type step: int 
            :return: returns the point that has been saved
            :rtype: list
        """
        pose = PS(side+"_gripper_base",[0,0,self.baxter.scene.gripperLength[side]])
        if not self.waitUntilFrameUpdate(side+"_gripper_base","base"):
            return None
        self.coords[step] = self.listener.transformPose("base",pose)
        rospy.loginfo("saved point %d : %s"%(step,str(self.coords[step])))
        return self.coords[step]
            
    def computeTransformation(self):
        """
            Computes the transformation from two saved points
            :return: computed pose and a unique frame id.
            :rtype (geometry_msgs.msg.PoseStamped,str)
        """
        num_coords = 0
        for coords in self.coords:
            if coords == {}:
                rospy.logwarn("computeTransformation: some points may not have been taken")    
            else:
                num_coords+=1
        if num_coords>= self.num_points : #compute pose offset
            rospy.loginfo("compute pose")
            try:
                p = []
                for coords in self.coords:
                    rospy.loginfo("%s"%coords)
                    try:
                        p.append(self.vector(self.baxter.pos2list(coords.pose.position)))
                    except Exception,e:
                        rospy.loginfo("compute transformation. no ROS pose found using baxter pose")
                        rospy.loginfo("%s"%str(e))
                        p.append(self.vector(list(coords['position'])))
            except:
                rospy.logwarn("insufficient points, try again")
                return
            pose = PoseStamped()
            while "box"+str(self.box_index) in self.transforms.keys() and not rospy.is_shutdown():
                self.box_index+=1
            pose.header = Header(stamp=rospy.Time.now(),frame_id="base")
            box_name = "box"+str(self.box_index)
            self.box_index+=1
            
            yaw = self.computeYaw(p)
            pitch = self.computePitch(p)
            roll = 0
            pose.pose.position = p[0]
            pose.pose.orientation = self.quaternion_from_euler(roll, pitch, yaw)
            rospy.loginfo( "Transformation computed")
            return (pose,box_name)    
        else: 
            rospy.loginfo("invalid number of points")
        

    
    def transformPose(self,target_frame,parent,src_pose):
        """
            Transforms a pose from one frame to another, which is NOT in the tf tree
            :param target_frame: The new and final frame id
            :type target_frame: str
            :param parent: The original source frame id
            :type parent: str
            :param src_pose: The current pose, which has to be transformed
            :type src_pose: geometry_msgs.msg.PoseStamped
            :return: The transformed pose that contains the position and orientation in a list
            :rtype: list
        """
        target_pose = self.getPose(target_frame,parent)
        target_pos = self.vector(target_pose[0:3])
        tar_rot = self.vector(self.euler_from_quaternion(target_pose[3:7]))
        tar_src_pos = list(target_pos - self.vector(src_pose[0:3]))
        src_rot = self.vector(self.euler_from_quaternion(src_pose[3:7]))
        tar_src_rot = (tar_rot - src_rot)%math.pi
        tar_src_quat = list(self.quaternion_from_euler(*tar_src_rot))
        return tar_src_pos + tar_src_quat

if __name__=="__main__":
    baxter=BaxterRobot(True)
    baxter.loadAll()
    side='left'
    baxter.bb.predefined_box='cover_door'
    pose = tf_helper.PS('base',[0.1666624398327384, 1.0099178330491019, -0.017062058603457095],[-0.0066044035893055306, -9.9674102351208056e-05, -0.9998643224110102, 0.015090019478927071])
    baxter.frame.setTF('cover_door_'+side,pose)
    baxter.frame.waitUntilFrameUpdate('cover_door_'+side)
    baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)
    rospy.spin()