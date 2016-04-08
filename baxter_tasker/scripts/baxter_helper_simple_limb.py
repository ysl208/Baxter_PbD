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
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    )
from baxter_helper_abstract_limb import *
import limb
from hr_helper.post_threading import Post
from threading import Lock,Thread
from std_msgs.msg import Float64
import baxter_dataflow
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
"""

    Simple Limb that uses Baxter API

"""


class SimpleLimb(limb.Limb,AbstractLimb):
    def __init__(self,side,ik=True):
        limb.Limb.__init__(self,side)
        self.side=side
        
        self.DEFAULT_BAXTER_SPEED=0.3
        
        if not side in ["left","right"]:
            raise BaxterException,"Error non existing side: %s, please provide left or right"%side

        self.post=Post(self)
        self.stop = False
        self.simple=self
        self._moving=False
        self.moving_lock=Lock()
        
        self.ik=ik        
        if self.ik:            
            self.ns = "/ExternalTools/%s/PositionKinematicsNode/IKService"%self.side    
            rospy.loginfo("Waiting for inverse kinematics service on %s..."%self.ns)
            rospy.wait_for_service(self.ns)
            self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
            rospy.loginfo("Waiting for inverse kinematics service DONE")
        else:
            rospy.loginfo("Skipping inverse kinematics service loading")
        
        self._pub_speed=rospy.Publisher("/robot/limb/%s/set_speed_ratio"%self.side,Float64,queue_size=1)
        while not rospy.is_shutdown() and self._pub_speed.get_num_connections() < 1:
            rospy.sleep(0.1)
        #self.setSpeed(3)
        

    def setSpeed(self,speed):
        finalspeed=self.DEFAULT_BAXTER_SPEED*speed
        if finalspeed>1: finalspeed=1
        if finalspeed<0: finalspeed=0
        self._pub_speed.publish(Float64(finalspeed))
        
    def getPose(self):
        p=self.endpoint_pose()
        if len(p.keys())==0:
            rospy.logerr("ERROR: Pose is empty, you may want to wait a bit before calling getPose to populate data")
            return None
        pose=Pose()        
        pose.position=Point(*p["position"])
        pose.orientation=Quaternion(*p["orientation"])
        return pose
        
    def getAngles(self):
        d=self.joint_angles()
        return getAnglesFromDict(d)

        
    def goToPose(self,pose,speed=DEFAULT_SPEED,position_tolerance=DEFAULT_POSITION_TOLERANCE, orientation_tolerance=DEFAULT_ORIENTATION_TOLERANCE,joint_tolerance=DEFAULT_JOINT_TOLERANCE, speed_tolerance=DEFAULT_SPEED_TOLERANCE,timeout=DEFAULT_TIMEOUT,cartesian=False,path_tolerance=0):
        """ position_tolerance, orientation_tolerance, speed_tolerance, cartesian are ignored"""
        rospy.logdebug("SimpleLimb %s goToPose %s speed %f position_tolerance %f orientation_tolerance %f joint_tolerance %f speed_tolerance %f timeout %f cartesian=%s"%(self.side,getStrFromPose(pose),speed,position_tolerance,orientation_tolerance,joint_tolerance,speed_tolerance,timeout,str(cartesian)) )
        angles=self.getAnglesFromPose(pose)
        if angles:
            ret=self.goToAngles(angles,speed,joint_tolerance,joint_tolerance,speed_tolerance,timeout,path_tolerance)
        else:
            ret=False
        diff=getPoseDiff(pose,self.getPose())
        rospy.logdebug("SimpleLimb goToPose distance to target: "+str(diff))
        return ret
            
    def moving(self):
        with self.moving_lock:
            return self._moving
        
    def goToAngles(self,angles,speed=DEFAULT_SPEED,joint_tolerance_plan=DEFAULT_JOINT_TOLERANCE_PLAN,joint_tolerance=DEFAULT_JOINT_TOLERANCE,speed_tolerance=DEFAULT_SPEED_TOLERANCE,timeout=DEFAULT_TIMEOUT,path_tolerance=0):
        """ joint_tolerance_plan,speed_tolerance are ignored, """
        with self.moving_lock:
            self._moving=True
        d=getDictFromAngles(angles)        
        rospy.logdebug("SimpleLimb %s goToAngles %s speed %f joint_tolerance %f speed_tolerance %f timeout %f"%(self.side,getStrFromAngles(angles),speed,joint_tolerance,speed_tolerance,timeout))
        self.setSpeed(speed)
        try:
            ret=self.move_to_joint_positions(d,joint_tolerance,speed_tolerance,timeout)
        except Exception,e:
            rospy.logwarn( "Timeout PID: "+str(e))
            ret=False

        with self.moving_lock:
            self._moving=False
            
        diff=getAnglesDiff(angles,self.getAngles())
        rospy.logdebug("SimpleLimb %s goToAngles distance to target: %s"%(self.side,str(diff)))
        return ret


    def cancel(self):
        self._cancel=True

    def getAnglesFromPose(self,pose):
        if type(pose)==Pose:
            goal=PoseStamped()
            goal.header.frame_id="/base"
            goal.pose=pose
        else:
            goal=pose
        
        
        if not self.ik:
            rospy.logerror("ERROR: Inverse Kinematics service was not loaded")
            return None
        goalstr="%f,%f,%f"%(goal.pose.position.x,goal.pose.position.y,goal.pose.position.z)
        ikreq = SolvePositionIKRequest()
        
        ikreq.pose_stamp.append(goal)
        try:
            resp = self.iksvc(ikreq)
            if (resp.isValid[0]):
                return resp.joints[0]
            else:
                rospy.logerr("FAILURE - No Valid Joint Solution Found for %s"%goalstr)
                return None
        except rospy.ServiceException,e :
            rospy.loginfo("Service call failed: %s" % (e,))
            return None

    # Copied from original limb, added joint and speed tolerance parameters
    def move_to_joint_positions(self, positions, joint_tolerance=0.015,speed_tolerance=0.015,timeout=10.0):
        """
        Commands the limb to the provided positions.

        :param positions: joint_name:angle command
        :type positions: dict({str:float})
        :param joint_tolerance: Desired maximum goal tolerance in radians
        :type joint_tolerance: float
        :param timeout: seconds to wait for move to finish
        :type timeout: float

        Waits until the reported joint state matches that specified.
        """
        self._cancel=False
        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self._joint_angle[joint])
            return joint_diff

        diffs = [genf(j, a) for j, a in positions.items() if
                 j in self._joint_angle]

        baxter_dataflow.wait_for(
            lambda: (all(diff() < joint_tolerance
                         for diff in diffs) or self._cancel),
            timeout=timeout,
            timeout_msg=("%s limb failed to reach commanded joint positions" %
                         (self.name.capitalize(),)),
            rate=DEFAULT_RATE,
            body=lambda: self.set_joint_positions(positions)
            )
        return True

    def move_to_neutral(self):
        """
        Command the joints to the center of their joint ranges
        """
        angles = dict(zip(self.joint_names(),
                          [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
        return self.move_to_joint_positions(angles)
