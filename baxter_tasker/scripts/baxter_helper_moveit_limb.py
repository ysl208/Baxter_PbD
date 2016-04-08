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
from hr_helper.post_threading import Post
from threading import Lock,Thread
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryGoal,
    JointTolerance,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from actionlib_msgs.msg import GoalID           
from moveit_msgs.srv import GetPositionIK,GetPositionFK
from baxter_helper_abstract_limb import *
from baxter_helper_simple_limb import SimpleLimb
from copy import deepcopy,copy




import sys
import rospy
import moveit_commander 
from hr_helper.singleton import Singleton
"""
    Limb that uses MoveIt API   

"""

    
class MoveItLimb(SimpleLimb,AbstractLimb):
    
    def __init__(self,side):
        if not side in ["left","right"]:
            raise BaxterException,"Error non existing side: %s, please provide left or right"%side
        SimpleLimb.__init__(self,side)
        self.side=side
        self.post=Post(self)
        self.stop = False
        self.DEFAULT_SPEED=0.3
        self.DEFAULT_PID_KP=2.0
        self.simple=SimpleLimb(side)
        self.group = self.getMoveGroup(side)
        self.group_name=side+"_arm"
        self.group.set_planning_time(10)
        ns="/robot/limb/%s/follow_joint_trajectory"%self.side
        self.pub_traj_goal=rospy.Publisher(ns+"/goal",FollowJointTrajectoryActionGoal,queue_size=1)           
        self.pub_traj_cancel=rospy.Publisher(ns+"/cancel",GoalID,queue_size=1)
        self.sub_traj_result=rospy.Subscriber(ns+"/result",FollowJointTrajectoryActionResult,self.callback_trajectory_result,queue_size=1)            
        self.fk_service=rospy.ServiceProxy("/compute_fk",GetPositionFK)
        self.ik_service=rospy.ServiceProxy("/compute_ik",GetPositionIK)
        self.is_moving=0
        self.current_goal_id=0
        self.moving_lock=Lock()
        self.result_error=False
        self.joints=self.group.get_joints()
        
    def getMoveGroup(self,side):
        """
            Gets the MoveIt interface to control an arm
            :param side: The arm side ("left" or "right")
            :return: Return the move group
        """
        try:
            return moveit_commander.MoveGroupCommander(side+"_arm")
        except Exception,e:
            rospy.logerr("MoveIt not found. Is the trajectory server and MoveIt running? %s"%str(e))
            exit() 

    def callback_trajectory_result(self,msg):
        """
            This callback function is invoked after a trajectory has been completed
            :param msg: The status and result published by the trajectory server 
        """
#         rospy.loginfo("Trajectory action server returned status %d"%msg.status.status)
        if msg.status.status!=3: # Error            
            self.result_error=True
            rospy.loginfo("Trajectory Result %d"%msg.result.error_code)
            rospy.logerr("ERROR: Trajectory action server returned status %d"%msg.status.status)
        else:
            self.result_error=False
            rospy.logdebug("OK Result received from trajectory action server on %s"%self.side)
            pass
        with self.moving_lock:
            self.is_moving=0
        
    def moving(self):
        """
            Checks if the robot is moving
            :return: Return True if something is moving on this side and False otherwise.
        """
        with self.moving_lock:
            return self.is_moving

    def getPose(self):
        return self.group.get_current_pose().pose

    def getAngles(self):
        js=JointState()
        values=self.group.get_current_joint_values()         
        for i in range(0,len(values)):
            js.name.append(self.joints[i])
            js.position.append(values[i])
        return js

    def cancel(self):
        with self.moving_lock:
            if self.is_moving==1:
                goal_id=GoalID()
                goal_id.id=self.side+"_"+str(self.current_goal_id)
                self.pub_traj_cancel.publish(goal_id)
            


    def goToPose(self,pose,speed=DEFAULT_SPEED,position_tolerance=DEFAULT_POSITION_TOLERANCE,orientation_tolerance=DEFAULT_ORIENTATION_TOLERANCE,joint_tolerance=DEFAULT_JOINT_TOLERANCE,speed_tolerance=DEFAULT_SPEED_TOLERANCE,timeout=DEFAULT_TIMEOUT,cartesian=False,path_tolerance=0):
        rospy.logdebug("MoveItLimb %s goToPose %s speed %f position_tolerance %f orientation_tolerance %f joint_tolerance %f speed_tolerance %f timeout %f cartesian=%s"%(self.side,getStrFromPose(pose),speed,position_tolerance,orientation_tolerance,joint_tolerance,speed_tolerance,timeout,str(cartesian)) )
        traj=self.goToPose_plan(pose,position_tolerance,orientation_tolerance,cartesian=cartesian)

        if traj is not None:
            ret=self.execute(traj,speed,joint_tolerance,speed_tolerance,timeout,path_tolerance)
            if type(pose)==PoseStamped:
                diff=getPoseDiff(pose.pose,self.getPose())
            else:
                diff=getPoseDiff(pose,self.getPose())            
            rospy.logdebug("MoveItLimb goToPose distance to target: "+str(diff))
            return ret                
        else:
            rospy.logerr("No trajectory found or trajectory empty")
            return False
        

    def goToAngles(self,angles,speed=DEFAULT_SPEED,joint_tolerance_plan=DEFAULT_JOINT_TOLERANCE_PLAN,joint_tolerance=DEFAULT_JOINT_TOLERANCE,speed_tolerance=DEFAULT_SPEED_TOLERANCE,timeout=DEFAULT_TIMEOUT,path_tolerance=0):
        rospy.logdebug("MoveItLimb %s goToAngles %s speed %f joint_tolerance_plan %f joint_tolerance %f speed_tolerance %f timeout %f"%(self.side,getStrFromAngles(angles),speed,joint_tolerance_plan,joint_tolerance,speed_tolerance,timeout) )

        traj=self.goToAngles_plan(angles,joint_tolerance_plan)
    
        if traj is not None:
            ret=self.execute(traj,speed,joint_tolerance,speed_tolerance,timeout,path_tolerance)
            diff=getAnglesDiff(angles,self.getAngles())
            rospy.logdebug("MoveItLimb goToAngles distance to target: "+str(diff))            
            return ret
        else:
            rospy.logerr("No trajectory found")
            return False

#             
    def goToPose_plan(self,pose,position_tolerance=DEFAULT_POSITION_TOLERANCE,orientation_tolerance=DEFAULT_ORIENTATION_TOLERANCE,cartesian=False):
        self.waitForStopped()

        self.group.clear_pose_targets()
        self.group.set_goal_position_tolerance(position_tolerance)
        self.group.set_goal_orientation_tolerance(orientation_tolerance)

        if cartesian:
            # MoveIt cartesian
            saved=None
            if type(pose)==Pose:
                waypoints=[pose]
            else:
                saved=self.group.get_pose_reference_frame()
                rospy.logdebug("Switching temporarily to reference frame %s"%pose.header.frame_id)
                self.group.set_pose_reference_frame(pose.header.frame_id)
                waypoints=[pose.pose]
                            
            (traj,fraction)=self.group.compute_cartesian_path(waypoints,0.01,10,True)
            
            if saved:
                self.group.set_pose_reference_frame(saved)
                rospy.logdebug("Switching back to reference frame %s"%saved)
                
            if fraction<1:
                rospy.logwarn("Fraction of incomplete cartesian path computed: %f, aborting"%fraction)
                return None
                #~ rospy.logwarn("Fraction of incomplete cartesian path computed: %f, falling back to non-cartesian"%fraction)
                #~ return self.goToPose_plan(pose,position_tolerance,orientation_tolerance,cartesian=False)
                
        else:                
            # Basic MoveIt
            self.group.set_pose_target(pose)

            traj=self.group.plan()
        
        if traj is None:
            rospy.logwarn("No trajectory found %s"%self.side)
            return None

        if len(traj.joint_trajectory.points)==0:
            rospy.logwarn("Empty trajectory %s"%self.side)
            return None

        #~ print "Original plan"
        #~ print traj

        self.cleanupTrajectory(traj)
        return traj
            
        



    def goToAngles_plan(self,angles,joint_tolerance_plan=DEFAULT_JOINT_TOLERANCE_PLAN):
        self.waitForStopped()        
        
        self.group.clear_pose_targets()
        self.group.set_goal_joint_tolerance(joint_tolerance_plan)

        d=getDictFromAngles(angles)    
        self.group.set_joint_value_target(dict(d)) # Apparently OrderedDict is not properly supported anymore
        traj=self.group.plan()
        
        if traj is None:
            rospy.logwarn("No trajectory found %s"%self.side)
            return None

        if len(traj.joint_trajectory.points)==0:
            rospy.logwarn("Empty trajectory %s"%self.side)
            return None

        #~ print "Original plan"
        #~ print traj

        self.cleanupTrajectory(traj)
        return traj

    def execute(self,traj,speed=DEFAULT_SPEED,joint_tolerance=DEFAULT_JOINT_TOLERANCE,speed_tolerance=DEFAULT_SPEED_TOLERANCE,timeout=DEFAULT_TIMEOUT,path_tolerance=0):
        traj = deepcopy(traj)
        self.applyTrajectorySpeed(traj,speed)

        rospy.logdebug("Performing trajectory on %s at speed %f tolerance %f speed_tolerance %f timeout %f"%(self.side,speed,joint_tolerance,speed_tolerance,timeout))
        joint_names=traj.joint_trajectory.joint_names

        self.waitForStopped()

        with self.moving_lock:
            if self.is_moving!=0:
                rospy.logerr("ERROR: trajectory already running")
                return False
            self.is_moving=1

        goal=FollowJointTrajectoryActionGoal()
        self.current_goal_id+=1
        goal.goal_id.id=self.side+"_"+str(self.current_goal_id)
        goal.goal.trajectory=traj.joint_trajectory
        if path_tolerance<=0: path_tolerance=DEFAULT_PATH_TOLERANCE*speed
        
        for name in joint_names:
            t=JointTolerance()
            t.name=name
            t.position=joint_tolerance
            t.velocity=speed_tolerance
            t.acceleration=0
            
            goal.goal.goal_tolerance.append(t)
        
            pt=JointTolerance()
            pt.name=name
            if name.find("w2") is False:
                pt.position=path_tolerance
            else:
                pt.position=0.0
            goal.goal.path_tolerance.append(pt)

        goal.goal.goal_time_tolerance=rospy.Duration(timeout)
                    
        # Send goal and wait for trajectory finished
        self.pub_traj_goal.publish(goal)                
        self.waitForStopped()
        
        return not self.result_error

    def applyTrajectorySpeed(self,traj,spd):
        spd=float(spd)
        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)
        if n_points==0:
            return
        for i in range(n_points):
            t=traj.joint_trajectory.points[i].time_from_start
            if type(t)!=float:
                t=t.to_sec()
            traj.joint_trajectory.points[i].time_from_start = rospy.Duration(t/spd)

    def cleanupTrajectory(self,traj):
        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)
        for i in range(n_points):
            traj.joint_trajectory.points[i].velocities=[]
            traj.joint_trajectory.points[i].accelerations=[]            




    def getAnglesFromPose(self,pose):
        """ Calls inverse kinematics to obtain joint angles from a cartesian pose, return None if failed """
        req=GetPositionIK()
        if type(pose)==Pose:
            ps=PoseStamped()
            ps.pose=pose
            ps.header.frame_id="/base"
        else:
            ps=pose
            
        req.group_name=self.group_name
        req.pose_stamped=ps
        req.robot_state=RobotState()
        req.constraints=Constraints()
        req.avoid_collisions=True
        req.ik_link_name=self.group.get_end_effector_link()
        req.ik_link_names=[]
        req.pose_stamped_vector=[]
        req.timeout=rospy.Duration(1)
        req.attempts=3
        
        resp=self.ik_service(req)
        result=resp.solution.joint_state
        ln=result.name
        lp=result.position
        zn=list()
        zp=list()
        for i in range(0,len(ln)):
            if ln[i] in self.joints:
                zn.append(ln[i])
                zp.append(lp[i])
        result.name=zn
        result.position=zp                
        return result

    
    def getPoseFromAngles(self,angles):
        rs=RobotState()
        rs.joint_state=angles
        header=Header()
        header.frame_id="/base"
        resp=self.fk_service(header,["%s_gripper"%self.side],rs)
        return resp.pose_stamped[0].pose
                



class MoveItHelper:
    __metaclass__=Singleton
    
    def __init__(self):
        rospy.loginfo("Starting MoveIt Commander")
        moveit_commander.roscpp_initialize(self._removeLaunchArgs(sys.argv))
        moveit_commander.MoveGroupCommander("left_arm") # just to check for correct initialization
    
    def __del__(self):
        rospy.loginfo("Shutting down MoveIt Commander")
        moveit_commander.roscpp_shutdown()
        
    def _removeLaunchArgs(self,args):    
        return [s for s in args if not s.startswith("__name:=")]
    
    