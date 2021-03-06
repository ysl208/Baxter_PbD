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
import pickle
from hr_helper.post_threading import Post
from copy import deepcopy
from hr_helper.tf_helper import *
import baxter_helper_abstract_limb

class HighLevelMovement:
    """
        This class offers higher movement functions to directly go to an object in 
        a box or executing saved files that contains angles or even trajectories
    """
    def __init__(self,baxter):
        self.status = False
        self.lock = Lock()
        self.baxter = baxter
        self.save_plans = True
        self.post = Post(self)
        self.state_pos = {"left":"I","right":"I"}
        
    def stop(self,status=None):
        """
            Gets or sets the stop flag. Which inhibits or allows new movement requests.
            
            :param status: flag to set or None to get the status
            :type status: bool
        """
        with self.lock:
            if not status is None:
                self.status = status
            return self.status
        
    def goToAngles(self,side,angles_file,move_params):
            """ 
                Moves the arm to angles that have been saved in a file
                
                :param side: arm side to move
                :type side: str
                :param angles_file: path to the file that contains the angles
                :type angles_file: str
                :param move_params: [speed, joint_tolerance_plan, joint_tolerance, speed_tolerance, timeout]
                :type move_params: list
                :return: if the motion was successful
                :rtype: bool 
            """
            if self.stop():
                rospy.logwarn("HLM: Stop triggered")
                return False 
            angles = baxter_helper_abstract_limb.loadAngles(angles_file)
            return self.baxter.arm[side].goToAngles(angles,*move_params) 
        
        
    def executePlan(self,box,side,id,dst_plan,move_params):
        """
            Loads a given plan and executes it. The plan name is created by the given parameters. 
            Only works if the plan has been generated in advance
            
            .. note:: MoveIt will not replan around freshly inserted objects,
             if they were not there at the creation of the plan
            
            :param box: destination box
            :type box: str
            :param side: arm side to move
            :type side: str
            :param id: object id, only used if multiple objects are in the box
            :type id: int
            :param dst_plan: The letter for the destination position. Currently available strings are: H,L,O,I,P,F
            :type dst_plan: str
            
        """
        route = self.state_pos[side]+dst_plan 
        single_routes = ["IP","PI","PF","FP","OP","PO","IO","OI"]
        if route in single_routes:
            plan_name = box+"_"+side+"_"+route
        else:
            plan_name = box+"_"+side+"_"+str(id)+"_"+route
            
        plan = baxter_helper_abstract_limb.loadPlan(plan_name)
        if plan is None:
            rospy.logerr("%s No plan found: %s"%(side,plan_name))
            self.stop(True)
            return False
        if self.stop() is False:
            result = self.baxter.arm[side].execute(plan,speed=move_params[0],joint_tolerance=move_params[1],speed_tolerance=move_params[2],timeout=move_params[3])
            if result is False:
                self.stop(True)
                return False
        else:
            rospy.logwarn("HLM: Stop triggered")
            return False 
        
        rospy.loginfo("%s arm state changed to: %s",side,dst_plan)
        self.state_pos[side] = dst_plan
        return result
        
    def goToPose(self,side,pose,move_params):
        """ 
            Moves the arm the pose that have been saved in a file
            
            .. note:: Going to the same pose several times, may yield different angle combinations 
            
            :param side: arm side to move
            :type side: str
            :param pose: path to the file that contains the angles
            :type pose: str
            :param move_params: [speed , position_tolerance, orientation_tolerance, joint_tolerance, 
                speed_tolerance, timeout, cartesian]
            :type move_params: list
            :return: if the motion was successful
            :rtype: bool 
        """        
        if self.stop():
            rospy.logwarn("HLM: Stop triggered")
            return False
#         self.baxter.frame.setTF("target_"+side,pose)        
        plan=self.baxter.arm[side].goToPose_plan(deepcopy(pose),move_params[1],move_params[2],move_params[6])
        if plan is None:
            rospy.logwarn("No plan found")
            return False
        return self.baxter.arm[side].execute(plan,move_params[0],move_params[3],move_params[4],move_params[5])

    def computeItemPose(self,pre_def_box,id,side,high):
        """
            Computes an object position in a box, base on the definitions of the object 
            in baxter_scene (move to baxter_objects in next release)
            
            :param pre_def_box: The predefined box, in which the objects are located
            :type pre_def_box: str
            :param id: The ID of the object. First objects starts with 0
            :type id: int
            :param side: Used to take into account the gripper length
            :type side: str
            :param high: False, if the pose is exactly the pose, where the object should be gripped.
                True for the pose above the object in a perpendicular angle to the box
            :type high: bool
            :return: Computed pose for the object
            :rtype: geometry_msgs.msg.PoseStamped 
        """
        if id == -1:
            dsts = []
            dst_box = None
            for dst,value in self.baxter.scene.boxes.iteritems():
                if type(value[1]) == dict and dst.startswith("table") is False:
                    dsts.append(dst)
            for dst in dsts:
                if 'drop_off_'+pre_def_box in self.baxter.scene.boxes[dst][1]:
                    dst_box = dst
            rospy.loginfo("dst_box: %s"%dst_box)
            if dst_box is None:
                return None
            drop_off = self.baxter.scene.boxes[dst_box][1]['drop_off_'+pre_def_box]
            pose = PS(dst_box+"_"+side, drop_off[0:3],drop_off[3:7])
            if high:
                pose.pose.position.z = pose.pose.position.z + self.baxter.scene.gripperLength[side]-self.baxter.scene.gripperOffset - self.baxter.scene.boxes[pre_def_box][1][6] + self.baxter.scene.boxes[pre_def_box][1][5] -self.baxter.scene.boxes[dst_box][1]['drop_off_'+pre_def_box][2]
            else:
                pose.pose.position.z = pose.pose.position.z + self.baxter.scene.gripperLength[side]-self.baxter.scene.gripperOffset - self.baxter.scene.boxes[pre_def_box][1][6]

        else:      
            initial_pos = deepcopy(self.baxter.scene.boxes[pre_def_box][2][1])
            rospy.loginfo("box: %s, id: %d, side: %s"%(pre_def_box,id,side))
            offset = self.baxter.scene.boxes[pre_def_box][2][3]
            [x,y,z] = self.baxter.scene.ind2sub(pre_def_box,id)
            rospy.loginfo("initial pos: %s, offset %s, sub: %d %d %d"%(str(initial_pos), str(offset),x,y,z))
            #print "x",x,"y",y,"z",z
            initial_pos[0] = initial_pos[0] + offset[0]*x
            initial_pos[1] = initial_pos[1] + offset[1]*y
            if high:
                initial_pos[2] = self.baxter.scene.gripperLength[side]-self.baxter.scene.gripperOffset - self.baxter.scene.boxes[pre_def_box][1][6] + self.baxter.scene.boxes[pre_def_box][1][5] +0.01
            else:
                initial_pos[2] = initial_pos[2] - offset[2]*z + self.baxter.scene.gripperLength[side]-self.baxter.scene.gripperOffset - self.baxter.scene.boxes[pre_def_box][1][6]
            #print "initial pos 2",initial_pos[2], "gripper len",self.baxter.scene.gripperLength[side]
            pose = PS(pre_def_box+"_"+side,initial_pos,self.baxter.scene.boxes[pre_def_box][2][2])        
            rospy.loginfo("pose %s"%str(pose))
        return pose
    
    
    def __initArm(self,side):
        """
            Moves an arm to saved angles which are regarded the initial angles. 
            
            .. note:: Saved angle files are located in datapath+"/angles/". In case these angles
                should be changed, use the baxter monitor application.
            
            :param side: The arm to move to the initial angles
            :type side: str
            :return: If the motion was successful
            :rtype: bool
        """
        move_params=[4,0.02,0.02,0.03,10]
        result = self.goToAngles(side,side+"_hand_init",move_params)
        if result:
            self.state_pos[side] = "I"
        else:
            rospy.sleep(0.5)
            rospy.logwarn("Could not go to init angles. Trying again for side %s"%(side))
            result = self.goToAngles(side,side+"_hand_init",move_params)
        return result
    
    def goToInitPose(self,side=None):
        """
            Moves (an) arm(s) to the initial angles. 
            
            .. note:: If no arm is defined, both arms move to the intiial angles
            
            :param side: If given moves the target arm to the initial angles
            :type side: str or None
            :return: If the motions was successful
            :rtype: bool
        """
        self.baxter.enable()
        if side is None:
            th = self.post.__initArm("right")
            result_left = self.__initArm("left")
            th.join()
            result = result_left and th.result            
            return result
        else:
            return self.__initArm(side)
       
    def goToObject(self,box,id,side,dst_plan,pick=True):
        """
            Prepare arm and gripper for the requested object position and executes the motion.
            Previously generated plans are required.
            
            :param box:
            :type box: str
            :param id:
            :type id: int
            :param side: 
            :type side: str
            :param dst_plan:
            :type dst_plan: str
            :param pick: 
            :type pick: bool
            :return: If the motion was successful
            :rtype: bool
        """
        if dst_plan == "L" or dst_plan == "F":
            if pick is True:
                self.baxter.scene.removeItem(box,id,side)
        else:
             pass
         
        if (dst_plan == "H" and self.state_pos[side] == "L") or (dst_plan == "P" and self.state_pos[side] == "F"):    
            if pick is True:
                self.baxter.gripper[side].grip()
                self.baxter.scene.pickUp(side,box,id,"top")
            else:
                self.baxter.gripper[side].release()
                self.baxter.scene.release(side)
    
        timeout=5
        if dst_plan == "L" or dst_plan == "F":
            speed = 1.5
            joint_tolerance=0.009
            speed_tolerance=0.009
        else:
            speed = 4
            joint_tolerance=0.02
            speed_tolerance=0.025
           
        move_params = [speed,joint_tolerance,speed_tolerance,timeout]
        result = self.executePlan(box,side,id,dst_plan, move_params)
        return result


    
