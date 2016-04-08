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
from high_level_movement import HighLevelMovement
from copy import deepcopy
from hr_helper.tf_helper import *
from hr_helper.post_threading import Post
import baxter_helper_abstract_limb
#############################################################
# Positions (for each side):
# H: high position on the back or side of baxter
# L: lower (grasp) position on the back or side of baxter
# P: same as H but in front of the robot
# F: same as L but in front of the robot
# O: first hand over position for two arm operations
# I: initial position of the robot arms
##############################################################

class PlanGenerator:
    """
        The PlanGenerator class is used to generate the plans between defined positions. 
        Currently there are 6 positions or angles for each side defined:
        
        * L - Low position: The gripping position of the object in a box
        * H - High position: The position above L. 
            Distance to L depends on the offset from the box to the object and the gripping depth
        * F - Front position: Similar to L but used for placing the item. It is the same for all items.
        * P - Prepare position: The position above F, depending on the offset between F and the dimensions of the box
        * I - Initial position: Initial position of an arm. Often used as starting point
        * O - Hand Over position: Specific position to pass items from one hand to the other.
    """
    def __init__(self,baxter):
        """
            :param baxter: object to access the scene and the high level motion
            :type baxter: BaxterRobot
        """
        self.baxter = baxter
        self.post = Post(self)
        self.source_boxes = ["item_tray","goblet_blue","goblet_red"]
        
        self.invalid_plans = 0
        self.p = {}
        self.tries = 3

    
    def generateAllPlans(self):
        """
            Generates all plans - non blocking - stoppable
        """
        self.post.__generateAllPlans()
    
    def __generateAllPlans(self):
        """
            Generates all plans - blocking
            
            Depending on the scenario, which has to be loaded in advance with the complete scene,
            scenario specific plans are generated.
            
            .. note:: If you add your own scenario you should add the plans you need in this file and create at 
            task in the GUI of your workstation
        """
        self.invalid_plans = 0
        self.baxter.enable()

##SELECT PLANS TO GENERATE
        if self.baxter.bb.bs.getScenario() == "consignment2":
            self.generateConsignment2Plans("right")
            self.generateConsignment2Plans("left")
        if self.baxter.bb.bs.getScenario() == "consignment":
            self.generatePlans("right","cover_small","top")
            self.generatePlans("right","switch","top")
            self.generatePlans("left","cover_door","top")
##############
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return
        
        rospy.loginfo("Generation complete. Items restored")
        self.tries-=1
        if self.invalid_plans!=0:
            rospy.logwarn("There are at least %d invalid plans!"%(self.invalid_plans))
            
            rospy.loginfo("Trying again. Number of tries: %d", 3-self.tries)
            if self.tries > 0:
                self.__generateAllPlans()
                return
            else:
                rospy.loginfo("Not all plans valid after %d tries",3-self.tries)
        else:
            self.baxter.hlm.goToInitPose()
            rospy.loginfo("All plans valid after %d iteration(s)!"%(3-self.tries))
        self.invalid_plans=0
        self.tries = 3
        
    def generateConsignment2Plans(self,side):
        """
            Generates plans for the consignment2 scenario
            
            :param side: creates plans for the given side
            :type side: str
        """
        for box in self.source_boxes:
            self.baxter.gripper[side].outside_grip = self.baxter.scene.boxes[box][2][0]
            self.baxter.gripper[side].release()
            if box == "goblet_blue":
                for id in xrange(0,self.baxter.scene.numItems(box)):
                    self.baxter.scene.removeItem(box,id,side)
            rospy.loginfo("generating plan for %s"%box)
            angles = []
            angles = self.getLH(side,box)
            self.baxter.scene.createItems2(box+'_'+side,box)
            self.getFP(side, box)
            self.getIP(side, box)
            self.getIH(side,box,angles)
            self.getHP(side, box, angles)

            

    def generatePlans(self,side,box,grip_point):
        """
            General function for plan generation, currently used by the consignment scenario.
            
            Creates the plans and most of the time also the reversed plans for:
            
            * LH
            * IH
            * HO
            * FP
            * OP
            * IP
        """
        angles = []
        self.p = {}
        angles = self.getLH(side,box)
        self.baxter.scene.createItems2(box+'_'+side,box)
        self.getIH(side,box,angles)
        if side=="right":
            self.getHO(side,box,angles,"top")
        if box == "switch":
            side = self.baxter.otherSide(side)
        self.getFP(side,box)
        if box != "switch":
            self.getHP(side, box, angles)
        self.getOP(side,box)
        self.getIP(side,box)
        
  
    def removeAllItems(self):
        """
            Removes all items inside the boxes
        """
        for cur_box in self.source_boxes:
            self.baxter.scene.removeItems(cur_box+"_left")
            self.baxter.scene.removeItems(cur_box+"_right")
    
    def restoreAllItems(self):
        """
            Restores the items of all boxes
        """
        for cur_box in self.source_boxes:
            self.baxter.scene.createItems2(cur_box+"_left",cur_box)
            self.baxter.scene.createItems2(cur_box+"_right",cur_box)        
        
    def moveAwayOtherArm(self,side):
        """
            Moves the other arm away to not interfere with the planning procedure
            
            :param side: The arm that is used. It is not the arm that is move away!
            :type side: str
        """
        move_params=[3,0.015,0.015,0.03,10]
        otherSide = self.baxter.otherSide(side)
        self.baxter.hlm.goToAngles(otherSide,otherSide+"_hand_init_away",move_params)
      
          
    def getLH(self,side,box):
        """
            Computes the plans L to H and H to L and returns the angles of the H "positions"
            
            :param side: arm side
            :type side: str
            :param box: Box where the items are located
            :type box: str
            :return: angles of the computed H positions
            :rtype: list(dict({str:float}))
        """
        rospy.loginfo("####Generating LH and HL plans for side %s for box %s",side,box)
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return []
        init = False

        angles = []
        num_items = self.baxter.scene.numItems(box) 
        for id in xrange(0,num_items):
            angles.append({})
            if box == "switch" and (id == 9 or id == 11):
                continue
            plan_name = box+"_"+side+"_"+str(id)+"_LH"
            reverse_plan_name = box+"_"+side+"_"+str(id)+"_HL"
            plan = baxter_helper_abstract_limb.loadPlan(plan_name)
            rplan = baxter_helper_abstract_limb.loadPlan(reverse_plan_name)
            self.baxter.scene.removeItem(box,id,side)
            if plan is None or rplan is None:
                if not init:
                    self.moveAwayOtherArm(side)
                    self.baxter.hlm.goToInitPose(side)
                    init = True
                pose = self.baxter.hlm.computeItemPose(box,id,side,False)
                move_params = [2,0.01,0.01,0.01,0.01,5,False]
                if not self.baxter.hlm.goToPose(side,deepcopy(pose),move_params):
                    rospy.logwarn("Cannot reach low position of item %d"%(id))
                    self.invalid_plans+=1
                    continue
                rospy.loginfo("Computing plan LH for item %d"%(id))
                #get LH plan
                pose = self.baxter.hlm.computeItemPose(box,id,side,True)
                plan  = self.baxter.arm[side].goToPose_plan(deepcopy(pose),position_tolerance=0.015,orientation_tolerance=0.01,cartesian = True)
                if plan is None:
                    rospy.logwarn("No plan LH found for item %d"%(id))
                    self.invalid_plans+=1
                    
                    continue    
                angles[id]= dict(zip(plan.joint_trajectory.joint_names,plan.joint_trajectory.points[-1].positions))
                baxter_helper_abstract_limb.savePlan(plan_name, plan)
                #get HL plan
                rplan = baxter_helper_abstract_limb.getReversePlan(plan)
                baxter_helper_abstract_limb.savePlan(reverse_plan_name, rplan)
            else:
                angles[id]= dict(zip(plan.joint_trajectory.joint_names,plan.joint_trajectory.points[-1].positions))
        print "angles:"
        for id in xrange(0,num_items):
            print id,":",angles[id]
        return angles
        rospy.loginfo("####LH and HL plans generated")
    
        
    def getFP(self,side,box):
        """
            Computes the plans F to P and P to F and saves the angles of the P "position" for the box
            
            :param side: arm side
            :type side: str
            :param box: Box where the items are placed
            :type box: str
        """
        rospy.loginfo("####Generating FP plans for side %s for box %s",side,box)
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return
        plan_name = box+"_"+side+"_FP"
        rplan_name = box+"_"+side+"_PF"
        plan = baxter_helper_abstract_limb.loadPlan(plan_name)
        rplan = baxter_helper_abstract_limb.loadPlan(rplan_name)
        
        pose =self.baxter.hlm.computeItemPose(box,-1,side,False)
        
        if plan is None or rplan is None:            
            self.moveAwayOtherArm(side)
            self.baxter.hlm.goToInitPose(side)
            move_params = [2,0.01,0.01,0.01,0.01,5,False]
#             self.baxter.frame.setTF("target",pose)
            if not self.baxter.hlm.goToPose(side,deepcopy(pose),move_params):
                rospy.logwarn("could not got to F position of item %s, side %s",box,side)
                self.invalid_plans+=1
                return
            pose = self.baxter.hlm.computeItemPose(box,-1,side,True)
#             self.baxter.frame.setTF("target",pose)
            plan  = self.baxter.arm[side].goToPose_plan(deepcopy(pose),position_tolerance=0.008,orientation_tolerance=0.008,cartesian = True)
            if plan is None:
                rospy.logwarn("FP plan not found!")
                self.invalid_plans+=1
                self.baxter.scene.release("left")
                return
            baxter_helper_abstract_limb.savePlan(plan_name, plan)
            #get LI plan
            rplan = baxter_helper_abstract_limb.getReversePlan(plan)
            baxter_helper_abstract_limb.savePlan(rplan_name, rplan)
        else:
            pose = self.baxter.hlm.computeItemPose(box,-1,side,True)
        rospy.loginfo("saving pose to p %s"%box)
        self.p[box] = dict(zip(plan.joint_trajectory.joint_names,plan.joint_trajectory.points[-1].positions))
        rospy.loginfo("####FP plans generated")

    def goToHighDestinationPose(self,side,box):
        """
            Moves the arm to the P angles computed by getFP
            
            :param side: arm side
            :type side: str
            :param box: Box where the items are placed
            :type box: str
            :return: True if the motion was successful
            :rtype: bool
        """
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return False
        try:
            move_params=[3,0.01,0.01,0.015,10]
            joint_angles = baxter_helper_abstract_limb.getAnglesFromDict(self.p[box])
            if not self.baxter.arm[side].goToAngles(joint_angles,*move_params):
                return False
            return True
        except:
            rospy.logwarn("could not move to prepare pose p")
            return False
        
    def getIP(self, side ,box):
        """
            Computes the plans I to P and P to I
            
            :param side: arm side
            :type side: str
            :param box: Box where the items are placed
            :type box: str
        """
        rospy.loginfo("####Generating IP plans for side %s for box %s",side,box)     
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return
        plan_name = box+"_"+side+"_IP"
        rplan_name = box+"_"+side+"_PI"
        plan = baxter_helper_abstract_limb.loadPlan(plan_name)
        rplan = baxter_helper_abstract_limb.loadPlan(rplan_name)
        if plan is None or rplan is None:
            self.moveAwayOtherArm(side)
            self.baxter.hlm.goToInitPose(side)
            rospy.loginfo("Computing plan IP")
            try:
                angles = self.p[box]
                joint_angles = baxter_helper_abstract_limb.getAnglesFromDict(angles)
            except:
                rospy.logwarn("p-Angles is empty")
                return
            plan  = self.baxter.arm[side].goToAngles_plan(joint_angles,0.02)
            if plan is None:
                rospy.logwarn("No plan IP found")
                self.invalid_plans+=1
                return
            baxter_helper_abstract_limb.savePlan(plan_name, plan)
            rplan = baxter_helper_abstract_limb.getReversePlan(plan)
            baxter_helper_abstract_limb.savePlan(rplan_name, rplan)
        rospy.loginfo("####IP plans generated")
        
    def getOP(self,side,box):
        """
            Computes the plans O to P and P to O
            
            :param side: arm side
            :type side: str
            :param box: Box where the items are placed
            :type box: str,
        """
        rospy.loginfo("####Generating OP and PO plans for side %s for box %s",side,box)      
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return
        plan_name = box+"_"+side+"_OP"
        rplan_name = box+"_"+side+"_PO"
        plan = baxter_helper_abstract_limb.loadPlan(plan_name)
        rplan = baxter_helper_abstract_limb.loadPlan(rplan_name)
        if plan is None or rplan is None:
            self.moveAwayOtherArm(side)
            move_params=[3,0.015,0.015,0.03,10]
            if not self.baxter.hlm.goToAngles(side,side+"_hand_over_0",move_params):
                rospy.logerr("Could not go to hand over angles 0")
            rospy.loginfo("Computing plan OP")
            try:
                angles = self.p[box]
                joint_angles = baxter_helper_abstract_limb.getAnglesFromDict(angles)
            except:
                rospy.logwarn("p-Angles is empty")
                return
            self.baxter.scene.pickUp(side,box,0,"side")
            plan  = self.baxter.arm[side].goToAngles_plan(joint_angles,0.02)
            if plan is None:
                rospy.logwarn("No plan IP found")
                self.invalid_plans+=1
                self.baxter.scene.release(side)
                return
            baxter_helper_abstract_limb.savePlan(plan_name, plan)
            rplan = baxter_helper_abstract_limb.getReversePlan(plan)
            baxter_helper_abstract_limb.savePlan(rplan_name, rplan)
        self.baxter.scene.release(side)
        rospy.loginfo("####OP and PO plans generated")

    def getHP(self,side,box,angles):
        """
            Computes the plans H to P and P to H
            
            :param side: arm side
            :type side: str
            :param box: Box where the items are placed
            :type box: str
            :param angles: angles of the H position of each item in the source box
            :type angles: list(dict({str,float}))
        """
        rospy.loginfo("####Generating PH plans %s"%(box))
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return
        init = False
        num_items = self.baxter.scene.numItems(box) 
        self.baxter.scene.pickUp(side, box,0,"top") 
        for id in xrange(0,num_items):
            if angles[id] == {}:
                continue
            plan_name = box+"_"+side+"_"+str(id)+"_PH"
            rplan_name = box+"_"+side+"_"+str(id)+"_HP"
            plan = baxter_helper_abstract_limb.loadPlan(plan_name)
            rplan = baxter_helper_abstract_limb.loadPlan(rplan_name)
            self.baxter.scene.removeItem(box,id,side)
            if plan is None or rplan is None:
                if init is False:
                    self.moveAwayOtherArm(side)
                    if not self.goToHighDestinationPose(side,box):
                        self.invalid_plans+=(num_items-1-id)
                        self.baxter.scene.release(side)
                        return
                    init = True
                rospy.loginfo("Computing plan PH for item %d"%(id))
                joint_angles = baxter_helper_abstract_limb.getAnglesFromDict(angles[id])
                plan  = self.baxter.arm[side].goToAngles_plan(joint_angles,0.02)
                if plan is None:
                    rospy.logwarn("No plan PH found for item %d"%(id))
                    self.invalid_plans+=1
                    continue
                baxter_helper_abstract_limb.savePlan(plan_name, plan)
                rplan = baxter_helper_abstract_limb.getReversePlan(plan)
                baxter_helper_abstract_limb.savePlan(rplan_name, rplan)
        self.baxter.scene.release(side)
        rospy.loginfo("####PH plans generated")

    def getIH(self,side,box,angles):
        """
            Computes the plans I to H
            
            :param side: arm side
            :type side: str
            :param box: Box where the items are placed
            :type box: str
            :param angles: angles of the H position of each item in the source box
            :type angles: list(dict({str,float}))
        """
        #get IH plan
        rospy.loginfo("####Generating IH plans %s"%(box))
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return

        num_items = self.baxter.scene.numItems(box) 

        init = False    
        for id in xrange(0,num_items):
            if angles[id] == {}:
                continue
            plan_name = box+"_"+side+"_"+str(id)+"_IH"
            #~ rplan_name = box+"_"+side+"_"+str(id)+"_HI"
            plan = baxter_helper_abstract_limb.loadPlan(plan_name)
            self.baxter.scene.removeItem(box,id,side)
            if plan is None: # or rplan is None:
                if init is False:
                    self.moveAwayOtherArm(side)
                    self.baxter.hlm.goToInitPose(side)
                    init = True
                rospy.loginfo("Computing plan IH for item %d"%(id))
                #get IH plan
                joint_angles = baxter_helper_abstract_limb.getAnglesFromDict(angles[id])
                plan  = self.baxter.arm[side].goToAngles_plan(joint_angles,0.02)
                if plan is None:
                    rospy.logwarn("No plan IH found for item %d"%(id))
                    self.invalid_plans+=1
                    continue
                baxter_helper_abstract_limb.savePlan(plan_name, plan)
                #~ rplan = baxter_helper_abstract_limb.getReversePlan(plan)
                #~ baxter_helper_abstract_limb.savePlan(rplan_name, rplan)
        rospy.loginfo("####IHs plans generated")


    def getHO(self,side,box, angles_H,grip_side="top"):
        """
            Computes the plans H to O and O to H
            
            :param side: arm side
            :type side: str
            :param box: Box where the items are located
            :type box: str
            :param angles: angles of the H position of each item in the source box
            :type angles: list(dict({str,float}))
            :param grip_side: depending on the grip side the 3D Model changes
            :type grip_side: str
        """
        #get HO plan
        rospy.loginfo("####Generating HO plans for %s, side %s"%(box,side))
        if self.baxter.hlm.stop():
            rospy.logwarn("Stop called")
            return
        num_items = self.baxter.scene.numItems(box) 
        move_params=[3,0.01,0.01,0.03,10]
        self.baxter.scene.pickUp(side,box,0,grip_side) 
        init = False
        for id in xrange(0,num_items):
            if angles_H[id] == {}:
                continue
            plan_name = box+"_"+side+"_"+str(id)+"_OH"
            plan = baxter_helper_abstract_limb.loadPlan(plan_name)
            rplan_name = box+"_"+side+"_"+str(id)+"_HO"
            rplan = baxter_helper_abstract_limb.loadPlan(rplan_name) 
            if plan is None:
               if init is False:
                   if not self.baxter.hlm.goToAngles(side,"right_hand_over_0",move_params):
                       rospy.logerr("Could not go to hand over angles") 
                       self.invalid_plans=+num_items
                       self.baxter.scene.release(side)
                       return
                   init = True
               rospy.loginfo("Computing plan HO for item %d"%(id))
               #get HO plan
               joint_angles = baxter_helper_abstract_limb.getAnglesFromDict(angles_H[id])
               plan  = self.baxter.arm[side].goToAngles_plan(joint_angles,0.02)
               if plan is None:
                   rospy.logwarn("No plan OH found for item %d"%(id))
                   self.invalid_plans+=1
                   continue    
               baxter_helper_abstract_limb.savePlan(plan_name, plan)
               rplan = baxter_helper_abstract_limb.getReversePlan(plan)
               baxter_helper_abstract_limb.savePlan(rplan_name, rplan)
        rospy.loginfo("####HO plans generated")
        self.baxter.scene.release(side)  
        
                  
                    
            
            