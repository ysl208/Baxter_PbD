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
import sys
import rospy

from copy import deepcopy
#from moveit_msgs.msg import RobotState, DisplayTrajectory, OrientationConstraint
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
try:
    from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
except:
    print "moveit not installed"
from operator import mul
from hr_helper.post_threading import Post
"""
    The baxter scene extends the moveit scene python wrapper to create more complex objects that are useful for baxter, like:
    
    * Cylinders
    * Empty Boxes
    * Grippers
    
    Additionally all pre defined item types are implemented manually here. 
    These are move to a file called baxter_objects.py on the next release
"""
class BaxterScene():
    def __init__(self):
        """
            Creates a link to the moveit scene, saves the current gripper lengths and knows the dimensions of all 
            important objects in the scene.
            
            As there is not object scanner yet, objects have to manually added to the scene.
            This corresponds to a dictionary entry in the member "boxes". For baxter's scenario is prooved to be useful
            to have boxes, which contain items. They are modelled in this form:
            |self.boxes[OBJECTNAME] = [(BOX_LENGTH, BOX_WIDTH, BOX_HEIGHT, BOX_THICKNESS),
            | (NUMBER_ITEMS_X, NUMBER_ITEMS_Y, NUMBER_ITEMS_Z, ITEM_LENGTH, ITEM_WIDTH, ITEM_HEIGHT, GRIPPING_DEPTH),
            | (OUTSIDE_GRIP?,[FIRST_ITEM_OFFSET_ORIGIN_X,FIRST_ITEM_OFFSET_ORIGIN_Y,FIRST_ITEM_OFFSET_ORIGIN_Z],
            | [GRIP_ORIENTATION_X,GRIP_ORIENTATION_Y,GRIP_ORIENTATION_Z,GRIP_ORIENTATION_W],
            | [ITEM2ITEM_OFFSET_X, ITEM2ITEM_OFFSET_Y, ITEM2ITEM_OFFSET_Z])
            
            .. note:: In this version the gripping point of the object cannot be defined explicitly. 
                It can be defined by the offset from the origin but this also shifts the center of the object.
        """
        import moveit_commander
        self.scene = moveit_commander.PlanningSceneInterface()
        while self.scene._pub_co.get_num_connections()<1 and not rospy.is_shutdown():
                rospy.sleep(0.1) 
        self.post = Post(self)
        #name: (box_length, box_width,box_height,box_thickness),(num_items_x, num_items_y, num_items_z,_item_length, item_width, item_height,item_grip_depth)
        # (outside_grip, position[x,y,z], orientation[x,y,z,w],offset[x,y,z])
        
        self.boxes = {
                      'tray':[(0.167,0.325,0.06,0.01),{'drop_off_goblet_blue':[0.167/2+0.02,0.06,-0.03,1,0,0,0],'drop_off_goblet_red':[0.167/2+0.02,0.19,0.0,1,0,0,0],'drop_off_item_tray':[0.167/2-0.005,0.115,0.02,1,0,0,0]}],
                      'table':[(0.71,0.5,1.5,0.4),{'item_tray':[0.1,0.05,0.06+0.01*12,0,0,0,1],'goblet_red':[0.53,0.13,0.058+0.00375*12,0,0,0,1],'goblet_blue':[0.3+0.167,0.05,0.105,0,0,0.7,0.7]}],
                      'item_tray':[(0.167,0.325,0.06+12*0.01,0.001),(1,1,12,0.167,0.325,0.06,0.05),(1,[0.167/2,0.115,-0.00],[1,0,0,0],[0.0,0.0,0.01])],
                      'goblet_red':[(0.112,0.112,0.058+0.00375*12,0.012),(1,1,12,0.112,0.112,0.058,0.055),(0,[0.112/2,0.112/2,0],[1,0,0,0],[0.0,0.0,0.00375])],
                      'goblet_blue':[(0.24,0.17,0.105,0.001),(3,2,2,0.072,0.065,0.04,0.02),(0,[0.05,0.05,-0.05],[1,0,0,0],[0.075,0.075,0.01])],
                      'wako':       [(0.565,0.295,0.21,0.005),   {'drop_off_cover_small':[0.07, 0.06, 0.045,-0.701, 0.713, 0.008, 0.010],'drop_off_switch': [0.339, 0.07,0.02 ,1,0,0,0],'drop_off_cover_door':[0.46, 0.06, -0.00,1.000, 0.024, 0.004, 0.019]}],#'drop_off_cover_door':[0.304, 0.262, 0.055,1.000, 0.024, 0.004, 0.019]}],
                      'switch':     [(0.593,0.395,0.28,0.015),(7,4,1,0.063,0.063,0.09,0.013),(1,[0.08,0.073,-0.03],[0.986,-0.16,0.017,0.003],[0.069,0.075,-0.085])],
                      'cover_small':[(0.59,0.39,0.09,0.015),  (4,4,1,0.075,0.075,0.07,0.030),(0,[0.107,0.060,-0.015],[1.000, 0.027, 0.008, 0.000],[0.129,0.081666,0.0])],
                      'cover_door':[(0.59,0.39,0.085,0.02),  (4,4,1,0.075,0.075,0.052,0.05),(1,[0.12,0.065,-0.01],[1.000, 0.024, 0.004, 0.019],[0.13,0.08,0.0])],
                      }
        self.cleanUp()
#         self.makeTables()
#         self.gripperLength={'left':0.125,'right':0.125} # short grippers
#         self.gripperLength={'left':0.162,'right':0.162} #long grippers
        self.gripperLength={'left':0.11,'right':0.125} # left vac gripper ; right short elec gripper
        self.gripperOffset = 0.045
        self.attachGrippers()
        self.cur_object ={'left':"",'right':""}
       
    def otherSide(self,side):
        """
            Return the name of the other arm
            
            :param side: One name of an arm
            :type side: str
            :return: Name of the other arm
            :rtype: str    
        """
        if side=='left':
            return 'right'
        elif side=='right':
            return 'left'
        else:
            rospy.logwarn("unknown request: %s",side)
            return False   
     
    def attachGripper(self,side,type,opened=True):
        """
            Add grippers objects in the virtual scene to include them in collision free planing
            
            :param side: Side where the gripper should be attached
            :type side: str
            :param type: The gripper type ["electric" or "suction"]
            :type type: str
            :param opened: Should the gripped be added opened?
            :type opened: bool
        """
        otherSide = self.otherSide(side)
        pose = PoseStamped()            
        pose.pose.orientation = Quaternion(*[0.0,0.0,0.0,1])
        pose.header.frame_id = side+"_gripper_base" 
        if opened:
            opening = 0.027
        else:
            opening = 0.01
        touch_links = [otherSide+"_electric_gripper_box",otherSide+"_suction_gripper_box","pedestal",side+"_gripper",
                       side+"_gripper_base",side+"_hand_camera",side+"_hand_range",side+"_hand",side+"_wrist"]
        if type=="electric":
            pose.pose.position = Point(*[0,opening,self.gripperLength[side]/2])
            self.attachCylinder(pose.header.frame_id,side+"_"+ type+"_gripper_box1", deepcopy(pose), (self.gripperLength[side],0.01),touch_links)
            pose.pose.position = Point(*[0,-opening,self.gripperLength[side]/2])
            self.attachCylinder(pose.header.frame_id,side+"_"+type+"_gripper_box2", deepcopy(pose), (self.gripperLength[side],0.01),touch_links)
        elif type=="suction":
            pose.pose.position = Point(*[0.0,0.0,self.gripperLength[side]/2])
            self.attachCylinder(pose.header.frame_id, side+"_"+type+"_gripper_box", deepcopy(pose), (self.gripperLength[side],0.0075),touch_links)
        else:
            return

    def cleanUp(self):
        """
            Removes old grippers, tables and known boxes at startup
        """
        self.scene.remove_attached_object("right_gripper_base","right_electric_gripper_box")
        self.scene.remove_attached_object("left_gripper_base","left_suction_gripper_box")        
        self.removeTables()
        for box in self.boxes.keys():
            self.removeBox(box)
#         rospy.sleep(0.1)
#         self.scene.remove_world_object("right_elec_gripper_box")
#         self.scene.remove_world_object("left_suction_gripper_box")
        
    
    def createPredefinedBox(self,name,box):
        """
            Inserts a pre defined box into the scene and adds the items in the box
            
            :param name: The reference system
            :type name: str
            :param box: The box to be inserted
            :type box: str
        """
        if name.startswith("item_tray") is False and name.startswith("goblet_red") is False :# dont make a box for these items
            self.makeBox(name,self.boxes[box][0])
#         if name=="wako":
#             self.makeBox(name+"2",self.boxes[box][0],(0,self.boxes[box][0][1],0))
#             self.makeBox(name+"3",self.boxes[box][0],(0,-self.boxes[box][0][1],0))
        else:
            if (name.startswith("tray") is False and name.startswith("table") is False and name.startswith("wako") is False ): # dont create items for these
#                 print "creating items"
                
                self.createItems2(name,box)
            
       
    def numItems(self,name):
        """
            Returns the amount of items in a box
            
            :param name: Name of the pre defined box
            :type name: str
            :return: The amount of items
            :rtype: int
        """
#         return 3
        return reduce(mul, self.boxes[name][1][0:3], 1)
        
             
    def ind2sub(self,name,item):
        """
            Returns the subscripts for an item in the box
            
            :param name: Name of the box
            :type name: str
            :param item: index of the item
            :type item: int
            :return: The subscripts as x,y,z
            :rtype: [int,int,int] 
        """
        items = self.boxes[name][1]
        x = (item%(items[0]*items[1]))%items[0]
        y = int((item%(items[0]*items[1]))/items[0])
        z = int(item/(items[0]*items[1]))
        return x,y,z
        
                    
    # items are distributed as messured, also used to grasp them
    def createItems2(self,name,box):
        """
            This function creates the items in a box
            
            :param name: The reference system where the items should be created
            :type name: str
            :param box: The pre defined box name to know the dimensions of the items
            :type box: str
        """
        if not (name=="wako" or name.startswith("tray") is True or name.startswith("table") is True):
            [num_x,num_y,num_z,item_length,item_width,item_height,gripping_depth] = list(self.boxes[box][1])
            [init_x,init_y,init_z]=list(self.boxes[box][2][1])
            [offset_x,offset_y,offset_z]=list(self.boxes[box][2][3])
            if name.startswith("item_tray") is True:
                init_x=item_length/2
                init_y=item_width/2
                init_z=0
            for z in xrange(0,num_z):
                for y in xrange(0,num_y):
                    for x in xrange(0,num_x):
                        if name.startswith("item_tray") is True or name.startswith("goblet_red") is True:
                            offset = (0,0,-self.boxes[box][2][3][2]*z)
                        else:
                            offset=(self.boxes[box][2][1][0]+self.boxes[box][2][3][0]*x -item_length/2,self.boxes[box][2][1][1]+self.boxes[box][2][3][1]*y-item_width/2,self.boxes[box][2][1][2]+self.boxes[box][2][3][2]*z)
                        self.makeBox(name+"_item_"+str(x)+"_"+str(y)+"_"+str(z),(self.boxes[box][1][3],self.boxes[box][1][4],self.boxes[box][1][5],self.boxes[box][0][3]),offset,name)
        
            
        else:
            # create wako internal box
            [length,width,height,thickness]=self.boxes[box][0]
            id=0
            cur_thickness=0.000
            cur_height = 0.26
            
            self.addSubBox(name+"_in_"+str(id),name,length/2,width/2,-height+(height-0.07)/2,length,width,height-0.07)
            id+=1
            self.addSubBox(name+"_in_"+str(id),name,length/2,0.16,0,0.15,0.008,0.07*2)

            
        
    def removeItems(self,name):
        """
            Removes all items of a reference frame
            
            :param name: Reference frame
            :type name: str
        """
        if not (name=="wako" or name.startswith("tray") is True or name.startswith("table") is True):
            if name in self.boxes.keys():
                items = self.boxes[name][1][0:3]
                for z in xrange(0,items[2]):
                    for y in xrange(0,items[1]):
                        for x in xrange(0,items[0]):
                            self.scene.remove_world_object(name+"_item_"+str(x)+"_"+str(y)+"_"+str(z))
                            self.removeBox(name+"_item_"+str(x)+"_"+str(y)+"_"+str(z))
        else:
            for id in xrange(0,7):
                self.scene.remove_world_object(name+"_in_"+str(id))
            

    def makeTables(self,scenario):
        """
            Creates tables around the robot to the corresponding scenario
            
            :param scenario: The target scenario
            :type scenario: str 
        """
        reference = "base"
        #base is at 0.92 from ground
        #base offset around 0.06
        table_height_ground = 0.767
        
        if scenario == "scenarioConsignment2":
            conveyor_belt = 0.63
            robot_base_height_ground = 0.92+0.06
            table_height_base = table_height_ground - robot_base_height_ground
            conveyor_height_base = conveyor_belt - robot_base_height_ground
            self.addSubBox("front_table",reference,0.6,0.0,conveyor_height_base-(conveyor_belt)/2,0.7,1.2,conveyor_belt)
            self.addSubBox("front_table_left",reference,0.6,0.6-0.03,conveyor_height_base+0.08,0.7,0.06,0.16)
        if scenario == "scenarioConsignment":
            conveyor_belt = 0.67
            robot_base_height_ground = 0.92+0.06
            table_height_base = table_height_ground - robot_base_height_ground
            conveyor_height_base = conveyor_belt - robot_base_height_ground
            self.addSubBox("front_table",reference,0.7,0.0,conveyor_height_base,0.75,1.5,0.035)
        #~ self.addSubBox("left_table",reference,-0.3,0.9,table_height_base,1.4,0.55,0.02)
#         self.addSubBox("right_table",reference,-0.3,-0.9,table_height_base,3.0,0.55,0.02)
        #~ self.addSubBox("wall",reference,-0.75,0,0,0.1,3,3) 
        #self.addSubBox("rightwall",reference,-0.3,-1.2,-robot_base_height_ground+1.31/2,0.6,0.01,1.31)
        
    def removeTables(self):
        """
            Removes the tables
        """
#         self.scene.remove_world_object("left_table")
#         self.scene.remove_world_object("right_table")
        self.scene.remove_world_object("front_table")
        self.scene.remove_world_object("front_table_left")

    def makeBox(self,name,dimension=(0.828,0.5,0.33,0.02),offset=(0,0,0),reference=None):
        """
            Creates a new and empty box without a top cover
            
            :param name: Name of the new box. If the reference frame is not defined, the name is also used as reference frame
            :type name: str
            :param dimension:  Size of the box (length, width, height, thickness)
            :type dimension: (float,float,float,float)
            :param offset: The offset from the origin of the frame (x_offset,y_offset,z_offset)
            :type offset: (float,float,float)
            :param reference: The reference frame where the box should be attached
            :type reference: str
        """
        if reference is None:
            reference = name
        length = dimension[0]
        width = dimension[1]
        height = dimension[2]
        thickness = dimension[3]   
            
        #baseplate
        id = 0
        if name.startswith("item_tray") is True or name.startswith("goblet_red") is True or name.startswith("goblet_blue") is True:
            pass
        else:
            if name.startswith("cover_door"):
#                 self.addSubBox(name+"_"+str(id),reference,length/2+offset[0],width/2+offset[1],thickness/2-height+offset[2],length,width,0.001)
                pass
            else:
                self.addSubBox(name+"_"+str(id),reference,length/2+offset[0],width/2+offset[1],thickness/2-height+offset[2],length,width,thickness)
            id+=1
        #left side        
        self.addSubBox(name+"_"+str(id),reference,length/2+offset[0],thickness/2+offset[1],-height/2+offset[2],length,thickness,height)
        #right side
        id+=1
        self.addSubBox(name+"_"+str(id),reference,length/2+offset[0],width-thickness/2+offset[1],-height/2+offset[2],length,thickness,height)
        #front side
        id+=1
        self.addSubBox(name+"_"+str(id),reference,thickness/2+offset[0],width/2+offset[1],-height/2+offset[2],thickness,width,height)
        #back side
        id+=1
        self.addSubBox(name+"_"+str(id),reference,length-thickness/2+offset[0],width/2+offset[1],-height/2+offset[2],thickness,width,height)
        
#         if name.find("tray") != -1:
#             id+=1
#             self.addSubBox(name+"_"+str(id),reference,length/2,0.213,-height+(0.045)/2,length,0.01,0.045)
        
        if name.startswith("wako") is True:
            #grundfuellung
#             id+=1
#             self.addSubBox(name+"_"+str(id),reference,length/2+offset[0],width/2+offset[1],-height+(height-0.07)/2+offset[2],length,width,height-0.07)
            #griff
            id+=1
            self.addSubBox(name+"_"+str(id),reference,length/2+offset[0],0.16+offset[1],0+offset[2],0.15,0.008,0.08*2)
            #unbenutzer grosse luefterablage
#             id+=1
#             self.addSubBox(name+"_"+str(id),reference,length-0.18/2+offset[0],width/2-0.055/2+offset[1],-height/2+offset[2],0.18,width-0.055,height)
            #mittelsteg
            id+=1
            self.addSubBox(name+"_"+str(id),reference,length/2+offset[0],0.16+offset[1],-height/2+offset[2],length,0.008,height)
            #steg zwischen lds und lds blende
            id+=1
            self.addSubBox(name+"_"+str(id),reference,0.17+0.12/2+offset[0],0.135/2+offset[1],-height/2+offset[2],0.12,0.135,height)
              
    def addSubBox(self,name,reference,x,y,z,width,length,thickness):
        """
            Creates one element of a box
        """
        pose = PoseStamped()
        pose.pose.position = Point(*[x,y,z])
        pose.pose.orientation = Quaternion(*[0.0,0.0,0.0,1])
        pose.header.frame_id = reference
        
#         print "adding",name, "with reference",reference
        self.scene.add_box(name,pose,(width,length,thickness))
        
#     def addSubCylinder(self,name,reference,x,y,z,height,radius):
#         """
#             Create one element of a cylinder
#         """
#         pose = PoseStamped()
#         pose.pose.position = Point(*[x,y,z])
#         pose.pose.orientation = Quaternion(*[0.0,0.0,0.0,1])
#         pose.header.frame_id = reference
#         self.addCylinder(name,pose,(height,radius))
    
    def removeBox(self,name):
        """
            Removes a box
            
            :param name: Name of the box to be removed
            :type name: str
        """
        box_parts = 8
        for i in xrange(0,box_parts):
#             print "removing",name+"_"+str(i)
            self.scene.remove_world_object(name+"_"+str(i))
            
#         self.removeItems(name)
    
    
    def removeItem(self,name,item,side):
        """
            Removes a specific item
            
            :param name: Name of the box that contains the item
            :type name: str
            :param item: Index of the item
            :type item: int
            :param side: The side where the item is located
            :type side: str
        """
#         print "removing item",name,item
#         if name in self.boxes.keys():
#         items = self.boxes[name][1]
        [x,y,z] = self.ind2sub(name, item)
        self.scene.remove_world_object(name+"_"+side+"_item_"+str(x)+"_"+str(y)+"_"+str(z))                    
        self.removeBox(name+"_"+side+"_item_"+str(x)+"_"+str(y)+"_"+str(z))
    
    def pickUp(self,side,name,item,grip_side="top"):
        """
            Removes an item of the scene and adds it to the gripper
            
            :param side: The side and gripper where the item should be picked
            :type side: str
            :param name: Name of the box that contains the item
            :type name: str
            :param item: Index of the item
            :type item: int
            :param grip_side: Where the item should be gripped ["top" or "side"]
            :type grip_side: str
        """
        if self.cur_object[side] != "":
            rospy.logwarn(side+" gripper seems not to be empty!")
        if name in self.boxes.keys():
            items = self.boxes[name][1]
            [x,y,z] = self.ind2sub(name, item)
            #self.scene.remove_world_object(name+"_item_"+str(x)+"_"+str(y)+"_"+str(z))
            #print "removing object from",name
            pose = PoseStamped()
            pose.header.frame_id = side+"_gripper_base"
            object = name+"_item_"+str(int(x))+"_"+str(int(y))+"_"+str(int(z))
            self.cur_object[side] = object
            touch_links = ["right_elec_gripper_box","left_suction_gripper_box","pedestal",side+"_gripper",side+"_gripper_base",side+"_hand_camera",side+"_hand_range",side+"_hand",side+"_wrist"]
            if name=="switch" or name.startswith("goblet"):
                if grip_side=="side":
                    pose.pose.position = Point(*[0.0,0.0,self.gripperLength[side]+items[3]/2])
                    pose.pose.orientation = Quaternion(*[-0.5,0.5,0.5,0.5])
                    self.attachCylinder(side+"_gripper_base",object,pose,(items[5],items[3]/2),touch_links)
                elif grip_side=="top": 
                    pose.pose.position = Point(*[0.0,0.0,self.gripperLength[side]- items[6] +items[5]/2])
                    pose.pose.orientation = Quaternion(*[0.0,0.0,0.0,1])
                    self.attachCylinder(side+"_gripper_base",object,pose,(items[5],items[3]/2),touch_links)
                else:
                    rospy.loginfo("This grip side is not implemented")
            else:
                if grip_side=="side":
                    pose.pose.position = Point(*[0.0,0.0,self.gripperLength[side] - items[6] +items[3]/2])
                    pose.pose.orientation = Quaternion(*[0,0.7,0.0,0.7])
                    self.scene.attach_box(side+"_gripper_base",object,pose,(items[3],items[4],items[5]),touch_links)
                elif grip_side=="top":

                    pose.pose.position = Point(*[0.0,0.0,self.gripperLength[side]- items[6] +items[5]/2])
                    pose.pose.orientation = Quaternion(*[1.000, 0.021, 0.003, 0.020])
                    self.scene.attach_box(side+"_gripper_base",object,pose,(items[3],items[4],items[5]),touch_links)
                else:
                    rospy.loginfo("This grip side is not implemented")
        
    
    def release(self,side):
        """
            Releases and object from the gripper
            
            :param side: The side of the gripper that should be released
            :type side: str
        """
        object = self.cur_object[side]
        if object!="":
            self.scene.remove_attached_object(side+"_gripper_base",object)
            rospy.sleep(0.1)
            self.scene.remove_world_object(object)
        self.cur_object[side] = ""
    
#     def release_old(self,side,name,item):
#         if name in self.boxes.keys():
#             items = self.boxes[name][1]
#             [x,y,z] = self.ind2sub(name, item)
#             self.scene.remove_attached_object(side+"_gripper_base",name+"_item_"+str(int(x))+"_"+str(int(y))+"_"+str(int(z)))
#             rospy.sleep(0.1)
#             self.scene.remove_world_object(name+"_item_"+str(int(x))+"_"+str(int(y))+"_"+str(int(z)))
#             
    def __make_cylinder(self, name, pose, size):
        
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = list(size)
        co.primitives = [cyl]
        co.primitive_poses = [pose.pose]
        return co
    
    def addCylinder(self,name,pose,size=(1,1)):
        """
            Adds a cylinder to the current scene
            
            :param name: Name of the new cylinder object
            :type name: str
            :param pose: Pose of the new object
            :type pose: geometry_msgs.msg.PoseStamped
            :param size: Dimensions of the new object (height[m],radius[m])
            :type size: (float, float)
        """
        #size: height, radius
        self.scene._pub_co.publish(self.__make_cylinder(name,pose,size))
        
    def attachCylinder(self,link,name,pose,size,touch_links=[]):
        """
            Attaches a cylinder to a gripper
            
            :param name: Name of the new cylinder object
            :type name: str
            :param pose: Pose of the new object
            :type pose: geometry_msgs.msg.PoseStamped
            :param size: Dimensions of the new object (height[m],radius[m])
            :type size: (float, float)
            :param touch_links: links that are already in collision with the attached object (added links do not trigger a collision)
            :type touch_links: list(str)
        """
        #print "attach object with name", name
        #print "touch links",touch_links
        aco = AttachedCollisionObject()
        aco.object = self.__make_cylinder(name, pose, size)
        aco.link_name = link
        if len(touch_links) > 0:
            aco.touch_links = touch_links
        else:
            aco.touch_links = [link]
        self.scene._pub_aco.publish(aco)
        

if __name__ == '__main__':  
    import moveit_commander
    def removeLaunchArgs(args):    
        return [s for s in args if not s.startswith("__name:=")]
    
    moveit_commander.roscpp_initialize(removeLaunchArgs(sys.argv))
    rospy.init_node('baxterSceneDemo',anonymous=True)
    scene = moveit_commander.PlanningSceneInterface()
    #rospy.sleep(1)
    #bs = BaxterScene(scene)
    rospy.sleep(1)
    from geometry_msgs.msg import *
    pose = PoseStamped()
    pose.header.frame_id = "right_gripper"
    pose.pose.position = Point(*[0.0,0.0,0.02])
    pose.pose.orientation = Quaternion(*[0.0,0.0,0.0,1])
    scene.attach_box("right_gripper","my_attached_gripper",pose,(0.04,0.04,0.04))
    rospy.sleep(3)
    moveit_commander.roscpp_shutdown()
