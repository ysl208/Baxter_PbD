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
from Tkinter import *
import tkSimpleDialog
import tkFileDialog
import rospy

from hr_helper.post_threading import Post
from threading import Lock
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    ListCameras,
    )
from baxter_core_msgs.msg import (    
    EndEffectorProperties,
    )
import baxter_interface
from moveit_msgs.srv import *
from moveit_msgs.msg import *

from baxter_helper import BaxterRobot
from baxter_helper_abstract_limb import *
from baxter_helper_moveit_limb import MoveItLimb


REFRESH_DELAY_MS=2000

GRIPPERS=False
ARMS=True



class ArmController:
    def __init__(self,main,title,arm):
        self.main=main
        self.title=title
        self.arm=arm
        self.arm.title=title
        
    def buildGUI(self,parent):
        lframe=LabelFrame(parent,text=self.title)
        self.lframe=lframe
        row=0
        Button(lframe,text="Record",command=lambda arm=self.arm: self.main.record(arm)    ).grid(row=row,column=0)
        Button(lframe,text="goToAngles",command=lambda arm=self.arm: self.main.goToAngles(arm)    ).grid(row=row,column=1)
        Button(lframe,text="goToPose",command=lambda arm=self.arm: self.main.goToPose(arm)    ).grid(row=row,column=2)
        row+=1
        if type(self.arm)==MoveItLimb:
            Button(lframe,text="goToAngles_plan",command=lambda arm=self.arm: self.main.goToAnglesPlan(arm)    ).grid(row=row,column=1)
            Button(lframe,text="goToPose_plan",command=lambda arm=self.arm: self.main.goToPosePlan(arm)    ).grid(row=row,column=2)
            row+=1
            

        Label(lframe,text="Angles:").grid(row=row,column=0)
        self.angles= StringVar()
        entryAngles= Entry(lframe, textvariable=self.angles,width=40)
        entryAngles.grid(row=row,column=1,columnspan=2)
        row+=1
        
        Label(lframe,text="Position:").grid(row=row,column=0)
        self.position= StringVar()
        entryPosition= Entry(lframe, textvariable=self.position,width=40)
        entryPosition.grid(row=row,column=1,columnspan=2)
        row+=1

        Label(lframe,text="Orientation:").grid(row=row,column=0)
        self.orientation= StringVar()
        entryOrientation= Entry(lframe, textvariable=self.orientation,width=40)
        entryOrientation.grid(row=row,column=1,columnspan=2)

        self.lframe.after(REFRESH_DELAY_MS,self.refresh)

        return lframe    
    

    def refresh(self):

        angles=self.arm.getAngles()
        
        #~ fkpose=self.arm.getPoseFromAngles(angles)
        #~ print fkpose
        
        dangles=getDictFromAngles(angles)
        sangles=""
        for v in dangles.values():
            sangles+="%.3f,"%v                    
        self.angles.set(sangles)

        pose=self.arm.getPose()
        sposition="%.3f,%.3f,%.3f"%(pose.position.x,pose.position.y,pose.position.z)
        self.position.set(sposition)
        sorientation="%.3f,%.3f,%.3f,%.3f"%(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        self.orientation.set(sorientation)
        self.lframe.after(REFRESH_DELAY_MS,self.refresh)

        #~ ikangles=self.arm.getAnglesFromPose(pose)
        #~ print ikangles




class GripperController:
    def __init__(self,main,title,gripper,dio):
        self.main=main
        self.title=title
        self.gripper=gripper
        self.dio=dio
        self.dio.callbackOnPressed(self.toggle)
        
    def buildGUI(self,parent):
        lframe=LabelFrame(parent,text=self.title)
        self.lframe=lframe
        Button(lframe,text="Grip",command=lambda: self.gripper.grip()   ).grid(row=0,column=0)
        Button(lframe,text="Release",command=lambda: self.gripper.release()  ).grid(row=0,column=1)
        Button(lframe,text="Test",command=lambda: self.test()  ).grid(row=0,column=2)

        Label(lframe,text="Position:").grid(row=1,column=0)
        self.position= StringVar()
        entryPosition= Entry(lframe, textvariable=self.position,width=20)
        entryPosition.grid(row=1,column=1,columnspan=2)

        self.lframe.after(REFRESH_DELAY_MS,self.refresh)

        return lframe    
    

    def refresh(self):

        position=self.gripper.position()
        self.position.set(str(position))

        self.lframe.after(REFRESH_DELAY_MS,self.refresh)

    def toggle(self):
        if self.gripper.gripping():
            self.gripper.release()
        else:
            self.gripper.grip()
        
    def test(self):
        while not rospy.is_shutdown():
            self.gripper.grip()
            rospy.sleep(0.1)
            print self.gripper.position()
            self.gripper.release()
    
class MainWindow:

    def __init__(self,master,baxter):
        self.master=master

        self.post=Post(self)
        self.frame = master
        self.master.bind('<Key-Escape>', self.exit )
        self.baxter=baxter
        
        


        self.pose_index=0
        
        #~ Button(self.frame,text="Enable",command=lambda: self.baxter.enable()    ).grid(row=0,column=0)
        #~ Button(self.frame,text="Disable",command=lambda: self.baxter.disable()    ).grid(row=1,column=0)

        self.buildParametersFrame(self.frame,"Parameters").grid(row=0,column=0)

        Button(self.frame,text="REMOVE COLLISIONS",command=lambda: self.baxter.collision.deactivate(0)    ).grid(row=0,column=1)
        Button(self.frame,text="ACTIVATE COLLISIONS",command=lambda: self.baxter.collision.activate()    ).grid(row=0,column=2)

        

        if ARMS:
            self.arms_right=[baxter.arm["right"].simple ,baxter.arm["right"]]
            self.arms_left=[baxter.arm["left"].simple ,baxter.arm["left"]]
            self.arms={"right":self.arms_right,"left":self.arms_left}

            Button(self.frame,text="Compare Left",command=lambda: self.compare("left")    ).grid(row=1,column=0)
            Button(self.frame,text="Compare Right",command=lambda: self.compare("right")    ).grid(row=1,column=1)

            
            self.sleft=ArmController(self,"Simple Left Arm",self.arms_left[0])
            self.sleft.buildGUI(self.frame).grid(row=2,column=0)

            self.sright=ArmController(self,"Simple Right Arm",self.arms_right[0])
            self.sright.buildGUI(self.frame).grid(row=2,column=1)

            self.mleft=ArmController(self,"MoveIt Left Arm",self.arms_left[1])
            self.mleft.buildGUI(self.frame).grid(row=3,column=0)

            self.mright=ArmController(self,"MoveIt Right Arm",self.arms_right[1])
            self.mright.buildGUI(self.frame).grid(row=3,column=1)

            Button(self.frame,text="Execute Plan",command=lambda: self.executePlan(self.arms_left[1])    ).grid(row=4,column=0)
            Button(self.frame,text="Execute Plan",command=lambda: self.executePlan(self.arms_right[1])    ).grid(row=4,column=1)

            Button(self.frame,text="Execute Plan Start",command=lambda: self.executePlanFromStart(self.arms_left[1])    ).grid(row=5,column=0)
            Button(self.frame,text="Execute Plan Start",command=lambda: self.executePlanFromStart(self.arms_right[1])    ).grid(row=5,column=1)

        if GRIPPERS:
            self.gleft=GripperController(self,"Left Gripper",self.baxter.gripper["left"],self.baxter.dio["left_lower_button"])
            self.gleft.buildGUI(self.frame).grid(row=4,column=0)

            self.gright=GripperController(self,"Right Gripper",self.baxter.gripper["right"],self.baxter.dio["right_lower_button"])
            self.gright.buildGUI(self.frame).grid(row=4,column=1)
        
    
    def buildParametersFrame(self,parent,name):
        lframe=LabelFrame(parent,text=name)
        row=0
        
        Label(lframe,text="Speed:").grid(row=row,column=0)
        self.speed= DoubleVar()
        self.speed.set(DEFAULT_SPEED)
        entrySpeed= Entry(lframe, textvariable=self.speed)
        entrySpeed.grid(row=row,column=1)
        row+=1
        
        Label(lframe,text="Position tolerance:").grid(row=row,column=0)
        self.position_tolerance= DoubleVar()
        self.position_tolerance.set(DEFAULT_POSITION_TOLERANCE)
        Entry(lframe, textvariable=self.position_tolerance).grid(row=row,column=1)
        row+=1
        
        Label(lframe,text="Orientation tolerance:").grid(row=row,column=0)
        self.orientation_tolerance= DoubleVar()
        self.orientation_tolerance.set(DEFAULT_ORIENTATION_TOLERANCE)
        Entry(lframe, textvariable=self.orientation_tolerance).grid(row=row,column=1)
        row+=1
        
        Label(lframe,text="Joint tolerance plan:").grid(row=row,column=0)
        self.joint_tolerance_plan= DoubleVar()
        self.joint_tolerance_plan.set(DEFAULT_JOINT_TOLERANCE_PLAN)
        Entry(lframe, textvariable=self.joint_tolerance_plan).grid(row=row,column=1)
        row+=1

        Label(lframe,text="Joint tolerance execution:").grid(row=row,column=0)
        self.joint_tolerance= DoubleVar()
        self.joint_tolerance.set(DEFAULT_JOINT_TOLERANCE)
        Entry(lframe, textvariable=self.joint_tolerance).grid(row=row,column=1)
        row+=1

        Label(lframe,text="Speed tolerance:").grid(row=row,column=0)
        self.speed_tolerance= DoubleVar()
        self.speed_tolerance.set(DEFAULT_SPEED_TOLERANCE)
        Entry(lframe, textvariable=self.speed_tolerance).grid(row=row,column=1)
        row+=1
        
        Label(lframe,text="Timeout:").grid(row=row,column=0)
        self.timeout= DoubleVar()
        self.timeout.set(DEFAULT_TIMEOUT)
        Entry(lframe, textvariable=self.timeout).grid(row=row,column=1)
        row+=1
        
        Label(lframe,text="Path tolerance:").grid(row=row,column=0)
        self.path_tolerance= DoubleVar()
        self.path_tolerance.set(0)
        Entry(lframe, textvariable=self.path_tolerance).grid(row=row,column=1)
        row+=1
                
        Label(lframe,text="Cartesian path:").grid(row=row,column=0)
        self.cartesian= IntVar()
        self.cartesian.set(0)
        Checkbutton(lframe, variable=self.cartesian).grid(row=row,column=1)
        row+=1
        
        Label(lframe,text="Reverse path:").grid(row=row,column=0)
        self.reverse_path= IntVar()
        self.reverse_path.set(0)
        Checkbutton(lframe, variable=self.reverse_path).grid(row=row,column=1)
        
        return lframe
   
    def record(self,arm):
        name=tkSimpleDialog.askstring("Record","Name:")
        if not name: return
        
        i=self.pose_index
        self.pose_index+=1
        
        angles=arm.getAngles()
        print angles
        pose=arm.getPose()
        print pose
                
        print "Recording Pose %s"%name
        savePose(name,pose)
        print "Recording Angles %s"%name
        saveAngles(name,angles)


    def goToAngles(self,arm):
        ret=tkSimpleDialog.askstring("GoTo Angles","Name:")
        if ret:
            angles=loadAngles(ret)
            print "Going to angles"
            print angles
            start=rospy.get_time()
            ret=arm.goToAngles(angles,speed=self.speed.get(),joint_tolerance_plan=self.joint_tolerance_plan.get(),joint_tolerance=self.joint_tolerance.get(),speed_tolerance=self.speed_tolerance.get(),timeout=self.timeout.get(),path_tolerance=self.path_tolerance.get())
            print "Returned "+str(ret)
            delay=(rospy.get_time()-start) 
            print "Trajectory Delay %f"%delay
                

    def goToAnglesPlan(self,arm):
        ret=tkSimpleDialog.askstring("GoTo Angles Plan","Angles:")
        if not ret: return
        angles=loadAngles(ret)
        planname=tkSimpleDialog.askstring("GoTo Angles Plan","Plan:")
        if not planname: return
        print "Planning to angles"
        print angles
        plan=arm.goToAngles_plan(angles,joint_tolerance_plan=self.joint_tolerance_plan.get())
        savePlan(planname,plan)
        print plan
        print "Saved plan as "+planname
                
  
    def goToPose(self,arm):
        ret=tkSimpleDialog.askstring("GoTo Pose","Name:")
        if ret:
            pose=loadPose(ret)
            
            ps=PoseStamped()            
            ps.pose=pose
            name="/target"+arm.title
            ps.header.frame_id="base"
            baxter.frame.setTF(name,ps)
            
            print "Going to pose"
            print pose
            cartesian=self.cartesian.get()==1
            start=rospy.get_time()
            ret=arm.goToPose(pose,speed=self.speed.get(),position_tolerance=self.position_tolerance.get(),orientation_tolerance=self.orientation_tolerance.get(),joint_tolerance=self.joint_tolerance.get(),speed_tolerance=self.speed_tolerance.get(),timeout=self.timeout.get(),cartesian=cartesian,path_tolerance=self.path_tolerance.get())
            print "Returned "+str(ret)
            
            delay=(rospy.get_time()-start)
            print "Trajectory Delay %f"%delay

    def goToPosePlan(self,arm):
        ret=tkSimpleDialog.askstring("GoTo Pose Plan","Pose:")
        if not ret: return
        pose=loadPose(ret)
        
        ps=PoseStamped()            
        ps.pose=pose
        name="/target"+arm.title
        ps.header.frame_id="base"
        baxter.frame.setTF(name,ps)

        planname=tkSimpleDialog.askstring("GoTo Pose Plan","Plan:")
        if not planname: return
        
        print "Planning to pose"
        print pose
        cartesian=self.cartesian.get()==1
        start=rospy.get_time()
        plan=arm.goToPose_plan(pose,position_tolerance=self.position_tolerance.get(),orientation_tolerance=self.orientation_tolerance.get(),cartesian=cartesian)
        savePlan(planname,plan)
        print plan
        print "Saved plan as "+planname

    def executePlan(self,arm):
        ret=tkSimpleDialog.askstring("Execute Plan","Name:")
        if ret:
            plan=loadPlan(ret)
            if self.reverse_path.get()==1:
                plan=getReversePlan(plan)
            print plan
            start=rospy.get_time()
            ret=arm.execute(plan,speed=self.speed.get(),joint_tolerance=self.joint_tolerance.get(),speed_tolerance=self.speed_tolerance.get(),timeout=self.timeout.get(),path_tolerance=self.path_tolerance.get())
            delay=rospy.get_time()-start
            print "Plan delay %f"%delay
            print "Returned "+str(ret)



    def executePlanFromStart(self,arm):
        ret=tkSimpleDialog.askstring("Execute Plan From Start","Name:")
        if ret:
            plan=loadPlan(ret)
            if self.reverse_path.get()==1:
                plan=getReversePlan(plan)
            print plan
            jnames=plan.joint_trajectory.joint_names
            jangles=plan.joint_trajectory.points[0].positions
            angles=getAnglesFromDict(dict(zip(jnames,jangles)))                        
            
            arm.simple.goToAngles(angles)
            
            start=rospy.get_time()            
            ret=arm.execute(plan,speed=self.speed.get(),joint_tolerance=self.joint_tolerance.get(),speed_tolerance=self.speed_tolerance.get(),timeout=self.timeout.get(),path_tolerance=self.path_tolerance.get())
            delay=rospy.get_time()-start
            print "Plan delay %f"%delay
            print "Returned "+str(ret)
                        


        
    def compare(self,side):
        arms=self.arms[side]
        pa=arms[0].getPose()
        pb=arms[1].getPose()
        aa=arms[0].getAngles()
        ab=arms[1].getAngles()
        print "DELTA ANGLES"
        print getAnglesDiff(aa,ab)
        print "DELTA POSE"
        print getPoseDiff(pa,pb)
            
        
    def exit(self,event=None):
        print "exit"
        self.master.destroy()
        sys.exit()
        
        
if __name__ == '__main__': 

    rospy.init_node("baxter_monitor",log_level=rospy.DEBUG)
    
    print "Creating Baxter Robot"
    baxter=BaxterRobot(moveit=True)
    if ARMS:
        baxter.loadArms() 
    if GRIPPERS:
        baxter.loadGripper("right")    
        baxter.loadGripper("left")
    baxter.loadDigitalIO()
    baxter.loadFrame()
    baxter.loadCollisionAvoidance()

    root = Tk()
    root.title("Baxter Monitor")
    
    mainwindow = MainWindow(root,baxter)
    
    root.protocol("WM_DELETE_WINDOW", mainwindow.exit)
    
    #root.after(0,rospy.sleep(10))
    root.mainloop()    
    
    