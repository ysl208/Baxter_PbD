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
from xml.dom.minidom import parseString
from hr_helper.post_threading import Post
import json

class BaxterURDFCreator():
    def __init__(self,arm,datapath):
        self.arm=arm
        self.post=Post(self)
        self.datapath = datapath
    
    def getAllJoints(self):
        all_joints = self.arm["left"].joint_names() + self.arm["right"].joint_names()
        return all_joints
    
    def __getRobotModel(self,output_file):
        description=rospy.get_param("/robot_description")
        f=open(output_file,"w")
        f.write(description)
        f.close()
        
    def setRobotModel(self,output_file=None):
        if output_file is None:
            output_file = self.datapath+"robot.urdf"
        f=open(output_file,"r")
        rospy.set_param("/robot_description",f.read())
        f.close()
        
    def __getLimitsFromJSONFiles(self,files):
        all_limits=[]
        for lfile in files:            
            all_limits+=json.loads(open(lfile,"r").read()).items()
        limits=dict(all_limits)            
        return limits
    
    def __getLimitsFromURDF(self,file):        
        data=open(file,"r").read()
        data=data.strip()
        dom=parseString(data)
        limits={}
        all_joints = self.getAllJoints()
        for jointnode in dom.getElementsByTagName('joint'):
            try:
                j=jointnode.attributes["name"] .value
                if j not in all_joints: continue
                for limitnode in jointnode .getElementsByTagName('limit'):
                    lower=float(limitnode.attributes["lower"].value)
                    upper=float(limitnode.attributes["upper"].value)
                    limits[j]=[lower,upper]                                            
            except Exception,e:
                rospy.logerr("%s"%str(e))
        return limits
        
    def __getLimitsFromRobot(self,file):
        tmp="tmp.urdf"
        self.__getRobotModel(tmp)
        return self.__getLimitsFromURDF(tmp)
    
    def createMoveItURDF(self):
        #Move arm to neutral position
        th_left_arm = self.arm["left"].post.move_to_neutral()
        self.arm["right"].move_to_neutral()
        th_left_arm.join()
        #Find limits
        calib_left=Calibrator(self.arm["left"])
        calib_right=Calibrator(self.arm["right"])
 
        calib_file_left = self.datapath + "calib_left.json"
        calib_file_right = self.datapath + "calib_right.json"
        calib_left.findLimits(calib_file_left)
        calib_right.findLimits(calib_file_right)
        self.updateURDF(calib_file_left,calib_file_right)
    
    def __createURDF(self,infile,outfile,limits):    
        data=open(infile,"r").read()
        data=data.strip()
        dom=parseString(data)
    
        self.__injectLimits(dom,limits)
    
        f=open(outfile,"w")        
        f.write(dom.toxml())
        f.close()
        
    def updateURDF(self,calib_file_left,calib_file_right):
        limitsJSON=self.__getLimitsFromJSONFiles([calib_file_left,calib_file_right])
        print "Measured Limits"
        print limitsJSON
        print self.datapath+"reference.urdf"
        limitsREF=self.__getLimitsFromURDF(self.datapath+"reference.urdf")
        print "Reference Limits"
        print limitsREF
        
        limits=self.__getConservativeLimits(limitsJSON,limitsREF)
        print "Final Limits"
        print limits
        
        self.__getRobotModel(self.datapath+"robot.urdf")        
        self.__createURDF(self.datapath+"robot.urdf",self.datapath+"../urdf/baxter.urdf",limits)

        
    def __injectLimits(self,dom,limits):
        for jointnode in dom.getElementsByTagName('joint'):
            try:
                j=jointnode.attributes["name"] .value
                if j in limits.keys():
                    for limitnode in jointnode .getElementsByTagName('limit'):
                        lowerold=float(limitnode.attributes["lower"].value)
                        upperold=float(limitnode.attributes["upper"].value)
                        lower=limits[j][0]
                        upper=limits[j][1]
                        print "Setting limits for joint %s"%j
                        print "old %f %f"%(lowerold,upperold)
                        print "new %f %f"%(lower,upper)                        
                        limitnode.attributes["lower"].value=str(lower)
                        limitnode.attributes["upper"].value=str(upper)                                
            except Exception,e:
                print e                

    def __getConservativeLimits(self,limitsa,limitsb):
        limits={}
        for j in limitsa.keys():
            lower=max(limitsa[j][0],limitsb[j][0])
            upper=min(limitsa[j][1],limitsb[j][1])
            limits[j]=[lower,upper]        
        return limits
    
    def __setJointOriginXYZ(self,dom,joint,xyz):
        for jointnode in dom.getElementsByTagName('joint'):
            try:
                j=jointnode.attributes["name"] .value
                if j==joint:
                    for origin in jointnode .getElementsByTagName('origin'):
                        xyzold=limitnode.attributes["xyz"].value
                        print "Setting enpoint for %s"%j
                        print "old %s"%(xyzold)
                        print "new %s"%(xyz)
                        limitnode.attributes["xyz"]=xyz
            except Exception,e:
                print e                
    
class Calibrator:
    def __init__(self,arm):
        self.arm=arm
        
    def findLimits(self,outfile):
        names=self.arm.joint_names()
        limits={}
        for j in names:
            print "Calibrating for joint %s"%j
            self.arm.move_to_neutral()
            uplimit=self.__findLimit(j,0.5)
            print "uplimit %f"%uplimit
            self.arm.move_to_neutral()
            downlimit=self.__findLimit(j,-0.5)
            print "downlimit %f"%downlimit
            limits[j]=[downlimit,uplimit]
        s=json.dumps(limits)
        f=open(outfile,"w")
        f.write(s)
        f.close()
        
    def __findLimit(self,joint,speed):
        names=self.arm.joint_names()
        values=[0.0]*len(names)
        vel=dict(zip(names,values))
        vel[joint]=speed
        for i in range(0,80):
            self.arm.set_joint_velocities(vel)
            rospy.sleep(0.1)
        self.arm.exit_control_mode()
        return self.arm.joint_angle(joint)
    
    
if __name__ == '__main__': 
    rospy.init_node("baxter_urdf_creator")
    from baxter_helper import BaxterRobot
    baxter=BaxterRobot(moveit=False)
    baxter.loadArms()
    baxter.enable()
    
    buc = BaxterURDFCreator(baxter.arm,baxter.datapath)
    #~ buc.setRobotModel()
    buc.createMoveItURDF()