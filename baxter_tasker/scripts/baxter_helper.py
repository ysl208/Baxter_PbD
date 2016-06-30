#!/usr/bin/env python
import pdb

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
# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##############################################################################

import os
import sys
import rospy

from baxter_helper_frame import *

from hr_helper.post_threading import Post
from threading import Lock,Thread

from std_msgs.msg import Empty, UInt16
from sensor_msgs.msg import Image,Range
import baxter_interface

from baxter_core_msgs.srv import (
    ListCameras,
    )
from baxter_core_msgs.msg import (    
    EndEffectorProperties,
    AssemblyState,
    )
import baxter_interface
from baxter_image_text_displayer import ImageTextDisplayer
import cv2
import cv_bridge
import numpy as np
from baxter_behaviors import BaxterBehaviors

from baxter_scene import BaxterScene 
from baxter_helper_menu import (
                         BaxterMenuEntries,
                         BaxterMenuComposer,
                         BaxterMenuManager,
                         )
from inspect import getmembers
DEFAULT_RATE=200
from high_level_movement import HighLevelMovement
from baxter_urdf_creator import BaxterURDFCreator
from hr_helper.singleton import Singleton

class BaxterException(Exception): pass

   
class BaxterRecorder():
    """
        Records and plays manually learned trajectories. While two trajectories can be played simultaneously,
        the recording has to take place consecutively
    """
    def __init__(self,baxter,filename = "current_recording_"):
        """
        Records joint data to a file at a specified rate.
        
        :param filename: defines the base name for a recording
        :type filename: str
        """
        self.post = Post(self)
        self.baxter = baxter
        self.filename = str(self.baxter.datapath + filename)
        self.stop_execution = {"left":False,"right":False}
        self.mutex = {"left":Lock(),"right":Lock()}
        self._raw_rate = DEFAULT_RATE
        self._rate = rospy.Rate(DEFAULT_RATE)
        self._start_time = rospy.get_time()
        self.state = False
        
    ###### RECORD ###########
    
    def record(self,side,number=""):
        """
            Records the current joint positions of the given side 
            to a csv file if the cuff button is pressed 
    
            This function will overwrite
            existing files.
            
            :param side: side to be recorded
            :type side: str
            :param number: added to the filename, if previous file should not be overwritten
            :type number: int  
        """
        with self.mutex[side]:
            self.baxter.enable()
            timestamp = 0.0
            file = self.filename + side + str(number)
            self.__recordState(side,True)
            joints = self.baxter.arm[side].joint_names()
            cuff_btn = self.baxter.dio[side+"_lower_cuff"]
            with open(file, 'w') as f:
                f.write('time,')
                f.write(','.join([j for j in joints]) + ',')
                f.write(side+'_gripper\n')
                while self.__recordState(side) and self.baxter.mm.cur_page == "teach":
                    if cuff_btn.state: # only record if cuff is pressed
                        angles = [self.baxter.arm[side].joint_angle(j) for j in joints]
                        f.write("%f," % (timestamp))
                        timestamp +=(1.0/self._raw_rate)
                        f.write(','.join([str(x) for x in angles]) + ',')
                        f.write(str(self.baxter.gripper[side].position()) + '\n')
                    self._rate.sleep()
            rospy.loginfo("stopped recording")


    def __recordState(self,side, state = None):
        """
        Changes or gets recording state
        
        :param side: side of the state that should be modified
        :type side: str
        :param state:
        :type state: bool
        :return: State of the recording
        :rtype: bool
        """
        if rospy.is_shutdown():
            state = True
        
        if not state is None:
            self.state = state
        return self.state
            

    def stopRecording(self,side):
        """
            Stops a recording
            
            :param side: The side that should be recorded
            :type side: str
        """
        try:
            self.__recordState(side,False)
        except Exception,e :
            rospy.logerr("%s"%e)
        
    ##### PLAYBACK ############

    def try_float(self,x):
        """
            Converts something to float
            
            :param x: data to be converted
            :type x: any
            :return: data converted to float or None
            :rtype: float
        """
        try:
            return float(x)
        except ValueError:
            return None

    def stopExecution(self,side,stop=None):
        """
            Sets an execution or gets the current execution state of the "stop" param is None
            
            :param side: side of execution to be modified
            :type side: str
            :param stop: Defines if current executions are stop and if future trajectories are executed.
            :type stop: bool
            :return: State of execution
            :rtype: bool
            
        """
        if not stop is None:
            self.stop_execution[side] = stop
        return self.stop_execution[side]
        
    def clean_line(self,line, names,side):
        """
        Cleans a single line of recorded joint positions

        :param line: the line described in a list to process
        :param names: joint name keys
        :param side: the arm side
        """
        #convert the line of strings to a float or None
        line = [self.try_float(x) for x in line.rstrip().split(',')]
        #zip the values with the joint names
        combined = zip(names[1:], line[1:])
        #take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        #convert it to a dictionary with only valid commands
        command = dict(cleaned)
        side_command = dict((key, command[key]) for key in command.keys()
                            if key[:-2] == side+'_')
        
        return (command, side_command, line)


    def play(self,side, loops=1,number=""):
        with self.mutex[side]:
            
            """
            Loops through csv file
    
            :param filename: the file to play
            :type filename: str
            :param loops: number of times to loop; values < 0 mean 'infinite'
            :type loops: int
    
            Does not loop indefinitely, but only until the file is read
            and processed. Reads each line, split up in columns and
            formats each line into a controller command in the form of
            name/value pairs. Names come from the column headers
            first column is the time stamp
            """
            
            rate = rospy.Rate(DEFAULT_RATE)
            self.baxter.arm[side].set_joint_position_speed(2)
            rospy.loginfo("Playing back: %s" % (self.filename+side+str(number),))
            import os
            if not os.path.isfile(self.filename+side+str(number)):
                rospy.logwarn("File to play back does not exist")
                return 
            with open(self.filename+side+str(number), 'r') as f:
                lines = f.readlines()
            keys = lines[0].rstrip().split(',')
            already_error = False
            l = 0
            # If specified, repeat the file playback 'loops' number of times
            try:
                if len(lines) <= 1:
                    rospy.loginfo("Current recording is empty. Have you recorded a trajectory?")
#                     self.baxter.hlm.stop(True)
                    return False
                self.baxter.mm.addEntriesToCurrentMenu({"Stop "+side+" Arm":self.baxter.bb.stopTeachedPath})
                while (loops < 1 or l < loops) and self.stopExecution(side) is False:
                    i = 0
                    l += 1
                    rospy.loginfo("Moving to start position...")
                    _cmd, cmd_start, _raw = self.clean_line(lines[1], keys,side)
                    
                    self.baxter.arm[side].move_to_joint_positions(cmd_start)
                    start_time = rospy.get_time()
                    for values in lines[1:]:
                        i += 1
                        loopstr = str(loops) if loops > 0 else "forever"
                        cmd, scmd, values = self.clean_line(values, keys,side)
                        while (rospy.get_time() - start_time) < values[0] and self.stopExecution(side) is False:
                            
                            if rospy.is_shutdown():
                                rospy.loginfo("\n Aborting - ROS shutdown")
                                return False
                            if len(scmd):
                                self.baxter.arm[side].set_joint_positions(scmd)
                            if side+'_gripper' in cmd:
                                if already_error is False:
                                    if self.baxter.gripper[side].command_position(cmd[side+'_gripper']) is False:
                                        already_error = True
                                    
                            rate.sleep()
                    rospy.loginfo("Done executing saved path")
            except Exception,e:
                rospy.loginfo("There was a problem in the motion of the %s arm. %s"%(side,str(e)))
                self.baxter.hlm.stop(True)
                return False
            finally:
                self.baxter.mm.removeEntriesFromCurrentMenu(["Stop "+side+" Arm"])
            return True

    def readSequence(self,side):
        with self.mutex[side]:
            """
            Loops through csv file
    
            :param filename: the file to play
            :type filename: str
            """
            
            rate = rospy.Rate(DEFAULT_RATE)
            self.baxter.arm[side].set_joint_position_speed(2)
            number = "01"

            self.filename = "/home/cobotics/ros_ws/src/git/baxter_tasker/baxter_tasker/data/plan_01.txt"
            rospy.loginfo("Reading action sequence: %s" % (self.filename,))
            import os
            if not os.path.isfile(self.filename):
                rospy.logwarn("File to play back does not exist")
                return 
            with open(self.filename, 'r') as f:
                lines = f.readlines()
            keys = lines[0].rstrip().split(',')
            already_error = False
            l = 0

            # If specified, repeat the file playback 'loops' number of times
            try:
                if len(lines) <= 1:
                    rospy.loginfo("Action sequence is empty.")
                    return False
                actionSeq = []
                for action in lines[0:]: #(loops < 1 or l < loops) and self.stopExecution(side) is False:
                    if rospy.is_shutdown():
                        rospy.loginfo("\n Aborting - ROS shutdown")
                        return False

                    actionSeq.append(action.rstrip())

            except Exception,e:
                rospy.loginfo("There was a problem in the action sequence of the %s arm. %s"%(side,str(e)))
                self.baxter.hlm.stop(True)
                return False
            return actionSeq


class Gripper(baxter_interface.gripper.Gripper):
    """ 
        Extends baxter gripper interface with:
        grip and release  commands, 
        replaces open/close
        set a specific position,
        custom calibration,
        check if object has been gripped 
    """
    def __init__(self,side,scene=None):
        """
            :param side: The side for which the gripper object should be created
            :type side: str
            :param scene: If MoveIt is used the scene has to be passed here
            :type: moveit_commander.PlanningSceneInterface()
        """
        if not side in ["left","right"]:
            raise BaxterException,"Error non existing side: %s, please provide left or right"%side        
        baxter_interface.gripper.Gripper.__init__(self,side)
        self.side=side
        self.post=Post(self)
        self.epsilon = 1
        self.open_position = 100
        self.close_position = 0
        self.opened = True
        self.outside_grip = True
        self.scene = scene
#         self.parameters()['moving_force']
        
        
    def open(self,block = True,timeout=8.0):
        """
            Opens the gripper
            
            :param block: If True waits until the gripper finished its movement
            :type block: bool
            :param timeout: Time to try to execute the command (for pneumatic gripper it affects the time to stop the airflow if nothing is grasped)
            :type timeout: float
        """
#         print "opening gripper"
        #~ print "moving force",self.parameters()['moving_force']
        baxter_interface.gripper.Gripper.open(self,block,timeout)
        self.opened = True
        if not self.scene is None:
            self.scene.attachGripper(self.side,self.type(),True)
            
            
        
    def close(self,block = True,timeout=8.0):
        """
            Closes the gripper
            
            :param block: If True waits until the gripper finished its movement
            :type block: bool
            :param timeout: Time to try to execute the command (for pneumatic gripper it affects the time to stop the airflow if nothing is grasped)
            :type timeout: float
        """
#         print "closing gripper"
        #~ print "moving force",self.parameters()['moving_force']
        baxter_interface.gripper.Gripper.close(self,block,timeout)
        self.opened = False
        if not self.scene is None:
            self.scene.attachGripper(self.side,self.type(),False)
        
    def setPosition(self,position,blocking = True):
        self.command_position(position,blocking)
        
    def get_distance(self, limb):
        if limb == "left":
            dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        # convert mm to m and return distance
        return float(dist / 1000.0)


    def calibrateAdvanced(self):
        """
            Gets the farmost open and close position 
        """
        self.open(True)
        rospy.sleep(0.05)
        self.open_position = self.position()
        rospy.logdebug("Calibrate Advanced Open: %f"%self.open_position)
        self.close(True)
        rospy.sleep(0.05)
        self.close_position = self.position()
        rospy.logdebug("Calibrate Advanced Close: %f"%self.close_position)
        self.open(False)
        
    def gripped(self):
        """
            Checks if an object has been gripped
            
            :return True is object is gripped
            :rtype: bool
        """
        #print "max open %f, max close %f, cur pos %f"%(self.open_position,self.close_position,self.position())
        cur_pos = self.position()
        if self.type() == 'electric':
            rospy.sleep(0.1)
            rospy.logdebug("check electric grip with epsilon at close position %f open position %f current %f"%(self.close_position+self.epsilon,self.open_position-self.epsilon,cur_pos) )
            if self.outside_grip:
                if cur_pos > (self.close_position + self.epsilon):
                    return True
                else:
                    return False
            else:
                if cur_pos < (self.open_position - self.epsilon):
                    return True
                else:
                    return False
        if self.type() == 'suction':
            rospy.sleep(0.2)
            if cur_pos == 100:
                return False
            else:
                return True
            return self.gripping()
        
    def grip(self,blocking = True):
        """
            Closes the gripper if outside grip and opens it otherwise 
            
            :param blocking :If True, the method waits until the gripper finished the grip
            :type blocking: bool
        """
        #print "outside grip set to:",self.outside_grip
        if self.type() == 'electric':
            if self.outside_grip:
                self.close(blocking)
            else:
                self.open(blocking)
        if self.type() == 'suction':
            self.close(blocking)
        
    def release(self,blocking = True):
        """
            Opens the gripper if outside grip and closes it otherwise
            
            :param blocking :If True, the method waits until the gripper finished the release
            :type blocking: bool
        """
        #print "outside grip set to:",self.outside_grip
        if self.type() == 'electric':
            if self.outside_grip:
                self.open(blocking)
            else:
                self.close(blocking)
        if self.type() == 'suction':
            self.open(blocking)
        if not self.scene is None:
            self.scene.post.release(self.side)
            
class BaxterDisplay():
    """
        Handles the image publishing to the display of baxter
    """
    def __init__(self,show_local=True):
        """
            :param show_local: If True it show a clone window of baxter's screen on the workstation computer
            :type show_local: bool  
        """
        self._display_pub= rospy.Publisher('/robot/xdisplay',Image,queue_size=1)
        self.show_local  = show_local
        self.display_width=1024
        self.display_height=600
        self.bridge = cv_bridge.CvBridge()
        if self.show_local:
            self.w_name = "Baxter's Display"
            cv2.namedWindow(self.w_name)
    
    def setImage(self,data):
        """
            Sends the image to the display of the robot and displays the image locally
            
            :param data: The image data can be passed as a sensor_msgs.msg.Image() or a string that contains the path to an image file.
            :type data: sensor_msgs.msgs.Image or str
        """
#         print "got image data",type(data)
        if type(data) == str:
            img = cv2.imread(data)        
            data = cv_bridge.CvBridge().cv2_to_imgmsg(img)
            
        elif self.show_local:
            #print "image type",type(data),"data", data
            try:
                img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except cv_bridge.CvBridgeError, e:
                rospy.logerr("%s"%str(e))
             
        if self.show_local:
            res_fac = 2
            dim = (1024/res_fac, 600/res_fac)
            img = cv2.resize(img,dim,interpolation=cv2.INTER_LINEAR)
            cv2.imshow(self.w_name,img)
            cv2.waitKey(30)
        try:
            self._display_pub.publish(data)
        except rospy.ROSException,e:
            pass

class BaxterTextDisplayer:
    """
        Emulates a terminal style output window to write text information to baxter's display
    """
    def __init__(self,datapath,display):
        """
            :param datapath: Path to the data directory that also contains a font file that is used for the text style
            :type datapath: str
            :param display: Uses the display object to send the created "text image" to the screen of baxter
            :type display: BaxterDisplay
        """
        self.post=Post(self)      
        self.text=""
        self.tmpfile= datapath+"display.jpg"
        self.fontfile=datapath+"font/UbuntuMono-R.ttf"
        #print "fontfile",self.fontfile
        self.displayer=ImageTextDisplayer(self.fontfile)
        self.display = display
        self.autorefresh=False
        
    def log(self,text,refresh=False):
        """
            Adds a new line to the output terminal
            
            :param text: Text to be written
            :type text: str
            :param refresh: If the output should be directly updated
            :type refresh: bool
             
        """
        self.text=self.text+text
        if refresh:
          self.refresh()

    def clear(self,refresh=False):
        """
            Removes all text from the terminal
            
            :param refresh: If the output should be directly updated
            :type refresh: bool 
        """
        self.text=""
        if refresh:
            self.refresh()
    
    def write(self,data):
        """
            Adds a new line to the output terminal
            
            :param data: Data to be written -> converted to string
            :type data: any
        """
        self.log(str(data))
        
    def refresh(self):
        """
            Updates the screen with the current saved text
        """
        self.displayer.drawText(self.tmpfile,self.text) 
        self.display.setImage(self.tmpfile)

    def autoRefresh(self,period=1):
        """
            Starts a loop that automatically updates the screen (Qttention: This function is blocking. Should be called with post)
            
            :param period: The time between the updates of the screen
            :type period: float
        """
        if self.autorefresh:
            return
        while not rospy.is_shutdown() and self.autorefresh:
            self.refresh()
            rospy.sleep(period)
#         self.refresh()
    
    def stopAutoRefresh(self):
        """
            Stops the auto refreshing loop
        """
        self.autorefresh=False
        

class DigitalIO(baxter_interface.DigitalIO):
    """ Extends DigitalIO with waitForPressed and blinking leds on button presses"""    
    def __init__(self,id,led=None):
        self.led = led
        self.post=Post(self)
        self.enable_state = [False, True][id.find("shoulder") == -1]
        baxter_interface.DigitalIO.__init__(self,id)
        if id.startswith("left"):
            self.lid = "left_inner_light"
        elif id.startswith("right"):
            self.lid = "right_inner_light"
        else:
            rospy.loginfo("No valid button selected")
    
    def __activateLed(self):
        """
            Activates a led to blink for some time
        """
        if not self.led is None:
            self.led.post.blink(self.lid, 0.5, 4)
    
    def waitForClick(self):
        """
            Waits for a press on the button
        """                
        while not rospy.is_shutdown() and self.state is False: # Wait pressed
            rospy.sleep(0.07)
        self.__activateLed()
        while not rospy.is_shutdown() and self.state is True: # Wait released
            rospy.sleep(0.07)
            
    def callbackOnPressed(self,callback):
        """
            Invokes a callback on a button press. This function is non blocking
            
            :param callback: function to be called 
            :type callback: any
        """
        self.post.__callbackOnPressed(callback)

    def __callbackOnPressed(self,callback):
        """
            Invokes a callback on a button press. This is a blocking function
            
            :param callback: function to be called
            :type callback: any
        """
        last=(not self.enable_state)
        while not rospy.is_shutdown(): # Wait pressed
            if last is (not self.enable_state) and self.state is self.enable_state:
                self.__activateLed()
                callback()
            last=self.state
            rospy.sleep(0.1)
        

class Navigator(baxter_interface.Navigator):
    """ Extends Navigator with waitForPressed  and pressed callbacks Navigator
    """
    def __init__(self,id,led=None):
        """
            :param id: Name of the led
            :type id: str
            :param led: object to control a led
            :type led: baxter_helper.Led  
        """
        self.led = led
        self.post=Post(self)
        baxter_interface.Navigator.__init__(self,id)
        self.id = id + "_inner_light"
        
    def activateLed(self):
        """
            Activates a led to blink for some time
        """
        if self.led!=None:
            self.led.post.blink(self.id, 0.5, 4)
    
    def waitForClickOk(self):
        """
            Waits until the wheel button has been pressed
        """
        self.__waitForClick(lambda: self.button0)

    def waitForClickCancel(self):
        """
            Waits until the arrow button has been pressed
        """
        self.__waitForClick(lambda: self.button1)

    def waitForClickShow(self):
        """
            Waits until the home button has been pressed
        """
        self.__waitForClick(lambda: self.button2)
        
        
    def __waitForClick(self,statefunc):
        """
            Waits until a button has been pressed
            
            :param statefunc: Function to be called on button press 
            :type statefunc: any
        """
        while not rospy.is_shutdown() and statefunc()==False: # Wait pressed
            rospy.sleep(0.1)
        self.activateLed()
        while not rospy.is_shutdown() and statefunc()==True: # Wait released
            rospy.sleep(0.1)
                
    def callbackOnPressedOk(self,callback):
        """
            Invokes a callback on a press on the wheel button. This is a non-blocking function
            
            :param callback: function to be called 
            :type callback: any
        """
        self._callbackOnPressed(lambda: self.button0,callback)

    def callbackOnPressedCancel(self,callback):
        """
            Invokes a callback on a press on the arrow button. This is a non-blocking function
            
            :param callback: function to be called
            :type callback: any 
        """
        self._callbackOnPressed(lambda: self.button1,callback)

    def callbackOnPressedShow(self,callback):
        """
            Invokes a callback on a press on the home button. This is a non-blocking function
            
            :param callback: function to be called 
            :type callback: any
        """
        self._callbackOnPressed(lambda: self.button2,callback)
        
    def callbackWheel(self,callback):
        """
            Invokes a callback on a rotation of the wheel button. This is a non-blocking function
            
            :param callback: function to be called 
            :type callback: any
        """
        self.post.__callbackWheel(lambda: self.wheel,callback)
        
    def __callbackWheel(self,statefunc, callback):
        """
            Invokes a callback on a wheel rotations. This is a blocking function
            
            :param statefunc: wheel position 
            :type statefunc: int
            :param callback: function to be called
            :type callback: any 
        """
        last = statefunc()
        while not rospy.is_shutdown():
            newstate = statefunc()
            if not last is newstate:            
                self.activateLed()
                callback(newstate)
                last = newstate
            rospy.sleep(0.1)

    def _callbackOnPressed(self,statefunc,callback):
        """
            Invokes a callback on a button press. This function is non blocking
            
            :param statefunc: button state 
            :type statefunc: bool
            :param callback: function to be called 
            :type callback: any 
        """
        self.post.__callbackOnPressed(statefunc,callback)

    def __callbackOnPressed(self,statefunc,callback):
        """
            Invokes a callback on a button press. This is a blocking function
            
            :param callback: function to be called 
            :type callback: any 
        """
        last=statefunc()
        while not rospy.is_shutdown(): # Wait pressed
            newstate=statefunc()
            if last is False and newstate is True:
                self.activateLed()
                callback()
            last=newstate
            rospy.sleep(0.1)
        
        
class Head(baxter_interface.Head):
    def __init__(self):
        baxter_interface.Head.__init__(self)
        self.post=Post(self)

    def command_deny(self, timeout=5.0):
        """
        Command the shake head  "No"

        :param timeout: Seconds to wait for the head to shake. If 0, just command once and return.  [0]
        :type timeout: float
        """
        self.set_pan(-0.3)
        self.set_pan(0.3)
        self.set_pan(0)
        
class Led(): 
    def __init__(self):
        """
            Class to control all leds
            Available leds are:
            * 'left_itb_light_outer',
            * 'left_itb_light_inner',
            * 'right_itb_light_outer',
            * 'right_itb_light_inner',
            * 'torso_left_itb_light_outer',
            * 'torso_left_itb_light_inner',
            * 'torso_right_itb_light_outer',
            * 'torso_right_itb_light_inner'
        """
        self.post = Post(self) 
        self.led_names = ['left_outer_light','left_inner_light', 
                          'right_outer_light', 'right_inner_light',
                          'torso_left_outer_light','torso_left_inner_light',
                          'torso_right_outer_light','torso_right_inner_light']
        self.led_handle = {}
        self.led_state = {}
        for led in self.led_names:
            self.led_handle[led] = baxter_interface.DigitalIO(led)
            self.led_state[led] = 0
    
    def enable(self,name):
        """
            Enables a Led
            
            :param name: Name of the led 
            :type name: str
        """
        self.led_handle[name].set_output(True)
        self.led_state[name] = 1
    
    def disable(self,name):   
        """
            Disables a Led
            
            :param name: Name of the led
            :type name: str
        """ 
        self.led_handle[name].set_output(False)
        self.led_state[name] = 0
        pass
    
    def disableAll(self):
        """
            Disables all leds
        """
        for led in self.led_names:
            self.disable(led)
        
    
    def blink(self,name, timeout=0.0, frequency=2.0): #timeout <= 0 blinks endlessly
        """
            Blinks a led for a specific time and frequency (blocking)
            
            :param name: Name of the led
            :type name: str
            :param timeout: Duration to let the led blinking. A value <= 0.0 means infinite time
            :type timeout: float
            :param frequency: Rate, how often a led blinks in one second
            :type frequency: float   
        """
        end = rospy.Time.now()+ rospy.Duration(timeout)
        self.led_state[name] = 1
        while not rospy.is_shutdown():
            start = rospy.Time.now()
            if (start > end and timeout > 0) or self.led_state[name] == 0:
                self.led_handle[name].set_output(False)
                break
            self.led_handle[name].set_output(True)
            rospy.Rate(frequency*2).sleep()
            self.led_handle[name].set_output(False)
            rospy.Rate(frequency*2).sleep()
            
    def blinkAllOuter(self,timeout=0,frequency=2):
        """
            Blinks with all blue leds for a specific time and frequency (blocking)
            
            :param timeout: Duration to let the leds blinking. A value <= 0.0 means infinite time
            :type timeout: float
            :param frequency: Rate, how often all leds blink in one second
            :type frequency: float
        """
        for led in xrange(0,len(self.led_names),2):
            self.post.blink(self.led_names[led],timeout,frequency)
        
    def blinkAllInner(self,timeout=0,frequency=2):
        """
            Blinks with all white leds for a specific time and frequency (blocking)
            
            :param timeout: Duration to let the leds blinking. A value <= 0.0 means infinite time
            :type timeout: float
            :param frequency: Rate, how often all leds blink in one second  
            :type frequency: float 
        """
        for led in xrange(1,len(self.led_names),2):
            self.post.blink(self.led_names[led],timeout,frequency)      
 
class Tee:
    def __init__(self,t1,t2):
        self.t1=t1
        self.t2=t2
        
    def write(self,data):
        self.t1.write(data)
        self.t2.write(data)       
    
    def isatty(self):
        return True
   
from camera import HRCamera
class BaxterCamera:
    def __init__(self,display, textDisplay):
        """
            :param display: sends image to the screen of baxter
            :type display: BaxterDisplay
            :param textDisplay: enables and disables the autorefresh
            :type textDisplay: BaxterTextDisplayer
        """
        self.post = Post(self)
        self.textDisplay = textDisplay
        self.display = display
        self.subscribers = {}
        self.cams = HRCamera()
        self.img_id=0
        self.bridge = cv_bridge.CvBridge()

    def grabImage(self,camera_name,filename):
        """
            Grabs exactly one image from a camera
            
            :param camera_name: The image of the camera that should be saved
            :type camera_name: str
            :param filename: The full path of the filename where this image should be saved
            :type filename: str 
        """
        self.cams.open(camera_name,(640,400))
        msg=rospy.wait_for_message('/cameras/' + camera_name + "/image", Image)        
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(filename,img)
        rospy.loginfo("Saved Image %s"%filename)
        self.cams.close(camera_name)

        
    def closeAllCameras(self):
        """
            Closes all active cameras
        """
        for camera_name in ["left_hand_camera","right_hand_camera","head_camera"]:
            self.closeCamera(camera_name)
        
       
    def startCamera(self,camera_name,open_only=False,resolution= (960, 600)):
        """
            Opens a camera
            
            :param camera_name: Name of the camera to be used
            :type camera_name: str
            :param open_only: If True only opens the camera stream, otherwise the stream is also redirected to the display
            :type open_only: bool
            :param resolution: Resolution of the camera stream
            :type resolution: (int,int) 
        """
        self.cams.open(camera_name)
        self.subscribers[camera_name] = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,self.__republish,None,1)
        
    def __republish(self,msg):
        """
            Sends the camera image to baxter's display
        """              
        self.display.setImage(msg)
        rospy.sleep(0.02)

    def closeCamera(self,camera_name):
        """
            Closes a specific camera
            
            :param camera_name: The name of the camera to be closed 
            :type camera_name: str
        """
        if camera_name in self.subscribers.keys():
            self.subscribers[camera_name].unregister()
            del self.subscribers[camera_name]
        self.cams.close(camera_name)
#             if self.textDisplay.autorefresh == False:
#                 self.textDisplay.post.autoRefresh(1)
            #print "camera closed, autorefresh enabled"

class BaxterRangeSensor():
    """
        Get the distances from the range sensors
    """
    def __init__(self):
        self.__distance = {}
        self.__mutex = {"left":Lock(),"right":Lock()}
        root_name = "/robot/range/"
        sensor_name = ["left_hand_range/state","right_hand_range/state"]
        self.__left_sensor = rospy.Subscriber(root_name + sensor_name[0],Range, callback=self.__sensorCallback, callback_args="left",queue_size=1)
        self.__right_sensor = rospy.Subscriber(root_name + sensor_name[1],Range, callback=self.__sensorCallback, callback_args="right",queue_size=1)
        
    def __sensorCallback(self,msg,side):
        """
            Updates the last saved distance value
            
            :param msg: Range msg of the distance sensor
            :type msg: sensor_msgs.msg.Range
            :param side: Side of the sensor
            :type side: str  
        """
        with self.__mutex[side]:
            self.__distance[side] = msg.range  
        
    def getDistance(self,side):
        """
            Return the last saved distance value
            
            :return: Distance in meters
            :rtype: float 
        """
        with self.__mutex[side]:
            return self.__distance[side]


class BaxterCollisionAvoidance():
    """
        Class to enable and disable the self-collision avoidance. Useful when the robot has to pass objects from one hand to the other
    """
    def __init__(self):
        self.pub1=rospy.Publisher("/robot/limb/left/suppress_collision_avoidance",Empty,queue_size=1)
        self.pub2=rospy.Publisher("/robot/limb/right/suppress_collision_avoidance",Empty,queue_size=1)
        self.running=False
        self.thread=None
        
    def deactivate(self,timeout):
        """
            Deactivates the self-collision avoidance for a specific amount of time (non-blocking) 
            
            :param timeout: How long the collision avoidance is disabled 
            :type timeout: float 
        """
        if not self.thread:
            self.thread=Thread(target=lambda timeout=timeout: self.__deactivate(timeout))
            self.running=True
            self.thread.start()
            rospy.sleep(0.2)
        
    def activate(self):
        """
            Reactivates the self-collision avoidance
        """
        self.running=False
        if self.thread:
            self.thread.join()

    def __deactivate(self,_timeout):
        """
            Worker thread to deactivate the collision avoidance (blocking)
        """
        r=rospy.Rate(10)
        start = rospy.Time.now()
        timeout = rospy.Duration(_timeout)
        rospy.loginfo("WARNING: COLLISION AVOIDANCE DEACTIVATED FOR %f SECONDS"%timeout.to_sec())
        empty=Empty()
        while self.running and not rospy.is_shutdown() and (_timeout==0 or (rospy.Time.now()-start)<timeout):
            #print "WARNING: COLLISION AVOIDANCE DEACTIVATED"
            self.pub1.publish(empty)
            self.pub2.publish(empty)
            r.sleep()
        rospy.sleep(0.2)
        self.thread=None
        rospy.loginfo("WARNING: COLLISION AVOIDANCE REACTIVATED") 

class BaxterSonar:
    """
        Enables or disables the sonar sensors
    """
    def __init__(self):
        self.__sonar_pub = rospy.Publisher("/robot/sonar/head_sonar/set_sonars_enabled",UInt16,queue_size=1)
        self.state = 0
        
    def checkConnection(self):
        """
            Checks if baxter's sonars already subscribed to the publisher
        """
        while not rospy.is_shutdown() and self.__sonar_pub.get_num_connections() < 1:
           # rospy.logwarn("No subscriber for sonar state found yet")
            rospy.sleep(0.01)
        rospy.loginfo("Found a subscriber. Changing sonar state")
        
    def enable(self):
        """
            Enables all sonar sensors
        """
        self.checkConnection()
        self.state = 4095
        self.__sonar_pub.publish(4095)
        
    def disable(self):
        """
            Disables all sonar sensors
        """
        self.checkConnection()
        self.state = 0
        self.__sonar_pub.publish(0)


class BaxterRobot:
    __metaclass__=Singleton
    """
        Principal class that holds interfaces to:
        
        * enable/disable the motors
        * the range sensors
        * the menu
        * the collision avoidance
        * the display
        * the text display
        * the cameras
        * the tf frames
        * the grippers
        * the arms
        * the leds
        * the buttons
        * the recorder
        * the 3D scene
        * the urdf creator
        * and high level motion controller
    """
    def __init__(self,moveit=True):
        """
        :param moveit: Defines if MoveIt should be used
        :type moveit: bool 
        """
        pubrate=rospy.Publisher("/robot/joint_state_publish_rate",UInt16,queue_size=1) 
        
        self.moveit=moveit
        if moveit:
            try:
                from baxter_helper_moveit_limb import MoveItHelper
                MoveItHelper()
            except Exception,e:
                rospy.logwarn("Cannot initialize MoveIt. Starting interface without MoveIt. %s"%str(e))
                self.moveit = False
        self.post=Post(self)
        self.gripper={"left":None,"right":None}
        self.arm={"left":None,"right":None}        
        self.dio={}
        self.navigator={}
        self.led=None
        self.camera=None
        self.menu=None
        self.collision=None
        self.enabler=baxter_interface.RobotEnable()
        self.head=Head()
        self.display=BaxterDisplay(True)
        self.textDisplay=None
        self.datapath=str(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  + "/data/")
        self.frame = None
        self.bb = None
        self.mm = None
        self.rs = None
        self.scene = None
        self.tasker = None
        self.hlm = None
        self.br = None
        self.uc = None
        self.sonar = None
        self.kinect = None
        while not rospy.is_shutdown() and pubrate.get_num_connections() < 1:
            rospy.sleep(0.01)
        pubrate.publish(UInt16(DEFAULT_RATE)) 

        self.motor_state = rospy.wait_for_message("/robot/state",AssemblyState).enabled

    def toggleMotorState(self):
        """
            Enables or disables the motorstate of the robot
        """
        self.led.post.blinkAllOuter(0.5,4)
        if self.motor_state:
            self.disable()
        else:
            self.enable()

    def enable(self):
        """
            Enables the motors
        """
        if self.motor_state:
            rospy.loginfo("Motors seem to be already enabled")
            return
        self.motor_state = True
        self.enabler.enable()

    def disable(self):
        """
            Disables the motors
        """
        if not self.motor_state:
            rospy.loginfo("Motors seem to be already disabled")
            return
        self.motor_state = False
        self.enabler.disable()
        
    def loadAll(self):
        """
            Loads all interfaces
        """
        elements = getmembers(self)
        elements=[e[0] for e in elements if e[0].startswith("load") and not e[0].startswith("loadAll")]
#         rospy.loginfo("Elements to load %s"%str(elements))
        [getattr(self,e)() for e in elements]
        rospy.loginfo("All components are loaded.")
    
    def loadKinect(self):
        """
            Publishes the transformation from the kinect rgb camera to the marker that is mounted on baxter
            
        """
        return
        try:
            from ar_track_alvar.msg import AlvarMarkers
        except:
            #~ rospy.warn("Cannot import AlvarMarkers -> ignoring TF from camera to marker")
            return
            
        if self.kinect is None:
            self.frame = self.loadFrame()
            self.kinect = BaxterKinect(self.frame)
        return self.kinect
        
    
    def loadSonarControl(self):
        """
            Loads Baxter's sonar control and disables the sonars
            
            :return: interface to the sonar sensors
            :rtype: BaxterSonar
        """
        if self.sonar is None: 
            start = rospy.get_time()
            self.sonar = BaxterSonar()
            self.sonar.disable() # disables all 12 sonar sensors
            self.timer("Sonar Control",start)
        return self.sonar
    
    def loadRangeSensor(self):
        """
            Loads Baxter's range sensors
            
            :return: interface to the range sensors
            :rtype: BaxterRangeSensor
        """
        if self.rs is None:
            start = rospy.get_time()
            self.rs = BaxterRangeSensor()
            self.timer("Range Sensor",start)
        return self.rs
    
    def loadMenuManager(self):
        """
            Loads Baxter's menu manager
            
            :return: interface to the menu
            :rtype: BaxterMenuManager
        """
        if self.mm is None:
            self.loadMenu()
            start = rospy.get_time()
            self.mm = BaxterMenuManager(self.menu,self.moveit)
            self.timer("Menu Manager",start)
        return self.mm
    
    def loadBehaviors(self):
        """
            Loads Baxter's custom behaviors
            
            :return: interface to the behaviors
            :rtype: BaxterBehaviors
        """
        if self.bb is None:
            start = rospy.get_time()
            self.bb = BaxterBehaviors(self)
            self.timer("Behaviors",start)  
        return self.bb

    def loadCollisionAvoidance(self):
        """
            Loads the self-collision avoidance interface
            
            :return: interface to the self-collision avoidance
            :rtype: BaxterCollisionAvoidance
        """
        if self.collision is None:
            start = rospy.get_time()
            self.collision = BaxterCollisionAvoidance()
            self.timer("Collision Avoidance",start)
        return self.collision  
    
    def loadMenu(self):
        """
            Loads the menu image composer
            
            :return: interface to draw and select the menu entries
            :rtype: BaxterMenuComposer
        """ 
        if self.menu is None:
            self.textDisplay=self.loadTextDisplay()
            start = rospy.get_time()
            self.menu = BaxterMenuComposer(self.display, self.textDisplay,self.datapath)
            self.timer("Menu Composer",start)
        return self.menu 
    
    def loadCamera(self):
        """
            Prepares the cameras
            
            :return: interface to the cameras
            :rtype: BaxterCamera
        """
        if self.camera is None:
            start = rospy.get_time()
            self.camera = BaxterCamera(self.display,self.textDisplay)
            self.timer("Cameras",start)
        return self.camera
    
    def loadFrame(self):
        """
            Prepares the tf broadcaster and listener
            
            :return: interface to the transformations
            :rtype: BaxterFrame
        """
        if self.frame is None:
            self.loadBaxterScene()
            start = rospy.get_time()
            self.frame = BaxterFrame(self)
            self.timer("Frame",start)
        return self.frame    
    
    def __loadGripper(self,side,force_reload=False):
        """
            Loads one gripper
            
            :param side: Gripper to be loaded
            :type side: str  
            :param force_reload: Recalibrate gripper? 
            :type force_reload: bool 
        """   
        self.gripper[side]=Gripper(side,self.scene)
        gripper_id = rospy.wait_for_message('/robot/end_effector/'+side+'_gripper/properties',EndEffectorProperties,5.0)
        gripper_name = gripper_id.product
        if gripper_name == 'Electric Parallel Gripper':
            try:
                
                if self.gripper[side].calibrated() is False or force_reload is True:
                    self.gripper[side].calibrate()
                    rospy.sleep(1)
                self.gripper[side].calibrateAdvanced()
            except Exception,e:
                rospy.logerr("check also if emergency button is popped out. error: %s"%str(e))
        elif gripper_name == 'Suction Cup Gripper':
            pass
        else:
            rospy.loginfo("Unknown Gripper with name %s loaded"%gripper_name) 
    
    def loadGrippers(self,force_reload=False):
        """
            Loads both grippers
            
            :param force_reload: Recalibrate grippers? 
            :type force_reload: bool
            :return: interface to the grippers
            :rtype: Gripper
        """ 
        self.scene = self.loadBaxterScene()
        if self.gripper["left"] is None or self.gripper["right"] is None or force_reload is True:
            start = rospy.get_time()
            th_left_gripper  = self.post.__loadGripper("left",force_reload )
            self.__loadGripper("right",force_reload)
            th_left_gripper.join()
            self.timer("Grippers",start)
        return self.gripper
        
    def __loadArm(self,side):
        """
            Load an arm
            
            :param side: Arm to be loaded
            :type side: str 
        """
        if self.moveit:
            from baxter_helper_moveit_limb import MoveItLimb
            self.arm[side]=MoveItLimb(side)
        else:
            from baxter_helper_simple_limb import SimpleLimb
            self.arm[side]=SimpleLimb(side,True)
        rospy.sleep(0.2) # wait until subscribers are active
        
    def loadArms(self):
        """
            Load both arms
            
            :return: interface to the arms/limbs
            :rtype: SimpleLimb or MoveItLimb
        """
        if self.arm["left"] is None or self.arm["right"] is None:
            start = rospy.get_time()
            th_left_arm  = self.post.__loadArm("left")
            self.__loadArm("right")
            th_left_arm.join()
            self.timer("Arms",start)
        return self.arm
        
    def loadLeds(self):
        """
            Loads all leds
            
            :return: interface to the leds
            :rtype: Led
        """
        if self.led is None:
            start = rospy.get_time()
            self.led = Led()
            self.timer("Leds",start)
        return self.led

    def loadDigitalIO(self):
        """
            Loads the buttons
            
            :return: interface to the buttons
            :rtype: DigitalIO
        """
        
        l=["right_lower_button","right_upper_button","right_lower_cuff","right_shoulder_button",
           "left_lower_button" ,"left_upper_button" ,"left_lower_cuff" ,"left_shoulder_button"]
        if self.dio == {}:
            self.led=self.loadLeds()        
            start = rospy.get_time()
            for id in l:
                self.dio[id]=DigitalIO(id,self.led)
            self.timer("Digital IO",start)
        return [self.led,self.dio,l]
            
    def loadNavigator(self):
        """
            Loads the navigators
            
            :return: interface to the navigators
            :rtype: Navigator
        """
        l=["right","left","torso_right","torso_left"]
        if self.navigator == {}:
            self.led=self.loadLeds()
            start = rospy.get_time()
            for id in l:
                self.navigator[id]=Navigator(id,self.led)
            self.timer("Navigators",start)
        return [self.navigator,l]
            
    def yes(self):
        """
            The robots nods with the display
        """
        self.head.command_nod()

    def no(self):
        """
            The robot shakes its display
        """
        self.head.command_deny()
    
    def loadTextDisplay(self,autostdout=True):
        """
            Loads the text display
            
            :param autostdout: Should the text display linked to the standard prints?
            :type autostdout: bool  
            :return: interface to the text display
            :rtype: BaxterTextDisplayer
        """     
        if self.textDisplay is None:
            start = rospy.get_time()   
            self.textDisplay=BaxterTextDisplayer(self.datapath,self.display)
            if autostdout:
                import sys
                sys.stdout=Tee(self.textDisplay,sys.stdout)
                sys.stderr=Tee(self.textDisplay,sys.stderr)
                self.textDisplay.post.autoRefresh(1)
            self.timer("Text Display",start)
        return self.textDisplay
    
    def loadRecorder(self):
        """
            Prepares the recorder
            
            :return: interface to the recorder
            :rtype: BaxterRecorder
        """
        if self.br is None:
            self.gripper=self.loadGrippers()    
            self.arm=self.loadArms()
            start = rospy.get_time()
            self.br = BaxterRecorder(self)
            self.timer("Recorder",start)
        return self.br
    
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
    
    def loadBaxterScene(self):
        """
            Initializes the MoveIt scene
            
            :return: interface to the moveit scene
            :rtype: BaxterScene
        """
        if self.moveit and self.scene is None:
            start = rospy.get_time()
            self.scene = BaxterScene()
            self.timer("Baxter Scene",start)
        return self.scene
        
    def addTasker(self,tasker):
        """
            Adds the GUI task interface of the workstation
        """
        self.tasker = tasker
    
    def loadHighLevelMovement(self):
        """
            Loads the high level movement controller
            
            :return: interface to high level motions
            :rtype: HighLevelMovement
        """
        if self.hlm is None:
            start = rospy.get_time()
            self.hlm = HighLevelMovement(self)
            self.timer("High Level Movement",start)
        return self.hlm
    
    def loadURDFCreator(self):
        """
            Loads the automatic URDF creator
            
            :return: interface to the automatic urdf creator
            :rtype: BaxterURDFCreator
        """
        if self.uc is None:
            if self.arm["left"] is None or self.arm["right"] is None:
                self.loadArms()
            start = rospy.get_time()
            self.uc = BaxterURDFCreator(self.arm,self.datapath)
            self.timer("URDF Creator",start)
        return self.uc
    
    def timer(self,name,start):
        rospy.loginfo("Loading time for %s was: %.4f"%(name,float(rospy.get_time() - start)))

if __name__ == '__main__':        
    rospy.init_node("baxter_helper")
    baxter=BaxterRobot()
    baxter.loadAll()

    
        
