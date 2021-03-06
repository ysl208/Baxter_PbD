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
from boxrenderer import BoxRenderer1
import baxter_helper_abstract_limb
RENDER_ERRORS=True
#ATTENTION: All functions with a name that starts with "scenario" will be automatically available in the run menu.
class BaxterScenarios:
    """
        This class contains all scenarios that are displayed in the scenario list of the robot. 
        For all function names that start with "scenario" a callable button is automatically created. 
    """
    def __init__(self,baxter):
        """
            :param baxter: object to access all robot functions
            :type baxter: BaxterRobot
        """
        self.baxter = baxter
        self.post = Post(self)
        self.__scenario = ""
        #SCENARIO PICK AND PLACE
        self.pnp_index = 0
        
        #SCENARIO Consignment
        self.renderer=BoxRenderer1(
            self.baxter.datapath+"box1.png",
            self.baxter.datapath+"element1.png",
            self.baxter.datapath+"mark1.png",
            self.baxter.datapath+"notpicked1.png",
            self.baxter.datapath+"nothanded1.png",
            )
        self.skip_first = True
        self.id = {"switch": -1, "cover_small":-1,"cover_door":-1}
        self.item_dropped = False
        self.recovery = "Skip"
        self.object_to_consign = 3
        #SCENARIO consignment2
        
    def getScenario(self):
        """
            Return the currently selected scenario
            
            :return: the active scenario
            :rtype: str
        """
        return self.__scenario
        
    def setScenario(self,scenario):
        """
            Sets a scenario
            
            :param scenario: the name of the scenario to set active
            :type scenario: str
        """
        self.__scenario = scenario
        
    def createScenarioName(self,**kwargs):
        try:
            scenario_name = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current scenario name")
            scenario_name = ""
        return "scenario"+scenario_name
        
    def resetItems(self,**kwargs):
        """
            Resets the id of the items in the consignment scenario.
            
            .. note:: Add your code here to reset your scenario
            
        """
        # Resets the consignment scenario
        self.skip_first = True
        self.id = {"switch": -1, "cover_small":-1,"cover_door":-1}

## pick and place learning scenario
    def scenarioTwoArmSync(self,**kwargs):
        """ - called by baxter_behaviors run() automatically
            Executes the task "scenarioTwoArmSync" first to be flexible to new parameters
            and then runs the pick and place scenario in another thread 
        """
        
        self.__scenario = self.createScenarioName(**kwargs)
        if not self.baxter.tasker is None:
            try:
                self.baxter.tasker.perform(self.__scenario)
            except Exception,e:
                rospy.logwarn("%s"%(e))
        self.post.__scenarioTwoArmSync(**kwargs)


    def __scenarioTwoArmSync(self,**kwargs):
        import os
        result = False
        looped = True
        while not rospy.is_shutdown() and self.baxter.hlm.stop() is False :#check if path should be executed in loop
            i = 0            
            while not rospy.is_shutdown() and self.baxter.hlm.stop() is False: #execute all substeps until no file is available
                side = "left"
                if self.baxter.br.stopExecution(side) is True:
                    break
                if not os.path.isfile(self.baxter.br.filename+side+str(i)):
                    rospy.loginfo("File does not exist, breaking loop")
                    break 
                print "number inside loop",i
                self.baxter.mm.changeMenuTitle("TwoArmSync "+str(i)+ " " + side)
                result = self.baxter.bb.executePath(**{'side':side,'number':i,'looped':False, 'threaded':False})
                if not result:
                    rospy.logwarn("Trajectory could not be executed: Stopping")
                    break
                
                
                side = "right"
                if self.baxter.br.stopExecution(side) is True:
                    break
                if not os.path.isfile(self.baxter.br.filename+side+str(i)):
                    rospy.loginfo("File does not exist, breaking loop")
                    break 
                print "number inside loop",i
                self.baxter.mm.changeMenuTitle("TwoArmSync "+str(i)+ " " + side)
                result = self.baxter.bb.executePath(**{'side':side,'number':i,'looped':False, 'threaded':False})
                if not result:
                    rospy.logwarn("Trajectory could not be executed: Stopping")
                    break
                i+=1
            
            if not looped or not result:
                break
        self.baxter.mm.changeMenuTitle("Pick and Place Step "+str(self.pnp_index))
        rospy.loginfo("done with all steps")

## pick and place learning scenario
    def scenarioPickAndPlace(self,**kwargs):
        """
            Executes the task "scenarioPickAndPlace" first to be flexible to new parameters
            and then runs the pick and place scenario in another thread 
        """
        self.__scenario = self.createScenarioName(**kwargs)
        if not self.baxter.tasker is None:
            try:
                self.baxter.tasker.perform(self.__scenario)
            except Exception,e:
                rospy.logwarn("%s"%(e))
        self.__scenarioPickAndPlace(**kwargs)
        
    def __scenarioPickAndPlace(self,**kwargs):
        """
            Loads the pick and place scenario:
            
            In this scenario the user can record multiple trajectories for each arm.
            The current trajectory number to be recorded is shown in the title bar of baxter.
            For every step a trajectory can be recorded. Every step can be relearned and play.
            After a complete recording all steps can be placed consecutively.
             
            All recordings are saved in the data directory of the project and are also available after restart.
        """
        entries ={#maybe add an insert step ..more complicated
                 'Previous Step':self.previousStep,
                 'Next Step':self.nextStep,
                 'Learn Step':self.learnStep,
                 'Run Step':[self.runStep,False],
                 'RunAll':[self.runAllSteps,False],
                 'RunAllLooped':[self.runAllSteps,True], 
                  }
        self.baxter.mm.addGenericMenu("scenarioPnP", "scenario","Pick and Place Step "+str(self.pnp_index),entries)
        self.baxter.mm.loadMenu("scenarioPnP")

    def previousStep(self,**kwargs):
        """
           Goes the the previous recorded trajectory 
        """
        
        self.pnp_index = max(0,self.pnp_index-1)
        self.baxter.mm.changeMenuTitle("Pick and Place Step "+str(self.pnp_index))
        
    def nextStep(self,**kwargs):
        """
            Goes to the next step to record a new trajectory or play back an already recorded one
        """
        self.pnp_index+=1
        self.baxter.mm.changeMenuTitle("Pick and Place Step "+str(self.pnp_index))
    
    def learnStep(self,**kwargs):
        """
            Activates the learning mode for the current step
            
            .. note:: Only if the cuff button is pressed the robot saved the current joint angles and gripper position.
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.baxter.mm.neglect()
            return
        self.baxter.bb.teach(**{'side':side,'number':self.pnp_index, 'parent': 'scenarioPnP'})
    
    def runStep(self,**kwargs):#because a thread can not be started more than once
        """
            Runs the current selected step - non blocking
        """
        self.post.__runStep(**kwargs)
    
    def __runStep(self,**kwargs):
        """
            Runs the current selected step - blocking
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.baxter.mm.neglect()
            return
        self.baxter.bb.executePath(**{'side':side,'number':self.pnp_index})
    
    def runAllSteps(self,**kwargs):
        """
            Runs all recorded steps - non blocking
            
            .. note:: Even if you have not recorded anything it could play back, trajectories that already 
                exist in your datapath.
        """
        self.post.__runAllSteps(**kwargs)
    
    def __runAllSteps(self,**kwargs):
        """
            Runs all recorded steps - blocking
            
            .. note:: Even if you have not recorded anything it could play back, trajectories that already 
                exist in your datapath.
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.baxter.mm.neglect()
            return
        looped = self.baxter.mm.default_values[self.baxter.mm.modes[self.baxter.mm.cur_mode]]
        import os
        self.baxter.br.stopExecution(side,False)
        while not rospy.is_shutdown() and self.baxter.br.stopExecution(side) is False and self.baxter.hlm.stop() is False :#check if path should be executed in loop
            i = 0
            print "start of loop"
            while not rospy.is_shutdown() and self.baxter.br.stopExecution(side) is False and self.baxter.hlm.stop() is False: #execute all substeps until no file is available
                if not os.path.isfile(self.baxter.br.filename+side+str(i)):
                    rospy.loginfo("File does not exist, breaking loop")
                    break 
                print "number inside loop",i
                self.baxter.mm.changeMenuTitle("Pick and Place Step "+str(i))
                result = self.baxter.bb.executePath(**{'side':side,'number':i,'looped':False, 'threaded':False})
                if not result:
                    rospy.logwarn("Trajectory could not be executed: Stopping")
                    break
                i+=1
            if not looped or not result:
                break
        self.baxter.mm.changeMenuTitle("Pick and Place Step "+str(self.pnp_index))
        rospy.loginfo("done with all steps")

##############

    def btnTray(self):
        """
            Button callback function to test the tray element
        """
        self.baxter.hlm.goToInitPose()
        self.manipulateItem(self.side,"item_tray",0,"side",True)
#         self.manipulateItem(self.side,"item_tray",0,"front",False)
        
    def btnGobletBlue(self):
        """
            Button callback function to test the blue goblet element
        """
        self.baxter.hlm.goToInitPose()
        self.manipulateItem(self.side,"goblet_blue",0,"side",True)
#         self.manipulateItem(self.side,"goblet_blue",0,"front",False)
        
    def btnGobletRed(self):
        """
            Button callback function to test the red goblet element
        """
        self.baxter.hlm.goToInitPose()
        self.manipulateItem(self.side,"goblet_red",0,"side",True)
#         self.manipulateItem(self.side,"goblet_red",0,"front",False)
    
#     def scenarioConsignment2(self,**kwargs):
#         """
#             Runs the consignment2 scenario - non blocking
#             
#             This scenario uses two long electric grippers to sort 3 simple 
#             plastic elements from a storage onto a conveyor belt with the left hand. 
#             Simultaneously it takes the 3 objects from the conveyor belt and puts it in a 
#             storage on the right side of the robot 
#         """
#         self.__scenario = self.createScenarioName(**kwargs)
#         
#         self.baxter.scene.makeTables(self.__scenario)
#         if not self.baxter.tasker is None:
#             try:
#                 self.baxter.tasker.perform(self.__scenario)
#             except Exception,e:
#                 rospy.logwarn("%s"%(e))
#         self.post.__scenarioConsignment2(**kwargs)
        
    def __scenarioConsignment2(self,**kwargs):
        """
            Runs the consignment2 scenario - blocking
        """
        self.baxter.display.setImage(self.baxter.datapath + "logo1024.jpg")
        # these are the used items
        boxes = ["item_tray","goblet_blue","goblet_red"] # TODO get from scene
        ids = []
        for i in boxes:
            ids.append(0)
        error = True

        try:
            rospy.loginfo("Going to init pose")
            self.baxter.hlm.goToInitPose()
            last_time=None
            left_thread = None
            right_thread = None 
            while not rospy.is_shutdown() and self.baxter.hlm.stop() is False:
                 # Print timing
                rospy.loginfo("Getting time for cycle")
                if not last_time is None:
                    delay=rospy.get_time()-last_time
                    rospy.loginfo("Cycle time "+str(delay)+" s")
                last_time=rospy.get_time()
                 # TODO return values should be checked
                
                if self.baxter.bb.use_arm["left"]:
                    #L pick goblet1 post back
                    left_thread = self.post.manipulateItem("left",boxes[1],ids[1],"side",True)
                #R wait
                if self.baxter.bb.use_arm["right"]:
                    if not right_thread is None:
                        right_thread.join()
                if self.baxter.bb.use_arm["right"]:
                    #R pick goblet1 post front
                    right_thread = self.post.manipulateItem("right",boxes[1],ids[1],"front",True)
                #L wait
                if self.baxter.bb.use_arm["left"]:
                    left_thread.join()
                    #L drop goblet1 post front
                    left_thread = self.post.manipulateItem("left",boxes[1],ids[1],"front",False)
                if self.baxter.bb.use_arm["right"]:
                    #R wait 
                    right_thread.join()
                    #R drop goblet 1 post back
                    right_thread = self.post.manipulateItem("right",boxes[1],ids[1],"side",False)
                if self.baxter.bb.use_arm["left"]:
                    #L wait
                    left_thread.join()

                    #L pick goblet 2 post back
                    left_thread = self.post.manipulateItem("left",boxes[2],ids[2],"side",True)
                if self.baxter.bb.use_arm["right"]:
                    #R wait
                    right_thread.join()
                    #R pick goblet 2 post front
                    right_thread = self.post.manipulateItem("right",boxes[2],ids[2],"front",True)
                if self.baxter.bb.use_arm["left"]:
                    #L wait
                    left_thread.join()
                    #L drop goblet 2 post front
                    left_thread = self.post.manipulateItem("left",boxes[2],ids[2],"front",False)
                if self.baxter.bb.use_arm["right"]:
                    #R wait
                    right_thread.join()
                    #R drop goblet 2 post back
                    right_thread = self.post.manipulateItem("right",boxes[2],ids[2],"side",False)
                if self.baxter.bb.use_arm["left"]:
                    #L wait
                    left_thread.join()
                    
                    #L pick tray 1 post back
                    left_thread = self.post.manipulateItem("left",boxes[0],ids[0],"side",True)
                if self.baxter.bb.use_arm["right"]:
                    #R wait
                    right_thread.join()
                    #R pick tray 2 front
                    right_thread = self.manipulateItem("right",boxes[0],ids[0],"front",True)
                if self.baxter.bb.use_arm["left"]:
                    #L wait
                    left_thread.join()
                    #L drop tray 1 post front
                    left_thread = self.post.manipulateItem("left",boxes[0],ids[0],"front",False)
                if self.baxter.bb.use_arm["right"]:
                    #R drop tray 2 post back
                    right_thread = self.post.manipulateItem("right",boxes[0],ids[0],"side",False)
                if self.baxter.bb.use_arm["left"]:
                    #L wait
                    left_thread.join()
                #items completed
                ids[0]+=1
                ids[2]+=1
                ids[1]+=1
                
                if ids[0]==12:   
                    break # stops after 12 elements of all items are manipulated
                if self.baxter.hlm.stop() is True:
                    break
                
        except Exception,e:
            rospy.logerr("task could not be completed. %s"%(e))
        
        self.baxter.mm.loadMenu("main")
        
    # for consignment2 scenario
    def manipulateItem(self,side,box,id,location,pick=True):
        """
            Manipulate Item is a more advanced manipulation which consists of:
            
            * approach to the object inside a box
            * grip/release the object inside the box
            * withdraw from the box
            
            :param side: Arm to be used
            :type side: str
            :param box: The box where the object should be picked/placed
            :type box: str
            :param id: The object number to be manipulated
            :type id: int
            :param location: if the object is in "front" of the robot or at the sides. $
                If location == "front" the positions F and P are used instead of L and H (see PlanGenerator)
            :type location: str
            :param pick: If the item to be picked or placed
            :type pick: bool
            :return: True if movement was successful
            :rtype: bool
        """
        rospy.loginfo("Trying to manipulate object %s from box %s with %s arm at %s, pick %d"%(id,box,side,location,pick))
        try:
            if self.baxter.hlm.stop() is True:
                print "stop called"
                return False
            if pick:
                self.baxter.gripper[side].outside_grip = self.baxter.scene.boxes[box][2][0]
                self.baxter.gripper[side].release() 
            num_items = self.baxter.scene.numItems(box)
            #if you want to sort the object with the right arm the other way around
#             if side == "right":
#                 id= num_items-1 - id%num_items
#             else:
            id= id%num_items
            
            if location == "front":
                high = "P"
                low = "F"
            else:
                high = "H"
                low = "L"
            if box == "goblet_blue" and pick is True:
                self.baxter.gripper[side].set_moving_force(5.0)
                self.baxter.gripper[side].set_dead_band(15)
                self.baxter.gripper[side].set_velocity(25)
                
            if box == "item_tray" and side == "left":
                self.baxter.gripper[side].set_moving_force(10.0)
                self.baxter.gripper[side].set_dead_band(10)
                self.baxter.gripper[side].set_velocity(35)
        
            
            rospy.loginfo("%s #Approach to box %s,id %d,high %s, outside grip: %d"%(side,box,id,high,self.baxter.scene.boxes[box][2][0]))
            if not self.baxter.hlm.goToObject(box,id,side,high,pick):
                return False
            
            rospy.loginfo("%s #Approach to object %s,id %d,low %s"%(side,box,id,low))
            if not self.baxter.hlm.goToObject(box,id,side,low,pick):
                return False
    
            rospy.loginfo("%s #Withdraw from object %s,id %d,high %s"%(side,box,id,high))
            if not self.baxter.hlm.goToObject(box,id,side,high,pick):
                return False
            
#             rospy.loginfo("%s #Moving to init from p %s,id %d,high %s"%(side,box,id,high))
#             if pick is False:
#                 if not self.baxter.hlm.goToInitPose(side):
#                     return False
    
            if box == "goblet_blue" or box == "item_tray":
                self.baxter.gripper[side].set_moving_force(40)
                self.baxter.gripper[side].set_dead_band(5)
                self.baxter.gripper[side].set_velocity(50)
            
            return True
        except Exception,e:
            rospy.logerr("Could not manipulate object %s"%(e))
            return False
    
#     def scenarioConsignment(self,**kwargs):
#         """
#             Loads the consignment scenario - non blocking
#             
#             The consignment scenario is a speed and feature optimized scenario for special parts
#             that could be used in the car industry.
#             
#             For this scenario a short electric gripper and the pneumatic gripper is beeing used.
#             All different gripping strategies are shown:
#             
#             * Inside electric grip
#             * Outside electric grip
#             * Grip with pneumatic gripper
#             * Passing objects between two objects
#         """
#         self.baxter.yes()
#         self.__scenario = self.createScenarioName(**kwargs)
#         if not self.baxter.tasker is None:
#             try:
#                 self.baxter.tasker.perform(self.__scenario)
#             except Exception,e:
#                 rospy.logwarn("%s"%(e))
#         self.post.__scenarioConsignment(**kwargs)
       
       
    def showBox(self):
        """
            Shows a sample box with it current state on the screen of the robot,
            which was used for the scenario
        """
        img="boxrenderer.png"
        self.renderer.save(img)
        self.baxter.display.setImage(img)
     
    def __scenarioConsignment(self,**kwargs):
        """
            Starts the consignment scenario - blocking
        """
        self.renderer.fill()
        self.showBox()
        self.item_dropped = False
        self.baxter.enable()
        error = True
        try:
            side = "right"
            otherSide = self.baxter.otherSide(side)
            success = {}
            
            result_pick = False
            result_drop = False
            result_handover = False
            skip_drop = True
            
            last_time=None
            move_params=[3,0.03,0.03,0.03,10]
            r_hand_over_0=baxter_helper_abstract_limb.loadAngles("right_hand_over_0")
            l_hand_over_0=baxter_helper_abstract_limb.loadAngles("left_hand_over_0")
            self.baxter.arm["right"].simple.goToAngles(r_hand_over_0,*move_params)
            self.baxter.hlm.state_pos["right"] = "O"     
            self.baxter.arm["left"].simple.goToAngles(l_hand_over_0,*move_params)
            self.baxter.hlm.state_pos["left"] = "O"
            # MAIN consignment LOOP
            while not rospy.is_shutdown() and self.baxter.hlm.stop() is False:
            
                # Print timing
                if not last_time is None:
                    delay=rospy.get_time()-last_time
                    rospy.loginfo("Cycle time "+str(delay)+" s")
                last_time=rospy.get_time()
            
                #Drop off the switch with the left vacuum gripper inside the wako
                th_drop_switch = self.post.dropOffItem(otherSide)
                #Pick a cover with the right arm and place it in the wako
                th_cover_small = self.post.doCoverSmall()
                #Wait until the drop off of left arm is ready for next operation
                th_drop_switch.join() 
                #Save the result
                success["drop_switch"] = th_drop_switch.result
                #Take a cover with the left arm and place it inside the wako
                th_cover_door = self.post.doCoverDoor()
                rospy.sleep(0.1)
                # finish after one of each item. special mode
#                 if self.item_dropped is True:
#                     th_cover_door.join()
#                     self.baxter.yes()
#                     break
  
                #Wait until the right arm  finished
                th_cover_small.join()
                success["cover_small"] = th_cover_small.result
                
                
                if self.id["switch"] >= self.object_to_consign and self.object_to_consign != -1: 
                    th_cover_door.join()
                    break
                
                #Pick next switch item with the right arm
                result_pick = self.preparePickNextItem()      
                
                #Wait until the left arm finished
                th_cover_door.join()
                success["cover_door"] = th_cover_door.result
                if th_cover_door.result is False:
                    self.baxter.hlm.goToInitPose("left")
                    
                #Dont skip items in the next round - for initialization
                self.skip_first = False
                
                # If pick is successfull then perform hand-over, otherwise move on to next item
                if result_pick is True:
                    result_handover = self.handOver()
                else:
                    result_handover = False
                success["result_handover"] = result_handover
                
                    
                self.printResults(success)
            rospy.loginfo("Scenario finished")
        except Exception,e:
            rospy.logerr("task could not be completed. error: %s"%(str(e)))
        
        pic = str(self.baxter.datapath + "logo1024.jpg")
        self.baxter.display.setImage(pic)
    
    def printResults(self,success):
        """
            As the consignment scenario consists of several subtasks, the results of each iteration
            that had some troubles are printed.
            
            :param success: contains the result of all subtasks
            :type success: list(bool)
            :return: True if all subtasks are successfully executed
            :rtype: bool
        """
         # Check and report subtask results
        if success!={}:
            if all([subtask==True for subtask in success.values()]):                    
                success = {}
                return True                    
            else:
                rospy.logwarn("Result of all subtasks %s"%(str(success)))
                return False
    
    def btnPickItem(self):
        """
            Button callback function to test the switch element
        """
        self.baxter.hlm.goToInitPose()
        self.id["switch"] += 1
        rospy.loginfo("id %d"%self.id["switch"])
        self.pickNextItem(self.id["switch"])
        
    def nextItemId(self,box):
        """
            Returns the next valid item id to be manipulated
            :param box: The box you want to have the next ID for
            :type box: int
            :return: The next ID for the given box
            :rtype: int
        """
        if box == "switch" and (self.id[box] == 8 or self.id[box] == 10): # Box has no items at this position
            self.id[box]+=2 
        else:
            self.id[box]+=1  
        num_items = self.baxter.scene.numItems(box)
        if box == "switch" and self.id[box]>=num_items:
            self.renderer.fill()
            self.showBox()
            
        self.id[box]= self.id[box]%num_items
        return self.id[box]
        
    def preparePickNextItem(self):
        """
            Manager for picking the next switch item. This function includes a recovery option
            and a graphical feedback on the screen.  
            
            :return: True if pick was successful
            :rtype: bool
        """
        box = "switch"
        side = "right"
        result_pick = False
        while not rospy.is_shutdown() and self.baxter.hlm.stop() is False:               
            self.nextItemId(box)
            self.renderer.setMark(self.id[box])
            self.showBox()
            result_pick = self.pickNextItem()
            if result_pick is  False:
                move_params=[3,0.02,0.02,0.03,10]
                self.baxter.hlm.goToAngles(side,side+"_hand_over_0",move_params)
                result_pick == -1
            if result_pick == -1:                        
                rospy.loginfo("Releasing unpicked object")
                self.baxter.gripper[side].release(True)
                self.renderer.unsetMark()
                if RENDER_ERRORS:
                    self.renderer.setNotPicked(self.id[box])
                self.showBox()
                
                rospy.sleep(0.1)
                
                if self.recovery == "Skip" or self.recovery == "Retry":                        
                    continue
                elif self.recovery == "Stop":
                    result_pick = False
                    break
                elif self.recovery == "Ignore":
                    result_pick = True
                    break
                else:
                    break
            else:
                self.renderer.removeItem(self.id[box])
                self.renderer.unsetMark()
                self.showBox()                        
                break                
            rospy.sleep(0.02)
        return result_pick
    
    def pickNextItem(self):
        """
            Picks the next switch item
            
            :return: True if pick was successful
            :rtype: bool
        """       
        # Move on to next item
        
        side="right"
        box = "switch"
        self.baxter.gripper[side].outside_grip = 1
        self.baxter.gripper[side].release(False)
        id = self.id[box]
        rospy.loginfo("%s#Approach to box %d"%(side,id))
        if not self.baxter.hlm.goToObject(box,id,side,"H",True):
            return False
        
        rospy.loginfo("%s#Approach to object %d"%(side,id))
        if not self.baxter.hlm.goToObject(box,id,side,"L",True):
            return False

        rospy.loginfo("%s#Withdraw from box %d"%(side,id))
        if not self.baxter.hlm.goToObject(box,id,side,"H",True):
            rospy.logwarn("%s #Withdraw with item failed. Skipping item %d"%(side,id))
            return False
        
        rospy.loginfo("%s#Move to hand over %d"%(side,id))
        move_params=[5,0.01,0.01,8]
        if not self.baxter.hlm.executePlan(box,side,id,"O",move_params):
            return False    
        
        if not self.baxter.gripper[side].gripped():
            rospy.logerr("%s #Could not detect a gripped object. Checking recovery option"%(side))
            return -1
        else:
            rospy.loginfo("%s #Detected a gripped object"%(side))
        return True

    def gripHandOverItem(self):
        """
            Performs the hand over operation. For this part fixed angles for both arm are saved.
            The left arm approches the right one until a grip is detected.
            
            The baxter monitor can be used to change the fixed hand over positions. 
            
            :return: True if hand over was successful
            :rtype: bool
        """
        
        box = "switch"
        self.baxter.gripper["right"].outside_grip = 1
        r_hand_over_0=baxter_helper_abstract_limb.loadAngles("right_hand_over_0")
        r_hand_over_1=baxter_helper_abstract_limb.loadAngles("right_hand_over_1")
        l_hand_over_0=baxter_helper_abstract_limb.loadAngles("left_hand_over_0")
        l_hand_over_1=baxter_helper_abstract_limb.loadAngles("left_hand_over_1")

        
        rospy.loginfo("Holding r_hand_over")
        self.baxter.arm["right"].simple.post.goToAngles(r_hand_over_0,*[0.5,0.00001,0.00001],timeout=6)
        self.baxter.arm["right"].simple.waitForMoving()
        self.baxter.collision.deactivate(20)
        rospy.sleep(0.3)
        rospy.loginfo("left #Activate grip")
        self.baxter.gripper["left"].grip(False)
        rospy.loginfo("left #Approach with left")
        self.baxter.arm["left"].simple.post.goToAngles(l_hand_over_1,*[0.10,0.00001,0.00001],timeout=6)
        self.baxter.arm["left"].simple.waitForMoving()
        
        rospy.loginfo("left #Check for grip")
        gripped = False
        while self.baxter.arm["left"].simple.moving() and not rospy.is_shutdown():
            if self.baxter.gripper["left"].gripped():
                print "right #Release grip"
                self.baxter.scene.pickUp("left",box,self.id[box],"side")
                self.baxter.gripper["right"].release()
                gripped = True
                self.baxter.arm["left"].simple.cancel()
                self.baxter.arm["right"].simple.cancel()
                break
            else:
                rospy.sleep(0.01)
        self.baxter.arm["left"].simple.waitForStopped()
        self.baxter.arm["right"].simple.waitForStopped()
        self.baxter.collision.activate()

        if not gripped:
            rospy.logerr("left #NOT GRIPPED")
            self.baxter.gripper["left"].release(False)
            move_params=[1,0.01,0.01,0.03,10]
            if not self.baxter.arm["left"].simple.goToAngles(l_hand_over_0,*move_params):
                return False            
            return -1
        else:
            rospy.loginfo("left #Successfully gripped")
            move_params=[2,0.025,0.025,0.025,5]
            self.baxter.arm["right"].simple.goToAngles(r_hand_over_1,*move_params)
            self.baxter.arm["left"].simple.goToAngles(l_hand_over_0,*move_params)

        return True
    
    def handOver(self):
        """
            Manager for the hand over procedure. This manager handles the recovery option
            and the graphical feedback.
            
            :return: True if hand over was successful
            :rtype: bool
        """
        box = "switch"
        retry = True
        self.baxter.scene.release("right")
        rospy.loginfo("left #Move to hand over")
        while not rospy.is_shutdown() and self.baxter.hlm.stop() is False:
            result_handover = self.gripHandOverItem() 
            if result_handover == -1:
                if (self.recovery == "Skip" or self.recovery == "Retry") and retry is True:
                    retry = False
                    continue
                elif self.recovery == "Stop":
                    return False
                elif self.recovery == "Ignore":
                    return True
                else:
                    if RENDER_ERRORS:
                        self.renderer.setNotHanded(self.id[box])
                    return False
            else:
                return result_handover
            rospy.sleep(0.02)
        
              
    def btnCoverSmall(self):
        """
            Button callback function to test the small cover element
        """
        self.skip_first = False
        self.doCoverSmall()
          
    def doCoverSmall(self):
        """
            Approches the box that contains the small covers with the eletric gripper,
            grips an object - inside out, withdraws from the source box and
            places the item in the destination box.
            
            :return: True if movement was successful
            :rtype: bool
        """
        if self.skip_first is True:
            return True
        self.item_dropped = True
        box = "cover_small"
        self.nextItemId(box)
        side = "right"
    
        rospy.sleep(1.5)# to ensure arms are not colliding with items
        r_hand_over_0=baxter_helper_abstract_limb.loadAngles("right_hand_over_0")
        move_params=[2,0.025,0.025,0.025,5]
        self.baxter.arm["right"].simple.goToAngles(r_hand_over_0,*move_params)     
        
        if not self.manipulateItem(side, box, self.id[box], "side", True):
            return False
        if not self.manipulateItem(side, box, self.id[box], "front", False):
            return False
        move_params=[4,0.03,0.03,10]
        rospy.loginfo("%s#Going to hand over position with right hand"%side)
        if not self.baxter.hlm.executePlan(box,side,self.id[box],"O",move_params):
            return False 
        return True

    def btnCoverDoor(self):
        """
            Button callback function to test the door cover element
        """
        self.skip_first = False
        self.doCoverDoor()
        
    def doCoverDoor(self):
        """
            Approches the box that contains the door covers with the pneumatic gripper,
            grips an object, withdraws from the source box and
            places the item in the destination box.
            
            :return: True if movement was successful
            :rtype: bool
        """
        if self.skip_first is True:
            return True
        box = "cover_door"
        self.nextItemId(box)
        side = "left"
        
        if not self.manipulateItem(side, box, self.id[box], "side", True):
            return False
        if not self.manipulateItem(side, box, self.id[box], "front", False):
            return False
        
        move_params=[4,0.02,0.02,10]
        rospy.loginfo("%s#Going to hand over position with left hand"%side)
        if not self.baxter.hlm.executePlan(box,side,self.id[box],"O",move_params):
            return False        
        return True

    def btnDropOff(self):
        """
            Button callback function to test the placement of the switch element
        """
        self.baxter.hlm.goToInitPose()
        self.dropOffItem(False, "left")
    
    def dropOffItem(self,side):
        """
            After a successful hand over procedure, this function places the switch 
            in the destination box.
            
            :return: True if movement was successful
            :rtype: bool
        """
        if self.skip_first is True:
            rospy.loginfo("Skipped dropoff")
            return True
        
        box = "switch"
        id = -1
        
        
        if not self.manipulateItem(side, box, id,"front",False):
            return False              
        rospy.loginfo("%s#Moving to I with left hand"%side)
        move_params=[4,0.02,0.02,10]
        if not self.baxter.hlm.executePlan(box,side,id,"I",move_params):
            return False  
        return True
        
