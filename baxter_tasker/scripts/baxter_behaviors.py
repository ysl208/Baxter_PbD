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


"""
    Provides baxter specific behaviors to handle GUI callbacks. 
    All implemented function here are callable from the menu. 
    Otherwise you have to provide your class to the menu manager.
"""

import rospy
import time,operator,sys
from threading import Lock
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    )
from copy import deepcopy


from hr_helper.post_threading import Post
from hr_helper.tf_helper import *


from std_msgs.msg import Int8


RENDER_ERRORS=True

from baxter_learner import BaxterLearner
from baxter_scenarios import BaxterScenarios
from baxter_locator import BaxterLocator
from inspect import getmembers
from functools import partial

class BaxterBehaviors():
    """
        The baxter behaviors implement useful stuff for the current interface
     
    """
    def __init__(self,baxter):
        self.post = Post(self)
        self.appendToTask = None
        self.baxter = baxter
        if baxter.mm is None:
            baxter.loadMenuManager()
        self.mm = baxter.mm
        self.mm.addBaxterBehavior(self)
        self.learning=True  
        self.coords = None
        self.predefined_box = None       
        if self.baxter.moveit is True:
            from generate_plans import PlanGenerator
            self.generator = PlanGenerator(baxter)
#         self.side = "left"
        self.use_arm = {"left":True,"right":True}
        self.bs = BaxterScenarios(baxter)
        self.bl = BaxterLearner(baxter)
        self.locator = BaxterLocator(baxter)
        self.actionSequence = []
        self.allActions = {}
        self.target_locations = {'A':(0.6,-0.419),'M':(0.795,-0.2),'D':(0.6, -0.018)}
        self.exp_predicates = {'1. Precondition' : 'object jaune: position de depart',
                               '2. Effet': 'object jaune: position d\'arrivee'}
        self.exp_position_occupied = False
        self.colour = 'red'
      
    def changeExecutionState(self,stop=True):
        """
            Sets and unsets the flag to stop all movements   
        """
        rospy.loginfo("STOP executing behavior set to %d"%stop)
        self.baxter.hlm.stop(stop) 
        self.baxter.br.stopExecution("left",stop)
        self.baxter.br.stopExecution("right",stop)
      
    def addMainWindow(self,appendToTask):
        """
            Link to main window to write text into the editor window 
        """
        self.appendToTask = appendToTask  

        
    def addFramePoint(self,**kwargs):
        """
            Saves the point on the finger tip of the gripper
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s",str(e))
            self.mm.neglect()
            return
        step = int(self.mm.modes[self.mm.cur_mode][6:7])
        rospy.loginfo("step %d"%step)
        point = self.baxter.frame.addPoint(side,step) 
        self.mm.confirm()
        #self.mm.loadMenu(self.mm.cur_page)

    def nothing(self, **kwargs):
        """
            Does nothing. Useful if you just create menu entries with no functionality but text information
        """
        pass
                      
    def showCamera(self,**kwargs):
        """
            Opens a hand camera stream and sends it to the display
            
           :param side: camera to be opened 
           :type side: str
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return
        self.hideGUI(**{'update':False})
        self.baxter.camera.startCamera(side+"_hand_camera")
        self.locator.publish_camera = True
                
    def solveTetris(self,**kwargs):
        """
            Moves the blocks to solve the current alignment of the tetris blocks
        """
        #self.showCamera()
        self.locator.recognise_grid()

             
    def run(self,**kwargs):
        """
            Creates a menu of all functions in baxter_scenarios.py that start with the name "scenario"
        """
        members = getmembers(self.bs)
        targets = "scenario"
        scenarios=[m[0] for m in members if m[0].startswith(targets)]
        entries={}
        for scenario in scenarios:
            entries[scenario[len(targets):]] = getattr(self.bs,scenario) # save scenario names in entries
        self.mm.addGenericMenu(targets,self.mm.cur_page,"Select your desired scenario", entries)
        self.mm.loadMenu(targets)

    def permutation(self, **kwargs):
        """
            Used for experiment to permutate the positions of two blocks
        """
        self.locator.recognise_grid()
        red = self.locator.detect_colour(0, 'red')
        rospy.loginfo("permutation(): looking for red object: %s" % str(red))
        blue = self.locator.detect_colour(0, 'blue')
        rospy.loginfo("permutation(): looking for blue object: %s" % str(blue))
        if red[0] < blue[0]:
            sequence = [('red','M'),('blue','A'),('red','D')]
        else:
            sequence = [('red','M'),('blue','D'),('red','A')]

        colours = self.locator.tetris_blocks.keys() 
        self.target_locations['D']
        self.target_locations['A']
        self.target_locations['M']
        answer = 'n'
        action_number = 0
        while action_number < len(sequence):

            (colour,pos) = sequence[action_number]
        
            rospy.loginfo('permutation(): %s to position %s' % (colour,pos))

            self.locator.update_pose()
            goal_pose = self.locator.pose[:]
            goal_pose[0:2] = self.target_locations[pos][0:2]
            success = self.locator.locate(colour, goal_pose)
            if not success:
                answer = raw_input('Failed to execute action. Try again? (y/n): ')
                if answer in ('y'):
                    action_number -= 1
                    continue
                
            action_number += 1 


    def exp_learnedPredicates(self, **kwargs):
        """
            Used for experiment to show 'what preconditions and effects Baxter understood'
        """

        entries={}
        for condition in self.exp_predicates.keys():
            text = '%s: %s' % (condition, self.exp_predicates[condition])
            entries[text] = self.readCommandLine

        title = "Ce qu'a appris Baxter pour effectuer le deplacement" 
        self.mm.addGenericMenu("expMenu", self.mm.cur_page, title, entries)
        self.mm.loadMenu("expMenu")

    def readCommandLine(self, **kwargs):
        """
            Used for experiment to show 'what preconditions and effects Baxter understood'
        """
        try:
            condition = kwargs["fname"].split(':')[0]
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return

        # experimentation parameters
        answer = raw_input('Change %s: %s to: ' % (condition,self.exp_predicates[condition]))
        self.exp_predicates[condition] = answer
        self.mm.loadMenu("teachMenu")


    def enterPredicates(self, **kwargs):
        """
            Creates a menu of all functions in baxter_learner.py that are in __predicates

            .. note:: The parameters below are not the method parameters but the entries in the kwargs
            
            :param precondition: selecting predicates for preconditions? If False, select for Effects
            :type precondition: bool

        """
        try:
            precond = kwargs['precondition']
        except:
            precond = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        self.bl.isPrecondition(precond)
        
        members = self.bl.getCurrentPredicate().keys() 
        entries={}

        for member in members:
            entries[member] = getattr(self.bl, 'predicateSelection') # save predicate names in entries
        entries['view selection'] = [getattr(self.bl, 'displayText'), str(self.bl.getAllPredicates())]
        entries['reset selection'] = getattr(self.bl, 'predicateReset')
        title = "Select %s for the new operator" % ('preconditions' if precond else 'effects')
        self.mm.addGenericMenu("predicate", self.mm.cur_page, title, entries)
        self.mm.loadMenu("predicate")

    def enterParameters(self,**kwargs):
        """
            Creates a menu of all functions in baxter_learner.py that are in __params
        """

        members = self.bl.getAllParameters().keys() 
        entries={}

        for param in members:
            entries[param] = getattr(self.bl, 'paramSelection') # save param names in entries
        entries['view selection'] = [getattr(self.bl, 'displayText'), str(self.bl.getAllParameters())]
        entries['reset selection'] = getattr(self.bl, 'paramReset')
        self.mm.addGenericMenu("param",self.mm.cur_page,"Select your desired params for this operation", entries)
        self.mm.loadMenu("param")

    def demonstrate(self,**kwargs):
        """
            Creates a menu of all atomic actions in baxter_learner.py that are in __atomic_actions
        """

        members = self.bl.getAllSavedActions()
        entries={}

        for param in members:
            entries[str(param)] = self.executeAction # save param names in entries

#        entries['search colour for position'] = self.search_menu
        entries['move block to position'] = self.move_block_menu
        entries['move arm to position'] = self.move_menu
        self.mm.addGenericMenu("actionMenu",self.mm.cur_page,"Select the action to demonstrate", entries)
        self.mm.loadMenu("actionMenu")

    def move_menu(self,**kwargs):
        entries={}

        for target in self.target_locations.keys():
            self.locator.update_pose()
            x_offset = self.target_locations[target][0]-self.locator.pose[0]
            y_offset = self.target_locations[target][1]-self.locator.pose[1]
            offset_pose = (x_offset,y_offset,0,0,0,0)
            entries['move to ' + str(target)] = [self.moveBy, offset_pose]
            if target == 'D':
                entries['move to empty'] = [self.moveBy, offset_pose]
        self.mm.addGenericMenu("moveMenu",self.mm.cur_page,"Select the target limb position", entries)
        self.mm.loadMenu("moveMenu")


    def search_menu(self,**kwargs):
        entries={}
        colours = self.locator.tetris_blocks.keys() 

        for block in colours:
            entries['search ' + str(block)] = [self.get_colour_position,False]
        self.mm.addGenericMenu("searchMenu",self.mm.cur_page,"Select the target position for block", entries)
        self.mm.loadMenu("searchMenu")


    def move_block_menu(self,**kwargs):
        entries={}
        
        for target in self.target_locations.keys():
            entries['move to ' + str(target)] = [self.moveBlock, self.target_locations[target]]
        self.mm.addGenericMenu("moveBlockMenu",self.mm.cur_page,"Select the target position", entries)
        self.mm.loadMenu("moveBlockMenu")

    def addEmptyCondition(self,**kwargs):
        """
            Used for Experimentation:
            adds condition that goal position needs to be empty
        """
        if self.exp_position_occupied:
            self.exp_position_occupied = False
        else:
            self.exp_position_occupied = True

        rospy.loginfo('addEmptyCondition(): new emptycondition = %s' % str(self.exp_position_occupied))

    def executeAction(self,**kwargs):
        """
            Creates a menu of learn or execute action
        """
        try:
            action = kwargs["fname"]
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return

        entries = {}
        pose_offset = 'empty'
        if action in self.bl.getAllSavedActions():
            pose_offset = self.bl.baxter_actions[str(action)]['joint_position']
            entries['Show action only'] = [self.moveBy, pose_offset]
            entries['Show pick up action'] = [self.pickUpActionColour, pose_offset]
#            entries['Add condition'] = self.addEmptyCondition
#        entries['Rename '+str(action)] = [self.renameAction, action]
        entries['Learn '+str(action)] = getattr(self.bl, 'demoAction')

        self.mm.addGenericMenu("learnMenu", self.mm.cur_page,"Action saved as: %s" % (str(pose_offset)),entries)
        self.mm.loadMenu("learnMenu")

    
    def renameAction(self,**kwargs):
        """
            Renames the selected action with the one entered in the command line
        """
        try:
            old_action = kwargs["fname"].split(' ')[1]
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return
        rospy.loginfo("Enter the new name of the action:")
        action = sys.stdin.readline().strip()

        self.bl.baxter_actions[str(action)] = self.bl.baxter_actions[str(old_action)]
        del self.bl.baxter_actions[str(old_action)]

        self.baxter.mm.changeMenuTitle("Action %s renamed to: %s" % (old_action, str(action)))
        self.baxter.yes() 
        self.mm.loadMenu("teachMenu")

    def moveBlock(self, **kwargs):
        """
            Moves to the target location
        """
        try:
            action = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current action selection")

        position = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        colours = self.locator.tetris_blocks.keys() 
        entries = {}

        for block in colours:
            entries[str(block)] = [self.moveTo, position]

        self.mm.addGenericMenu("colourMenu",self.mm.cur_page,"Select the block colour for %s" %action, entries)
        self.mm.loadMenu("colourMenu")

    def pickUpAction(self, **kwargs):
        """
            Moves object by the offset pose
        """
        pose_offset = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        colour = kwargs["fname"]
#        pdb.set_trace()
        self.locator.update_pose() #get current pose of arm
#        x_offset = self.locator.pose[0] + pose_offset[0]
#        y_offset = self.locator.pose[1] + pose_offset[1]
#        goal_pose = (x_offset,y_offset,0,0,0,0)

        if self.exp_position_occupied:
            self.colour = 'blue'
            self.baxter.no()
        else:
            success = self.locator.locate(colour, pose_offset, 1)
            self.mm.loadMenu("actionMenu")

    def pickUpActionColour(self, **kwargs):
        """
            Moves arm by the offset pose
        """
        pose_offset = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]

        try:
            action = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current action selection")

#        position = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        colours = self.locator.tetris_blocks.keys() 
        entries = {}

        for block in colours:
            entries[str(block)] = [self.pickUpAction, pose_offset]
        entries['any'] = [self.pickUpActionAny, pose_offset]
        self.mm.addGenericMenu("colourMenu",self.mm.cur_page,"Select the block colour for %s" %action, entries)
        self.mm.loadMenu("colourMenu")

    def pickUpActionAny(self, **kwargs):
        """
            Moves object of any colour (red or blue) that is closer to the D position by the offset pose
        """
        pose_offset = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        colour = kwargs["fname"]
        self.locator.recognise_grid()
        red = self.locator.detect_colour(0, 'red')
        rospy.loginfo("permutation(): looking for red object: %s" % str(red))
        blue = self.locator.detect_colour(0, 'blue')
        rospy.loginfo("permutation(): looking for blue object: %s" % str(blue))
        if red[0] < blue[0]:
            colour = 'blue'
        else:
            colour = 'red'

        self.locator.update_pose() #get current pose of arm

        success = self.locator.locate(colour, pose_offset, 1)
        self.mm.loadMenu("actionMenu")


    def moveBy(self, **kwargs):
        """
            Moves arm by the offset pose
        """

        try:
            shake = kwargs["fname"].split(' ')
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return

        pose_offset = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        rospy.loginfo('moveBy(): pose_offset = %s' % str(pose_offset))

        if 'empty' in shake:
            self.locator.recognise_grid()
            x_offset = self.target_locations['D'][0]-self.locator.pose[0]
            y_offset = self.target_locations['D'][1]-self.locator.pose[1]
            pose_offset = (x_offset,y_offset,-0.05,0,0,0)
            self.locator.moveBy(offset_pose=pose_offset)
            self.baxter.no()
        else:
            self.locator.moveBy(offset_pose=pose_offset)
        self.mm.loadMenu("actionMenu")

    def moveTo(self, **kwargs):
        """
            Moves to the target location
        """

        try:
            colour = kwargs["fname"]
            position = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return

        rospy.loginfo('moveTo(): new position = %s' % str(position))
        self.locator.update_pose()
        goal_pose = self.locator.pose[:]
        if self.exp_position_occupied:
            self.baxter.no()
            self.baxter.mm.changeMenuTitle("Goal position not empty")
        else:
            goal_pose[0:2] = position[0:2]
            success = self.locator.locate(colour, goal_pose)
        #update target position of the colour
            if success:
                self.target_locations[colour] = goal_pose[0:2]
        self.mm.loadMenu("actionMenu")

    def savePredicates(self,**kwargs):
        """
            Creates a menu of all actions in baxter_learner.py that are in __atomic_actions to save
        """

        members = self.bl.getAllSavedActions()
        entries={}

        for param in members:
            entries[str(param)] = getattr(self.bl, 'savePredicatesToAction') # save param names in entries
        entries['view all saved'] = [getattr(self.bl, 'displayText'), str(self.bl.getAllSavedActions())]
        entries['reset saved'] = getattr(self.bl, 'resetSavedActions')

        self.mm.addGenericMenu("saveMenu",self.mm.cur_page,"Select the action to save the predicates to", entries)
        self.mm.loadMenu("saveMenu")


    def createSequence(self,**kwargs):
        """
            Creates a menu of all existing actions learned in baxter_learner.py including the block locator and adds them to the selection
        """
        members = self.bl.getAllSavedActions() 
        entries={}

        num = len(self.actionSequence)
        self.baxter.mm.changeMenuTitle("%f actions saved: %s" % (num, str(self.actionSequence)))

        for param in members:
            entries[str(param)] = self.chooseBlock

        entries["Run Sequence"] = self.runSequence
        entries["Reset"] = self.resetSequence
        self.mm.addGenericMenu("sequenceMenu",self.mm.cur_page,"Select the action to add to the sequence", entries)
        self.mm.loadMenu("sequenceMenu")

    def get_colour_position(self, **kwargs):
        """
            Searches the coloured block
        """
        try:
            colour = kwargs["fname"].split(' ')
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return
        if len(colour) > 1:
            colour = colour[1]
        else:
            colour = colour[0]
        self.locator.recognise_grid()
        self.target_locations[colour] = self.locator.find_tetris_block(colour, return_center=True)
        
        self.mm.loadMenu("teachMenu")

    def loadSequence(self, **kwargs):
        """
            Reads action sequence saved in file
        """

        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return

        if self.baxter.br.mutex[side].locked():
            return
        self.baxter.br.stopExecution(side,False)

        try:
            threaded = kwargs['threaded']
        except Exception,e:
            threaded = False
        if threaded: 
            actionSeq = self.baxter.br.post.readSequence(side)
        else:
            actionSeq = self.baxter.br.readSequence(side)

        for action in actionSeq:
            if 'block -' in action:
                colour = action.rstrip().split(' - ')[1].rstrip()
            else:
                self.addAction(fname = colour, action = action)
        self.runSequence()


    def runSequence(self, **kwargs):
        """
            Runs actions that are saved in the actionSequence variable
        """

        answer = 'n'        
        action_number = 0

        while action_number < len(self.actionSequence):
#            pdb.set_trace()

            (colour,pose_offset) = self.actionSequence[action_number]
            rospy.loginfo('Running action %s for %s block' % (str(pose_offset), colour))
#            pose_offset = self.bl.baxter_actions[action]['joint_position']
#            goal_pose = tuple(map(operator.add, self.locator.pose, pose_offset))
            success = self.locator.locate(colour, pose_offset)

            rospy.sleep(0.2)
            if success:
                rospy.loginfo('Action %s executed successfully, moving to next action.' % str(pose_offset))
            else:
                answer = raw_input('Failed to execute action. Try again? (y/n): ')
                if answer in ('y'):
                    action_number -= 1
                    continue
                
            action_number += 1 

        self.baxter.mm.changeMenuTitle("Action sequence completed successfully!")

    def resetSequence(self, **kwargs):
        """
            Resets actions that are saved in the actionSequence variable
        """
        self.actionSequence = []
        self.baxter.mm.changeMenuTitle("Current sequence: %s " % str(self.actionSequence))


    def chooseBlock(self, **kwargs):
        try:
            action = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current action selection")

        colours = self.locator.tetris_blocks.keys() 
        entries = {}

        for block in colours:
            entries[str(block)] = [self.addAction, action]

        self.mm.addGenericMenu("blockMenu",self.mm.cur_page,"Select the block colour for the action", entries)
        self.mm.loadMenu("blockMenu")

    def addAction(self, **kwargs):
        """
            Adds action to action sequence list; if last action uses same coloured block, sum actions into one
        """

        try:
            colour = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current action selection")

        try:
            action = kwargs['action']
        except:
            action = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]

        if action in self.bl.getAllSavedActions():
            # if selected action modifies same block, add actions
            new = self.bl.baxter_actions[action]['joint_position']
            rospy.loginfo('New action is: %s' % str(new))
            if len(self.actionSequence) > 0 and colour == self.actionSequence[-1][0]:
                last_action = self.actionSequence[-1][1]
                new_action = tuple(map(operator.add, last_action, new))
                self.actionSequence[-1] = (colour, new_action)
                rospy.loginfo('Updated action to %s for %s block' % (str(self.actionSequence[-1][1]), colour))

            else:
                self.actionSequence.append((str(colour),new))
                rospy.loginfo('Added action %s for %s block' % (str(new), colour))
            self.mm.loadPreviousMenu()
        else:
            rospy.logwarn("Action does not exist. Skip action.")
            self.mm.neglect()



    def showGUI(self,**kwargs):
        """
            Redraws the GUI and selects the first element
        """
        self.baxter.menu.select(self.modes[0])
    
    def hideGUI(self,**kwargs):
        """
            Hides the GUI
            
            :param update: text display enabled?
            :type update: bool
        """
        try:
            update = kwargs['update']
        except Exception,e:
            update = True
        self.baxter.menu.hide(update)
        
    def back(self,**kwargs):
        """
            Loads the previous menu
        """
        self.mm.loadPreviousMenu()
    
    def goToInit(self,**kwargs):
        """
            Moves both arms to the init position
        """
        self.baxter.yes()
        self.baxter.hlm.post.goToInitPose()
        

    def computeAndInsertBox(self,**kwargs):
        """
            Computes the transformation for the 2 previously saved points in relation to the base and inserts the selected box type
            in the scene. This also includes additional transformations which are related to the box.
            Also writes everything to the current open task in case the transformation should be saved.
            
            :param side: influences the name of the inserted box and broadcasted tf
            :type side: str
        """
        if self.predefined_box is None:
            self.mm.neglect()
            return
        (pose,new_frame) = self.baxter.frame.computeTransformation() 
        if pose is None:
            self.mm.neglect()
            return
        
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return
        else:
            self.baxter.frame.setTF(self.predefined_box+'_'+side,pose)
            self.baxter.frame.waitUntilFrameUpdate(self.predefined_box+"_"+side)
            self.baxter.scene.createPredefinedBox(self.predefined_box+"_"+side,self.predefined_box)
            if self.learning:
                self.appendToTask("import tf_helper \n")
                self.appendToTask("side='%s'\n"%(side))
                self.appendToTask("baxter.bb.predefined_box='%s'\n"%(self.predefined_box))
                self.appendToTask("pose = tf_helper.PS('%s',%s,%s)\n"%(FRAME_ORIGIN,list(pose.pose.position),list(pose.pose.orientation)))
                self.appendToTask("baxter.frame.setTF('%s_'+side,pose)\n"%(self.predefined_box))
                self.appendToTask("baxter.frame.waitUntilFrameUpdate('%s_'+side)\n"%(self.predefined_box))
                self.appendToTask("baxter.scene.createPredefinedBox(baxter.bb.predefined_box+'_'+side,baxter.bb.predefined_box)\n")
                if self.predefined_box == "wako" or self.predefined_box.startswith("tray") is True or self.predefined_box.startswith("table") is True:
                    self.appendToTask("for drop_off in baxter.scene.boxes[baxter.bb.predefined_box][1].keys():\n"%())
                    self.appendToTask("    pose = tf_helper.PS('%s_'+side,%s,%s)\n"%(self.predefined_box,"baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][0:3]","baxter.scene.boxes[baxter.bb.predefined_box][1][drop_off][3:7]"))
                    self.appendToTask("    baxter.frame.setTF(drop_off+'_'+side,pose)\n")
            if self.predefined_box == "wako" or self.predefined_box.startswith("tray") is True or self.predefined_box.startswith("table") is True:
                for drop_off in self.baxter.scene.boxes[self.predefined_box][1].keys():
                    pose = PS(self.predefined_box+'_'+side,self.baxter.scene.boxes[self.predefined_box][1][drop_off][0:3],self.baxter.scene.boxes[self.predefined_box][1][drop_off][3:7])
                    self.baxter.frame.setTF(drop_off+'_'+side,pose)
            self.mm.confirm()
        
        
    def addSceneBox(self,**kwargs):
        """
            Set the current box type to the selected option in the GUI
        """
        box = self.mm.modes[self.mm.cur_mode]
        self.predefined_box = box 
        self.mm.confirm()
        self.baxter.yes()
        self.mm.loadPreviousMenu()
        
        
    def selectBoxType(self,**kwargs):
        """
            Creates a menu with all available predefined boxes
        """
        elements = self.baxter.scene.boxes.keys()

        entries={}
        for element in elements:
            entries[element] = self.addSceneBox
        self.mm.addGenericMenu("selectBox","addBox","Select a box type you want to add", entries)
        self.mm.loadMenu("selectBox")

    
     
###  TEACHING MODE
    def stopTeachedPath(self,**kwargs):
        """
            Stops a trajectory execution that is called by the teaching interface
        """
        rospy.loginfo("stop execution called")
        if not self.mm.modes[self.mm.cur_mode].find("left") == -1:
            side="left"
        elif not self.mm.modes[self.mm.cur_mode].find("right") == -1:
            side="right"
        else:
            rospy.logwarn("could not determine arm to stop")
        self.baxter.br.stopExecution(side,True)
        self.baxter.yes()
          
    def teach(self,**kwargs):
        """
            Teaches a new trajectory and overwrites the old one - called by baxter_scenarios and baxter_helper_menu
            
            :param side: arm to be taught
            :type side: str
            :param number: if multiple trajectories are saved the number is added to the name
            :type number: int
            :param parent: parent menu from which this function is called, to be able to return properly
            :type parent: str 
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return
        try:
            number = kwargs['number']
        except:
            number = ""
        try:
            parent = kwargs['parent']
        except:
            parent = "teachMenu"
        entries = {}
        self.baxter.yes() # head nod confirm
        entries["Stop recording"] = self.saveTeachPath
        self.mm.addGenericMenu("teach",parent,"Saving current path...", entries)
        self.mm.loadMenu("teach")
        self.bl.update_watch_parameters('before')
        self.baxter.br.post.record(side,str(number))


       
    def saveTeachPath(self,**kwargs):
        """
            Stops the current recording, saves it and return to the previous menu
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return
        self.baxter.br.stopRecording(side)


        # done with learning - go back to previous

        self.bl.update_watch_parameters('after')
        self.bl.create_action()
        self.baxter.yes() # head nod confirm
        self.mm.loadPreviousMenu()
    
    
    def executePath(self,**kwargs):
        """
            Executes a saved trajectory. 
            
            .. note:: The parameters below are not the method parameters but the entries in the kwargs
            
            :param side: arm to execute the trajectory
            :type side: str
            :param looped: Play infinitely?
            :type looped: bool
            :param number: Trajectory to be executed
            :type number: int
            :param threaded: Nonblocking? Threaded?
            :type threaded: bool
           
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return

        if self.baxter.br.mutex[side].locked():
            return
        self.baxter.br.stopExecution(side,False)
        try:
            looped = kwargs['looped']
        except: #if looped is not defined, use default value that has been passed
            looped = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        try:
            number = kwargs['number']
        except Exception,e:
            number = ""
        try:
            threaded = kwargs['threaded']
        except Exception,e:
            threaded = True
        if threaded: 
            self.baxter.br.post.play(side,[1,-1][looped],str(number))
        else:
            return self.baxter.br.play(side,[1,-1][looped],str(number))
        



### ARM SELECTION
    
    def useArm(self,**kwargs):
        """
            Set the arm that is selected in the GUI to True to be able to be used.
        """
        if not self.mm.modes[self.mm.cur_mode].find("left") != -1:
            rospy.loginfo("Using the left arm")
            self.use_arm["left"] = True
        elif not self.mm.modes[self.mm.cur_mode].find("right") != -1:
            rospy.loginfo("Using the right arm")
            self.use_arm["right"] = True
        else:
            self.baxter.no()
            return
        self.mm.confirm()
        self.baxter.yes()
        
    def selectArms(self,**kwargs):
        """
            Set both arms to False, when the menu is loaded.
            Afterwards it offers a menu to select the arms to be used.
        """
        self.use_arm["left"] = False
        self.use_arm["right"] = False
        entries = {}
        entries["Use Left"] = self.useArm
        entries["Use Right"] = self.useArm
        self.mm.addGenericMenu("selectArms","main","Select the arm you want to use", entries)
        self.mm.loadMenu("selectArms")


###################""

    def btnToggleSonar(self,**kwargs):
        if self.baxter.sonar.state == 0:
            self.baxter.sonar.enable()
        else:
            self.baxter.sonar.disable()

    def setHRLogo(self,**kwargs):
        """
            Displays the HumaRobotics Logo on baxter's display
        """
        self.baxter.display.setImage(self.baxter.datapath + "logo1024.jpg")
        
        
###########
#gripper setup

    def gripperType(self,**kwargs):
        """
            Creates a menu to change the length of the gripper (electric and suction)
            
            :param side: gripper side to be changed
            :type side: str 
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return
        gripper_type = self.baxter.gripper[side].type()
        
        if gripper_type=="suction":
            dimensions = ["Short","Middle","Long"]
        elif gripper_type=="electric":
            dimensions = ["Short","Long"]
        else:
            dimension = ["Unknown Gripper"]
            rospy.loginfo("Unknown Gripper")
            self.baxter.no()
            self.mm.neglect()
            return
        entries={}
        for dimension  in dimensions:
            entries[dimension] = [self.selectGripper,gripper_type]
        
        self.mm.addGenericMenu("gripperType","main","Select your gripper type", entries)
        self.mm.loadMenu("gripperType")

    def selectGripper(self,**kwargs):
        """
            Sets the gripper length according to the type of the gripper and the selected length from the previous menu.
            
            :param side: gripper side to be changed  
            :type side: str
        """
        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return
        gripper_type = self.mm.default_values[self.mm.modes[self.mm.cur_mode]]
        dimension = self.mm.modes[self.mm.cur_mode]
        if gripper_type=="suction":
            if dimension == "Short":
                length = 0.06
            elif dimension == "Middle":
                length = 0.06 + 0.05
            elif dimension == "Long":
                length = 0.06 + 0.05*2
        elif gripper_type=="electric":
            if dimension == "Short":
                length = 0.125
            elif dimension == "Long":
                length = 0.162
        else:
            self.baxter.no()
        self.baxter.scene.gripperLength[side] = length 
        self.baxter.scene.attachGripper(side,gripper_type,self.baxter.gripper[side].opened)
        self.baxter.yes()
        self.mm.loadPreviousMenu()
################
