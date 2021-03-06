#!/usr/bin/env python

import pdb
#####################################################################################
#                                                                                   #
# Copyright (c) 2016                                                                #
# All rights reserved.                                                              #
#                                                                                   #
# Author: Ying Siu Liang                                                            #
# Team MAGMA, LIG                                                                   #
#                                                                                   #
#####################################################################################

import random, math
import rospy
import tf
from hr_helper.post_threading import Post
from boxrenderer import BoxRenderer1
import baxter_helper_abstract_limb
RENDER_ERRORS=True
import baxter_helper_simple_limb
from baxter_action import BaxterAction
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

class Grid:
    """
        This class contains the tetris grid and information if a cell is free or occupied.  
    """
    def __init__(self,baxter):
        """
            :param baxter: object to access all robot functions
            :type baxter: BaxterRobot
        """
        self.baxter = baxter
        self.post = Post(self)

        self.rows = 10
        self.columns = 20

        self.matrix = [[]]
        self.matrix_2print = [[]]

    def setUpGrid(self):
        self.matrix = [ [ "free" for i in range(rows)] for j in range(columns)  ]
        self.matrix_2print = [ [ "**" for i in range(rows)] for j in range(columns)  ]

    def getSize(self):
        return (self.x_size,self.y_size)

class BaxterLearner:
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

        #Tetris grid
        self.grid = Grid(baxter)

        #Baxter demonstration parameters
        self.baxter_actions = {#'move_left': {'gripper_free': '', 'joint_position': (-0.06,0.0,0.0,0,0,0),'side': 'right'},
                               'MOVE': {'gripper_free': '', 'joint_position': (0.0,0.16,0.0,0,0,0),'side': 'right'},
#                               'move_block_back': {'gripper_free': '', 'joint_position': (0.0,-0.16,0.0,0,0,0),'side': 'right'},
#                               'rotate': {'gripper_free': '', 'joint_position': (0,0,0,0,0,math.pi/2),'side': 'right'}
}
        self.current_action = BaxterAction()
        self.__watch_parameters = {'joint_position': ['',''], #[before,after]
                             #'gripper_orientation': ['',''],
                             #'holding': ['',''],
                             'gripper_free': ['','']
                             }
        self.action_index = 0
        self.__action_name = "action"
        self.__side = "right"
        self.__all_actions = {'move_block':[], #'move_w':[], 'move_s':[], 'move_e':[], 
                              #'move_left':[], 'move_right':[], 'move_up':[], 'move_down':[], 
                              'rotate':[],# 'rotate_acw':[],
                              #'pick':[], 'drop':[]
                             }

        #Tetris predicates
        self.__current_predicate = ""
        self.__preconditions = []
        self.__effects = []
        self.__isPrecond = True
        self.__predicates = {#'gripper-at':['gripper','xy_position'],
                             'block_at_position': ['block', 'xy_position'],
                             'gripper_free': [],
                             'gripper_holding_block': ['block'],
                             'position_free': ['xy_position']
                             }

        #Tetris parameters
        self.__params = {'xy_position':['xy_from','xy_to'], 'block':['yellow','red','blue','green']} #, 'gripper':['g0']


    def displayText(self, **kwargs):
        """
            Displays text on screen
        """

        text = self.baxter.mm.default_values[self.baxter.mm.modes[self.baxter.mm.cur_mode]]

        entries = {}
        entries[str(text)] = self.baxter.bb.nothing
        parent = self.baxter.mm.page[self.baxter.mm.cur_page][0]
        self.baxter.mm.addGenericMenu("display", self.baxter.mm.cur_page, "The current selection is:", entries)
        self.baxter.mm.loadMenu("display")

    def loadActions(self, **kwargs):
        """
            Loads the actions saved from files
            TO DO
        """


    def savePredicatesToAction(self, **kwargs):
        """
            Saves the entered predicates and action parameters to the action
        """
        try:
            action = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current action selection")

        empty_params = [len(item) for sublist in self.__params.values() for item in sublist]
        if len(empty_params) == 0 or len(self.__effects) == 0 or len(self.__preconditions) == 0 or len(self.__all_actions[action]) == 0:
            self.baxter.mm.default_values[self.baxter.mm.modes[self.baxter.mm.cur_mode]] = "Missing predicates or action. Please complete before saving. Press Back."
            self.displayText()
        else:
            baxter_action = BaxterAction(self.action_index)
            baxter_action.setActionName(action)
            baxter_action.setParameters(self.__params)
            baxter_action.setEffects(self.__effects)
            baxter_action.setPreconditions(self.__preconditions)
            baxter_action.setAction(self.__all_actions[action])
            self.baxter_actions[action] = baxter_action
            self.action_index = len(self.baxter_actions)
            self.baxter.yes()
            pddl = baxter_action.printPDDLaction()
            rospy.loginfo(pddl)
            self.baxter.mm.changeMenuTitle("Saved new action %s" % (action))
            side = self.__all_actions[action]['side']
            file = self.baxter.br.filename+side+'_'+action + '_pddl'
            rospy.loginfo(self.baxter.br.filename+side+action + '_pddl')
            with open(file, 'w') as f:
                f.write(pddl)
            rospy.loginfo("saved PDDL file for %s" % action)

    def getActionName(self, **kwargs):
        """
            Returns the current action being learned
        """
        return self.__action_name

    def getAllSavedActions(self, **kwargs):
        """
            Returns atomic actions that have been saved
        """
        return self.baxter_actions.keys()

    def resetSavedActions(self, **kwargs):
        """
            Returns atomic actions that have been saved
        """
        self.baxter_actions = {}

    def getAllActionNames(self, **kwargs):
        """
            Returns atomic actions that can be learned
        """
        return self.__all_actions.keys()

    def demoAction(self, **kwargs):
        """
            Learns the action from the user's demonstration
        """
        try:
            action = kwargs["fname"].split(' ')[1]
        except:
            rospy.logwarn("Could not get the current action selection")
        self.__action_name = action

        try:
            side = kwargs['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.baxter.mm.neglect()
            return
        self.__side = side

        self.baxter.bb.teach(**{'side':side,'number':self.action_index, 'parent': 'actionMenu'})


    def update_watch_parameters(self, time):
        """
           Updates the watch parameters for before/after conditions
        """
        i = 0 if time == 'before' else 1
        side = self.__side
        pose = self.baxter.arm[side].getPose()
        self.__watch_parameters['joint_position'][i] = pose
        self.__watch_parameters['gripper_free'][i] = self.baxter.gripper[side].gripped()

    def getAnglesFromPose(self, pose):
        """
           Returns the angle of the camera around the z axis
        """
        temp = pose.orientation
        orient = tf.transformations.euler_from_quaternion((temp.x,temp.y,temp.z,temp.w))
        return orient[2]*180/math.pi

    def create_action(self):
        """
           Creates new action based on the before and after states of the watch parameters
        """

        subtasks = {}
        subtasks['side'] = self.__side
        watch_params = self.__watch_parameters
        # calculate Pose difference from before and after
        diff = baxter_helper_abstract_limb.getPoseDiff(watch_params['joint_position'][1],watch_params['joint_position'][0])

        # round numbers to 2 decimal places, set to zero if change is minimal
        for idx, pos in enumerate(diff['position']):
            if abs(pos) >= 0.02:
                diff['position'][idx] = round(pos, 2)
            else:
                diff['position'][idx] = 0

        diff['position'][2] = 0 # keep the z coordinate the same

        # calculate Angle difference
        #pdb.set_trace()
        before_angles = self.getAnglesFromPose(watch_params['joint_position'][0])
        after_angles = self.getAnglesFromPose(watch_params['joint_position'][1])
        angle_diff = after_angles - before_angles 
        rotation_angle = angle_diff - (angle_diff%5)
        orientation = rotation_angle * (math.pi / 180)

        pose = (diff['position'][0],
                diff['position'][1],
                0,
                0,
                0,
                orientation)
        subtasks['joint_position'] = pose
        rospy.loginfo("Movement recorded at %s deg:%s" % (str(rotation_angle),str(pose)))

        if watch_params['gripper_free'][0] is watch_params['gripper_free'][1]:
            subtasks['gripper_free'] = ''
        else:
            subtasks['gripper_free'] = watch_params['gripper_free'][1]

        self.__all_actions[self.__action_name] = subtasks
        rospy.loginfo(subtasks)
        self.baxter_actions[self.__action_name] = subtasks
        

        # create opposite effect automatically
        opposite = subtasks.copy()
        opp_pose = list(opposite['joint_position'])
        opposite['joint_position'] = tuple([-i for i in opp_pose])
#        self.__all_actions[self.__action_name + '_back'] = opposite
#        self.baxter_actions[self.__action_name + '_back'] = opposite


    # find distance of limb from nearest line of sight object
    def get_distance(self, limb):
        if limb == "left":
            dist = self.baxter.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = self.baxter.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        # convert mm to m and return distance
        return float(dist / 1000.0)


    def executeAction(self, **kwargs):
        """
            Executes the selected action
        """
        try:
            action = kwargs["fname"]
        except:
            action = self.baxter.mm.default_values[self.baxter.mm.modes[self.baxter.mm.cur_mode]]

        try:
            side = self.__all_actions[action]['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.baxter.mm.neglect()
            return

	pose = self.baxter.arm[side].getPose()
        rospy.loginfo(self.baxter.gripper[side].get_distance(side))

	pose_dict = baxter_helper_abstract_limb.getDictFromPose(pose)
	rospy.loginfo("action: %s" % self.__all_actions[action])

	pose_change = (self.__all_actions[action]['joint_position'])
	goal_pose = baxter_helper_abstract_limb.getPoseAddition(pose,pose_change)
	rospy.loginfo("goal %s" % goal_pose)
	self.baxter.arm[side].goToPose(baxter_helper_abstract_limb.getPoseFromDict(goal_pose), speed=0.7)
	pose = self.baxter.arm[side].getPose()
	rospy.loginfo("end %s" % pose)

        # gripper changes
	gripper_change = self.__all_actions[action]['gripper_free']
        if type(gripper_change) is bool:
            if gripper_change:
                self.baxter.gripper[side].grip()
            else: 
                self.baxter.gripper[side].release()


    def paramReset(self, **kwargs):
        """
            Remove all created parameters
        """
        self.__params = dict.fromkeys(self.__params.keys(),[])

    def getAllParameters(self, **kwargs):
        """
            List of parameters for the learned operator
        """
        return self.__params

    def paramSelection(self, **kwargs):
        """
            Adds this to the list of parameters for the learned operator
        """
        try:
            param_type = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current parameter selection")

        # new parameter name consists of the first character and the next available number
        new_param = "%s%s" % (str(param_type[0]), str(len(self.__params[param_type])))
        self.__params[param_type].append(new_param)
        self.baxter.yes() 
        self.baxter.mm.changeMenuTitle("Parameters for %s: %s" % (param_type, str(self.__params[param_type])))


    def isPrecondition(self, pred):
        """
            Flag for whether the predicates are preconditions or effects
        """
        self.__isPrecond = pred


    def getAllPredicates(self):
        """
            Return the all predicates
        """
        if self.__isPrecond:
            return self.__preconditions
        else:
            return self.__effects

    def getCurrentPredicate(self):
        """
            Return the currently selected predicate
        """
        return self.__predicates
        
    def setPredicate(self,predicate):
        """
            Sets a predicate
        """
        self.__current_predicate = predicate

    def predicateReset(self, **kwargs):
        """
            Remove all created preconditions
        """
        if self.__isPrecond:
            self.__preconditions = []
            #self.__preconditions = dict.fromkeys(self.__preconditions.keys(),[])
        else:
            self.__effects = []
            #self.__effects = dict.fromkeys(self.__effects.keys(),[])

    def predicateSelection(self, **kwargs):
        """
            Adds this predicate to the list of predicates for the learned operator
        """
        try:
            predicate = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current predicate selection")

        param_types = self.__predicates[predicate]
        
        params = {p:'empty' for p in param_types }
        self.__current_predicate = {'predicate':predicate, 'params': params}
        
        # Create selection for parameters
        entries = {}

        for ptype in param_types:
            entries[ptype] = self.__predicateParamMenu

        entries['submit'] = self.__predicateCheckMenu
        entries['view predicate'] = [self.displayText,str(self.__current_predicate)]

        self.baxter.mm.addGenericMenu("paramMenu",self.baxter.mm.cur_page,"Choose parameter type for the new operator", entries)
        self.baxter.mm.loadMenu("paramMenu")

    def __predicateParamMenu(self, **kwargs):
        """
            Add selected parameter and final predicate
        """
        try:
            param_type = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current parameter type selection")

        params = [p for p in self.__params[param_type]]
        entries = {}

        for param in params:
            entries[param] = [self.__addParam, param_type]

        title = "Choose parameter for %s" % param_type
        self.baxter.mm.addGenericMenu("parameters", self.baxter.mm.cur_page, title, entries)
        self.baxter.mm.loadMenu("parameters")


    def __addParam(self, **kwargs):
        """
            Adds the chosen parameter to the created predicate out of the list of created parameters
        """
        try:
            param = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current parameter selection")

        param_type = self.baxter.mm.default_values[self.baxter.mm.modes[self.baxter.mm.cur_mode]]

        self.__current_predicate['params'][param_type] = param
        self.baxter.yes()
        self.baxter.mm.loadPreviousMenu()


    def __predicateCheckMenu(self, **kwargs):
        """
            Negates the selected predicate for the learned operator
        """
        entries = {}
        if self.__predicateComplete():
            entries = {
                   "Yes": [self.negatePredicate,True],
                   "No": [self.negatePredicate,False]
            }
            title = "Negate selected predicate?"
            parent = "predicate"
        else:
            entries = {
                   str(self.__current_predicate): self.baxter.bb.nothing
            }
            title = "Missing values in predicate. Press back to return"
            parent = self.baxter.mm.cur_page

        self.baxter.mm.addGenericMenu("negateMenu",parent,title, entries)
        self.baxter.mm.loadMenu("negateMenu")


    def negatePredicate(self, **kwargs):
        """
             Negates the currently selected predicate and adds it to the list
        """

        if len(self.__current_predicate) == 0:
            rospy.logwarn("Could not get the current predicate selection")

        try:
            negate = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current negation selection, assume No")
            negate = "No"

        if negate == "Yes":
            self.__current_predicate['predicate'] = "not(%s)" % self.__current_predicate['predicate']

        if self.__isPrecond:
            self.__preconditions.append(self.__current_predicate)
        else:
            self.__effects.append(self.__current_predicate)

        self.baxter.yes()
        self.baxter.mm.loadPreviousMenu()        

    def __predicateComplete(self, **kwargs):
        """
            Checks if the current predicate has all parameters selected
        """
        predicate = self.__current_predicate

        if predicate['predicate'] not in self.__predicates:
            return False
        else:
            #params = [item for sublist in predicate['params'].values() for item in sublist]
            params = [item for item in predicate['params'].values()]
            existing_params = [item for sublist in self.__params.values() for item in sublist]
            for param in params:
                if len(param) == 0 or param not in existing_params:
                    return False
            return True

    def printPDDLheader(self, **kwargs):
        print '(define (problem Tetris-'+str(size_grid)+'-'+str(conf_blocks)+'-'+str(random.randint(0,9875232))+')\n'
        print '(:domain tetris)\n'
        print '   (:requirements :typing)\n'
        print '   (:types position \n'
        print '           gripper\n'
        print '           block)\n'
        print '   (:predicates (at-gripper ?g - gripper ?r - position)\n'
        print '        (at ?b - block ?r - position)\n'
        print '        (free ?g - gripper)\n'
        print '        (carry ?o - block ?g - gripper)\n'
        print '        (clear ?xy - position)\n'
        print '   )'

