#!/usr/bin/env python

import pdb
########################################################################### 
# Author: Ying Siu Liang
# Team MAGMA, LIG
#############################################################################
import random

import rospy
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
        self.baxter_actions = {}
        self.current_action = BaxterAction()
        self.__watch_parameters = {'joint_position_x': ['',''], #(before,after)
                             #'joint_position_y': ['',''],
                             #'joint_position_z': ['',''],
                             #'gripper_orientation': ['',''],
                             #'holding': ['',''],
                             'gripper_free': ['','']
                             }
        self.action_index = 0
        self.__action_name = "action"
        self.__side = "right"
        self.__all_actions = {#'arm_down':[], 'arm_up':[],
                              'move_n':[], 'move_w':[], 'move_s':[],
                              'move_e':[], 'rotate_cw':[], 'rotate_acw':[], 'pick':[], 'drop':[]}

        #Tetris predicates
        self.__current_predicate = ""
        self.__preconditions = []
        self.__effects = []
        self.__isPrecond = True
        self.__predicates = {'gripper-at':['gripper','position'],
                             'block-at': ['block', 'position'],
                             'free': ['gripper'],
                             'holding': ['gripper', 'block'],
                             'clear': ['position']
                             }

        #Tetris parameters
        self.__params = {'position':['p0'], 'block':['b0'], 'gripper':['g0']}


    def displayText(self, **kwargs):
        text = self.baxter.mm.default_values[self.baxter.mm.modes[self.baxter.mm.cur_mode]]

        entries = {}
        entries[str(text)] = self.baxter.bb.nothing
        parent = self.baxter.mm.page[self.baxter.mm.cur_page][0]
        self.baxter.mm.addGenericMenu("display", self.baxter.mm.cur_page, "The current selection is:", entries)
        self.baxter.mm.loadMenu("display")

    def loadActions(self, **kwargs):
        """
            Loads the actions saved from files
        """


    def savePredicatesToAction(self, **kwargs):
        """
            Saves the entered predicates and action parameters to the action
        """
        try:
            action = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current action selection")
        pdb.set_trace()
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
            action = kwargs["fname"]
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
#        self.initialiseArmDown()
        self.baxter.bb.teach(**{'side':side,'number':self.action_index, 'parent': 'actionMenu'})


    def update_watch_parameters(self, time):
        """
           Updates the watch parameters for before/after conditions
        """
        i = 0 if time == 'before' else 1
        side = self.__side
        pose = self.baxter.arm[side].getPose()
        self.__watch_parameters['joint_position_x'][i] = pose
        self.__watch_parameters['gripper_free'][i] = self.baxter.gripper[side].gripped()

    def create_action(self):
        """
           Creates new action based on the before and after states of the watch parameters
        """
        subtasks = {}
        side = self.__side
        watch_params = self.__watch_parameters
        diff = baxter_helper_abstract_limb.getPoseDiff(watch_params['joint_position_x'][1],watch_params['joint_position_x'][0])
        diff['position'][2] = 0 #keep the z coordinate the same
#        diff['orientation'] = baxter_helper_abstract_limb.getDictFromPose(watch_params['joint_position_x'][0])['orientation']
        subtasks['side'] = side
        subtasks['joint_position_x'] = baxter_helper_abstract_limb.getPoseFromDict(diff)
#        subtasks['joint_position_y'] = baxter_helper_abstract_limb.getPoseFromDict(diff)
        # if gripper_free didnt change, do nothing and leave it as not holding
        # pick: gripper open > close :: True > False
        # pick: suction open > close :: False > True
        if watch_params['gripper_free'][0] is watch_params['gripper_free'][1]:
            subtasks['gripper_free'] = ''
        else:
            subtasks['gripper_free'] = watch_params['gripper_free'][1]
        rospy.loginfo(self.__watch_parameters['gripper_free'])
        self.__all_actions[self.__action_name] = subtasks

    def initialiseArmDown(self, **kwargs):
        """
            Creates the change motion parameter for moving the arm down to the block
            TO DO: need to keep moving down until target hit
        """
	pose = Pose()
	pose.position.x = 0
	pose.position.y = 0
	pose.position.z = 0.0987

        self.__all_actions['arm_down'] = pose

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


    def approach(self):
        while self.get_distance(self.limb) > 0.12:
            self.update_pose(0, 0, -0.05)
            last_distance = self.get_distance(self.limb)
            print self.get_distance(self.limb)


    def executeAction(self, **kwargs):
        """
            Executes the selected action
        """
        try:
            action = kwargs["fname"]
        except:
            rospy.logwarn("Could not get the current action selection")

        try:
            side = self.__all_actions[action]['side']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.baxter.mm.neglect()
            return

	pose = self.baxter.arm[side].getPose()
        rospy.loginfo(self.baxter.gripper[side].get_distance(side))

	pose_dict = baxter_helper_abstract_limb.getDictFromPose(pose)
	rospy.loginfo("start: %s" % self.__all_actions[action])

	pose_change = (self.__all_actions[action]['joint_position_x'])
	goal_pose = baxter_helper_abstract_limb.getPoseAddition(pose,pose_change)
#        goal_pose['position'][0] = pose_dict['position'][0] #keep the x coordinate the same
#        goal_pose['position'][2] = pose_dict['position'][2] #keep the z coordinate the same
	rospy.loginfo("goal %s" % goal_pose['position'])
	self.baxter.arm[side].goToPose(baxter_helper_abstract_limb.getPoseFromDict(goal_pose), speed=0.7)
	pose = self.baxter.arm[side].getPose()
	rospy.loginfo("end %s" % pose.position)

        # gripper changes
	gripper_change = self.__all_actions[action]['gripper_free']
        if type(gripper_change) is bool:
            if gripper_change: #move down and grip
#                pose.position.z = -0.159
#              	self.baxter.arm[side].goToPose(pose, speed=0.7)
                self.baxter.gripper[side].grip()

            else: 
                self.baxter.gripper[side].release()


    def paramReset(self, **kwargs):
        """
            Remove all created parameters
        """
        self.__params = {'position':[], 'block':[], 'gripper':[]}

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
        else:
            self.__effects = []

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
