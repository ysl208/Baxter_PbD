#!/usr/bin/env python

########################################################################### 
# Author: Ying Siu Liang
# Team MAGMA, LIG
#############################################################################

class BaxterAction:

    def __init__(self, action_id=0):
        self.action_id = action_id
        self.action_name = "new_action"
        self.parameters = {}
        self.preconditions = []
        self.effects = []
        self.action = []


    def setActionId(self, action_id):
	self.action_id = action_id

    def getActionId(self):
	return self.action_id


    def setActionName(self, action_name):
	self.action_name = action_name

    def getActionName(self):
	return self.action_name


    def setParameters(self, parameters):
	self.parameters = parameters

    def getParameters(self):
	return self.parameters


    def setPreconditions(self, preconditions):
	self.preconditions = preconditions

    def getPreconditions(self):
	return self.preconditions


    def setEffects(self, effects):
	self.effects = effects

    def getEffects(self):
	return self.effects


    def setAction(self, action):
	self.action = action

    def getAction(self):
	return self.action


    def printPDDLaction(self):
        pddl = '(:action %s \n' % self.action_name
        pddl += '\t:parameters (%s)\n' % self.printParam(self.parameters) # :parameters  (?from ?to - position)
        pddl += '\t:precondition (%s)\n' % self.printPredicates(self.preconditions) # :precondition (at-gripper ?g - gripper ?from - position)
        pddl += '\t:effect (%s))\n' % self.printPredicates(self.effects)
        return pddl
        
    def printParam(self, array):
        """
            Prints the array of parameters in PDDL format
        """
        output = ''
        for key in array:
            for value in array[key]:
                output += '?%s ' %value
            output += '- %s ' %key
        return output


    def printPredicates(self, array):
        """
            Prints the array of predicates in PDDL format
        """
        output = ''
        for pred in array:
            predicate = pred['predicate']
            if predicate.startswith('not('):
            # if the predicate is not(), move the closing braket to the end
                output += "\n\t\t(%s %s))" % (predicate[:-1], self.printParam(pred['params']))
            else:
                output += "\n\t\t(%s %s)" % (predicate, self.printParam(pred['params']))

        if len(array) > 1:
           output = 'and %s' %output
        return output

