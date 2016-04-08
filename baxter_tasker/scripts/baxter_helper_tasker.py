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
"""
Helper Classes and functions for Baxter
"""
import copy
from  hr_helper.post_threading import *

class Task:
    """
        Creates a new task in the GUI on the workstation that has access to the robot.
        Additionally it can be instantly executed, saved and reloaded
    """
    def __init__(self,tasker):
        self.tasker=tasker
        self.baxter=tasker.baxter
        
    def perform(self):
        return True

class TaskEval(Task):
    """
        Evaluates a previously defined task
    """
    def __init__(self,tasker,toeval):
        Task.__init__(self,tasker)
        self.toeval=toeval
    
    def perform(self):
        """
            Executes a task by evaluating the text in the task
        """
        if self.toeval.strip()=="": return None
        rospy.loginfo("%s"%str(dir(self)))
        # Wrap in function with return value
        self.cmd="def _localfunction():\n"
        for l in self.toeval.split("\n"):
            self.cmd+="  "+l+"\n"
        self.cmd+="_return=_localfunction()"
        
        # Build context        
        context=self.buildContext()
        exec self.cmd in context
        return context["_return"]
    
    def buildContext(self):
        # Build context        
        context=dict(globals())
        exec self.tasker.tasks["_globals"].toeval in context
        context["baxter"]=self.baxter                
        for task in self.tasker.tasks.keys():
            context[task]=lambda task=task: self.tasker.perform(task)
        return context
        
        

class Tasker:
    """
        Manages the tasks that are visible in the GUI
    """
    def __init__(self,baxter):        
        """
            :param baxter: handle to all function of the robot
            :type baxter: BaxterRobot
        """
        self.baxter=baxter
        self.post=Post(self)
        self.tasks={}
        self.add("_globals",TaskEval(self,"bip=5"))
        
    
    def perform(self,taskname):
        """
            Calls the perform function of an existing task
            
            :param taskname: Name of the task
            :type taskname: str
        """
        rospy.loginfo("Executing task %s"%taskname)
        ret=self.tasks[taskname].perform()
        rospy.loginfo("Done executing task %s result=%s"%(taskname,str(ret)))
        return ret
        

    def add(self,taskname,task):
        """
            Adds a new task to the GUI
            
            .. note:: The task is only saved if the "Save tasks" in the menu is invoked
            
            :param taskname: Name of the task
            :type taskname: str
            :param task: Handle to the task
            :type task: Task
        """
        self.tasks[taskname]=task

        
        
if __name__=="__main__":
    cmd="""
    """
    tasker=Tasker(None)
    te1=TaskEval(tasker,"""
print 'task1'
print task2()
print 'task1 done'
return True
"""
    )
    tasker.add("task1",te1)

    te2=TaskEval(tasker,"""
print 'task2'
print 'task2 done'
print bip
return "ret2"
"""
    )
    tasker.add("task2",te2)

    tasker.perform("task1")