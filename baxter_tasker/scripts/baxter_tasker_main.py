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
import time
from Tkinter import *
import tkSimpleDialog
import tkFileDialog
import glob
import traceback
import rospy
import os
from baxter_helper import *
import baxter_helper_tasker

import python_text
DEFAULT_EXTENSION=".task.py"
 
"""
    Main class for the baxter tasker that creates the GUI and objects for the robot.
"""

class MainWindow:
    """
        The main window for baxter tasker. It links the buttons on the robot and the GUI to their corresponding behaviors.
        
        :param master: Handle to the Tkinter object
        :type master: Tk
        :param tasker: Handle to the task management. Contains also the robot object that can be accessed in the GUI
        :type tasker: baxter_helper_tasker.Tasker
    """
    def __init__(self,master,tasker):
        self.master=master
        self.tasker=tasker
        self.baxter=tasker.baxter
        self.post=Post(self)
        self.buildMenu(master)
        #~ self.frame = Frame(master, width= self.width, height= self.height)
        self.frame = master
        self.master.bind('<Key-Escape>', self.exit )
        if tasker.baxter.mm == None:
            tasker.baxter.loadMenuManager()
        self.mm = tasker.baxter.mm
        if tasker.baxter.bb == None:
            tasker.baxter.loadBehaviors()
        self.bb = tasker.baxter.bb
        bs = self.bb.bs
        self.bb.addMainWindow(self.appendToTask)
        self.mm.addAllMenus()
        self.mm.loadMenu("main")
        #self.position=0
        self.selected=None
        #~ Button(self.frame,text="Refresh",command=self.refresh).grid(row=0,column=0)
        Button(self.frame,text="Add Task",command=self.addTask).grid(row=0,column=0)
        Button(self.frame,text="Execute Task",command=self.executeTask).grid(row=1,column=0)
        Button(self.frame,text="Enable/Disable Robot",command=self.baxter.toggleMotorState).grid(row=2,column=0)
        

        Button(self.frame,text="Back Button",command=(lambda: self.back("left"))).grid(row=4,column=0)
        Button(self.frame,text="Confirm Right Button",command=(lambda: self.confirm("right"))).grid(row=5,column=0)
        Button(self.frame,text="Confirm Left Button",command=(lambda: self.confirm("left"))).grid(row=6,column=0)
        Button(self.frame,text="Turn Wheel",command=(lambda: self.cycleMode("left"))).grid(row=7,column=0)
        Button(self.frame,text="Round Cuff Right Button",command=(lambda: self.posButton("right"))).grid(row=8,column=0)
        Button(self.frame,text="Round Cuff Left Button",command=(lambda: self.posButton("left"))).grid(row=9,column=0)
        Button(self.frame,text="Oval Cuff Right Button",command=(lambda: self.gripButton("right"))).grid(row=10,column=0)
        Button(self.frame,text="Oval Cuff Left Button",command=(lambda: self.gripButton("left"))).grid(row=11,column=0)
        Button(self.frame,text="Set HR Logo",command=(lambda: self.bb.setHRLogo(**{}))).grid(row=12,column=0)
        Button(self.frame,text="Create MoveIt URDF",command=(lambda: self.baxter.uc.createMoveItURDF())).grid(row=13,column=0)
        
        
        Button(self.frame,text="STOP",command=(lambda: self.bb.changeExecutionState())).grid(row=0,column=1)
        Button(self.frame,text="Continue",command=(lambda: self.bb.changeExecutionState(False))).grid(row=1,column=1)
        Button(self.frame,text="Go to Init",command=(lambda: self.bb.goToInit(**{}))).grid(row=2,column=1)
        Button(self.frame,text="Generate Plans",command=(lambda: self.bb.generator.generateAllPlans())).grid(row=3,column=1)
        Button(self.frame,text="Run Scenarios",command=(lambda: self.bb.run(**{}))).grid(row=4,column=1)
        
        
        Button(self.frame,text="Test Switch 1",command=(lambda: bs.btnPickItem())).grid(row=6,column=1)
        Button(self.frame,text="Test Switch 2",command=(lambda: bs.btnDropOff())).grid(row=7,column=1)
        Button(self.frame,text="Test Cover Small",command=(lambda: bs.btnCoverSmall())).grid(row=8,column=1)
        Button(self.frame,text="Test Cover Door",command=(lambda: bs.btnCoverDoor())).grid(row=9,column=1)
        Button(self.frame,text="Test Tray",command=(lambda: bs.btnTray())).grid(row=10,column=1)
        Button(self.frame,text="Test Goblet Blue",command=(lambda: bs.btnGobletBlue())).grid(row=11,column=1)
        Button(self.frame,text="Test Goblet Red",command=(lambda: bs.btnGobletRed())).grid(row=12,column=1)
        
        Button(self.frame,text="Toggle Sonar",command=(lambda: self.bb.btnToggleSonar())).grid(row=13,column=1)
        
         
        self.listTasks = Listbox(master,width=30,height=40)
        self.listTasks.grid(row=0,column=2,rowspan=20)
        self.listTasks.bind("<<ListboxSelect>>", self.selectTask)
        self.listTasks.bind("<Delete>", self.deleteTask)
        
        self.textTask=python_text.PythonText(self.frame,width=120,height=40)
        self.textTask.grid(row=0,column=3,rowspan=20)
        self.textTask.bind("<KeyRelease>",self.updateTask)

        self.baxter.dio["right_upper_button"].callbackOnPressed(lambda: self.gripButton("right") )
        self.baxter.dio["left_upper_button"].callbackOnPressed(lambda: self.gripButton("left") )

        self.baxter.dio["right_lower_button"].callbackOnPressed(lambda: self.posButton("right") )
        self.baxter.dio["left_lower_button"].callbackOnPressed(lambda: self.posButton("left") )
        
        self.baxter.dio["left_shoulder_button"].callbackOnPressed(self.baxter.toggleMotorState)
        self.baxter.dio["right_shoulder_button"].callbackOnPressed(self.baxter.toggleMotorState)

        self.baxter.navigator["left"].callbackOnPressedShow(lambda: self.home("left"))
        self.baxter.navigator["right"].callbackOnPressedShow(lambda: self.home("right"))
    
        self.baxter.navigator["left"].callbackOnPressedOk(lambda: self.confirm("left"))
        self.baxter.navigator["right"].callbackOnPressedOk(lambda: self.confirm("right"))
        self.baxter.navigator["left"].callbackOnPressedCancel(lambda: self.back("left"))
        self.baxter.navigator["right"].callbackOnPressedCancel(lambda: self.back("right"))         
    
        self.baxter.navigator["torso_left"].callbackOnPressedCancel(lambda: self.bb.changeExecutionState())
        self.baxter.navigator["torso_right"].callbackOnPressedCancel(lambda: self.bb.changeExecutionState())
        self.baxter.navigator["torso_left"].callbackOnPressedOk(lambda: self.bb.changeExecutionState(False))
        self.baxter.navigator["torso_right"].callbackOnPressedOk(lambda: self.bb.changeExecutionState(False))
       
        self.position = {'left':0,'right':0}
        self.baxter.navigator["left"].callbackWheel(lambda newstate: self.wheel("left",newstate))
        self.baxter.navigator["right"].callbackWheel(lambda newstate: self.wheel("right",newstate))
        self.refresh()
        
        
    def buildMenu(self, root):
        """
            Creates the menu bar
            
            :param root: Tkinter interface
            :type root: Tk
        """
        menubar = Menu(root)
        root.config(menu=menubar)
        filemenu = Menu(menubar)
        menubar.add_cascade(label='File', menu=filemenu)
        filemenu.add_command(label='Load all tasks', command=self.loadAllTasks)
        filemenu.add_command(label='Load task ...', command=self.loadTask)
        filemenu.add_command(label='Save tasks', command=self.saveTasks)
        filemenu.add_command(label='Quit', command=self.exit)

            
    def wheel(self,side,position):
        """
            Callback for the wheel buttons on the elbows are rotated. Checks in which direction the wheel was turned
            
            :param side: The side that has been altered
            :type side: str
            :param position: The new position of the wheel [0-255]
            :type position: int
        """
        if self.position[side] > position and  not(self.position[side] == 255 and position==0) or (self.position[side] == 0 and position==255) :
            increase = False
        else:
            increase = True
        self.position[side] = position
        self.cycleMode(increase)
               
    def home(self,side):
        """
            Callback for the home buttons on the elbows. Directly shows the root menu on the robot display
            
            :param side: The side that has been pushed
            :type side: str
        """
        if self.baxter.menu.hidden:
            self.baxter.camera.closeAllCameras()
            self.baxter.menu.show()
            return
            
        self.mm.loadMenu("main")        

    def back(self,side):
        """
            Callback for the back (arrow) buttons on the elbows. Shows the previous menu on the robot display
            
            :param side: The side that has been pushed
            :type side: str
        """
        if self.baxter.menu.hidden:
            self.baxter.camera.closeAllCameras()
            self.baxter.menu.show()
            return
        if not self.baxter.menu.hidden:
            self.mm.loadPreviousMenu()
            
    def confirm(self,side):
        """
            Callback for a button push for the wheel buttons on the elbows. Calls the current menu entry on the robot display
            
            :param side: The side that has been pushed
            :type side: str
        """
        # hide menu
        if self.baxter.menu.hidden:
            self.baxter.camera.closeAllCameras()
            self.baxter.menu.show()
            return
#         print "modes",self.mm.modes
#         print "selected", self.mm.cur_mode
        fname = self.mm.modes[self.mm.cur_mode]
        kwargs = {'position':self.position[side],'side':side,'fname':fname}
        rospy.loginfo( "calling method with name %s"%(fname))
        try:
            self.mm.callMenuEntry(fname,**kwargs)
        except Exception,e:
            rospy.logwarn("Could not call entry: %s"%str(e))
            try:
                self.baxter.no()
            except:
                pass

    def cycleMode(self,increase=True):
        """
            Selects the next or previous entry in the menu on the display. Called by the wheel method when the wheel is turned.
            
            :param increase: Move to the next entry?
            :param increase: bool
        """
        if self.baxter.menu.hidden:
            self.baxter.camera.closeAllCameras()
            self.baxter.menu.show()
            return
        num_modes = len(self.mm.modes)
        if num_modes != 0:
            if increase:
                self.mm.cur_mode = (self.mm.cur_mode +1)%num_modes
            else:
                self.mm.cur_mode = (self.mm.cur_mode -1)%num_modes
        #self.mm.changeMenuTitle("%s" % (action))
        rospy.loginfo("Current mode changed to %s",self.mm.modes[self.mm.cur_mode])
        if self.baxter.menu.hidden is False:
            self.baxter.menu.select(self.mm.modes[self.mm.cur_mode])
        
    def appendToTask(self,txt):
        """
            Appends text to the currently selected editor window in the GUI. Can be used by all behaviors to easily store robot data
            
            :param txt: Text to be written to the editor window
            :param txt: str
        """
        if self.selected:
            self.textTask.insert(END,txt)
            self.tasker.tasks[self.selected].toeval=self.textTask.get(1.0,END)
            self.textTask.colorize()
            
    def gripButton(self,side):
        """
            Callback when the oval (grip) button on the hand cuff is pressed. Opens or closes the gripper
            
            :param side: Side of the gripper to be activated
            :param side: str
        """
        opened = self.baxter.gripper[side].opened
        rospy.loginfo("Grip button pressed. gripper was %d"%(opened))
        if not opened:
            self.baxter.gripper[side].open()
        else:
            self.baxter.gripper[side].close([True, False][self.baxter.gripper[side].type()=='suction'])
        rospy.sleep(0.1)

    def posButton(self,side):
        """
            Callback for the round button on the hand cuff. This button is not assigned yet. Add your code here for additional functionality
            
            :param side: The side where the button was pressed
            :param side: str
        """
        rospy.loginfo("The grip button is not used at this very moment")
          
    def selectTask(self,event=None):
        """
            Selects a task
        """
        l=self.getSelectedKeys()
        if len(l)!=0:
            k=l[0]
            txt=self.tasker.tasks[k].toeval
            self.selected=k
            self.textTask.delete(1.0,END)            
            self.textTask.insert(END,txt)

    def refresh(self):
        """
            Refreshes the task list
        """
        self.listTasks.delete(0,END)
        keys=list(self.tasker.tasks.keys())
        keys.sort()
        for k in keys:
            self.listTasks.insert(END,k)
            
    def addTask(self):
        """
            Adds an empty task
        """
        defaulttaskcontent=""        
        ret=tkSimpleDialog.askstring('Add Task', 'Task name:')
        if ret:            
            self.tasker.add(ret,baxter_helper_tasker.TaskEval(self.tasker,defaulttaskcontent))
            self.refresh()

    def deleteTask(self,event):
        """
            Deletes a task from the current menu. (Does not delete the file)
        """
        l=self.getSelectedKeys()
        for k in l:
            del self.tasker.tasks[k]            
        self.refresh()
        
    def executeTask(self):
        """
            Executes selected tasks
        """
        l=self.getSelectedKeys()
        for k in l:
            self.tasker.post.perform(k)
        
    def exit(self,event=None):
        """
            Stops the robots movement and 
        """
        rospy.loginfo("exit")
        self.baxter.hlm.stop(True)
        self.master.destroy()
        sys.exit()

    def saveTasks(self,event=None):
        """
            Saves all tasks
        """
        options={}
        for k,v in self.tasker.tasks.items():
            open(self.baxter.datapath + "tasks"+os.sep+k+DEFAULT_EXTENSION,"w").write(v.toeval)

    def updateTask(self,event=None):
        """
            Updates the tasks list and colorizes the python code
        """
        if self.selected:
            self.tasker.tasks[self.selected].toeval=self.textTask.get(1.0,END)
            self.textTask.colorize()
            
    def loadTaskFile(self,file):
        """
            Loads the task file that corresponds to the path from the harddisk into the GUI
            
            :param file:path and name to the task file
            :type file:str
        """ 
        taskcode=open(file,"r").read()
        toks=file.split('/')
        name=toks[-1]
        name=name.replace(DEFAULT_EXTENSION,"")
        if name in self.tasker.tasks.keys():
            del self.tasker.tasks[name]
        self.tasker.add(name,baxter_helper_tasker.TaskEval(self.tasker,taskcode) )
        self.listTasks.insert(END, name)
        self.refresh()
        
    def loadAllTasks(self,event=None):
        """
            Loads all tasks in the data/tasks directory of the package
        """
        for file in glob.glob(self.baxter.datapath + "tasks"+os.sep+"*.task.py"):
            self.loadTaskFile(file)

    def loadTask(self,event=None):
        """
            Callback from the menu to load a task. Opens a file dialog
        """
        options={}
        options['defaultextension'] = DEFAULT_EXTENSION
        options['filetypes'] = [('task files', DEFAULT_EXTENSION)]
        file=tkFileDialog.askopenfilename(**options)
        if file:
            self.loadTaskFile(file)

    def getSelectedKeys(self):
        """
            Returns the selected tasks
        """
        l=[]
        items = map(int, self.listTasks.curselection())
        for i in items:
            v=self.listTasks.get(i)
            k=v.strip()
            l.append(k)
        return l
    

def startGUI():
    """
        Starts the main baxter_tasker
    """
    rospy.init_node("baxter_tasker_main")
#     rospy.init_node("baxter_tasker_main",log_level=rospy.DEBUG) # start baxter with debug output
    baxter=BaxterRobot(False) # in baxter_helper.py
    baxter.loadAll()
    tasker=baxter_helper_tasker.Tasker(baxter)
    baxter.addTasker(tasker)
    root = Tk()
    root.title("baxter_tasker_main")
    
    mainwindow = MainWindow(root,tasker)
    mainwindow.loadAllTasks()
    
    root.protocol("WM_DELETE_WINDOW", mainwindow.exit)
    root.mainloop()

if __name__=="__main__":
    startGUI()

