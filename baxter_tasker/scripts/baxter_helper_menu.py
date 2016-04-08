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
import PIL
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
import math
class BaxterMenuComposer:
    """
        Creates the image that is shown on the display of the robot and on the local workstation
    """    
    def __init__(self,display,textDisplay,datapath):
        """
            :param display: object from BaxterDisplay to send images to the robot display
            :type display: BaxterDisplay
            :param textDisplay: object from BaxterTextDisplayer to enable and disable the autorefresh
            :type textDisplay: BaxterTextDisplayer
            :param datapath: path for datafiles
            :type datapath: str 
        """
        self.entries = {}
        self.display = display
        self.textDisplay = textDisplay
        self.img = None
        self.tmp_menu_path = str(datapath) + "menu.jpg"
        self.display_offset = 150
        self.display_width=1024
        self.display_height=600
        self.char_width=16
        self.char_height=32
        self.text_width=self.display_width/self.char_width
        self.text_height=self.display_height/self.char_height
        
        self.font = PIL.ImageFont.truetype(datapath+"font/UbuntuMono-R.ttf", self.char_height)
        self.font_title = PIL.ImageFont.truetype(datapath+"font/UbuntuMono-R.ttf", 128)
        self.line_spacing=self.char_height
        self.title = ""
        self.cols = 0
        self.rows = 0
        self.selection = {}
        self.hidden = True
        
    def loadEntries(self,title,entries):
        """
            Preloads a new menu
            
            :param title: The text in the title bar of the menu
            :type title: str
            :param entries: A list of the button names
            :type entries: list(str)
        """
        self.entries = entries
        self.title = title
        [rows, cols] = self.__computeDimensions(len(self.entries))
        self.cols = cols
        self.rows = rows
        self.x_spacing = self.display_width/self.cols
        self.y_spacing = (self.display_height-self.display_offset)/self.rows
        
    def __computeDimensions(self,num_entries):
        """
            Computes the dimensions (rows and cols) of the buttons based on the number of entries
            
            :param num_entries: Number of menu entries
            :type num_entries: int
            :return: list of number of rows and columns
            :rtype: [int,int]
        """
        tmp = math.sqrt(num_entries)
        tmp2 = int(tmp)
        if tmp==tmp2:
            return [tmp2,tmp2]
        else:
            return [[tmp2, tmp2+1][num_entries>tmp2*tmp2+tmp2],tmp2+1]
        
    def show(self):
        """
            Sends the last menu to the robot
        """
        if not self.textDisplay is None and self.textDisplay.autorefresh is True:
            self.textDisplay.autorefresh = False
            rospy.sleep(1.5)
        if not self.img is None:
            self.__saveImage()
            if not self.display is None:       
                self.display.setImage(self.tmp_menu_path)
                self.hidden = False
                
    def hide(self,update=True):
        """
            Hides the GUI
            
            :param update: If True, it enables the text display
            :type update: bool 
        """
        if self.hidden:
            return
        if not self.textDisplay is None and self.textDisplay.autorefresh is False:
            if update is True:
                self.textDisplay.post.autoRefresh(1)
        self.hidden = True
    
    def __computeRectangle(self,entry):
        """
            Computes the corners of the rectangle of the selected element
            
            :param entry: Element that corners should be computed
            :type entry: str
            :return: tuple of two point tuples e.g.((p1x,p1y),(p2x,p2y))
            :rtype: tuple(tuple(int,int),tuple(int,int))
        """
        [x,y] = self.selection[entry]
        p1x = x - self.x_spacing/2 + self.x_spacing/10
        p1y = y - self.y_spacing/2 + self.y_spacing/10
        p2x = x + self.x_spacing/2 - self.x_spacing/10
        p2y = y + self.y_spacing/2 - self.y_spacing/10
        return ((p1x,p1y),(p2x,p2y))
        
    def __addTitle(self):
        """
            Draws the title inside the image
        """
        x_pos = self.x_spacing/10
        y_pos = self.display_offset/2
        self.draw.text((x_pos, y_pos),self.title,(255,255,255),font=self.font)
        
    def __drawMenuEntries(self): 
        """
            Creates a new image containing all previously defined entries
        """       
        self.img = PIL.Image.new("RGB", (self.display_width,self.display_height), "#313133")
        self.draw = PIL.ImageDraw.Draw(self.img)
        x_pos = self.x_spacing/2
        y_pos = self.y_spacing/2+self.display_offset
        
        for entry in self.entries:
            chars = len(entry)
            self.selection[entry] = [x_pos,y_pos] 
            self.__drawBackground(entry)
            button_width = self.x_spacing*4/5
            if len(entry)*self.char_width < button_width:
                self.draw.text((x_pos-chars*self.char_width/2, y_pos-self.char_height/2),entry,(0,0,0),font=self.font)
            else:
                split_entries = entry.split(" ")
                i = int(len(split_entries)/-2)
                for split_entry in split_entries:
#                     print "new y pos",y_pos+i*self.char_height, "old y pos", y_pos, "offset", i*self.char_height
                    self.draw.text((x_pos-len(split_entry)*self.char_width/2, y_pos+i*self.char_height),split_entry,(0,0,0),font=self.font)
                    i+=1
            x_pos +=self.x_spacing
            if x_pos > self.display_width:
                x_pos = self.x_spacing/2
                y_pos +=self.y_spacing
        return self.draw

    def __drawBackground(self,entry):
        """
            Draws a filled rectangle into the image (corresponds to a button)
        """
        self.draw.rectangle(self.__computeRectangle(entry), fill= '#c45221')
        
    def resetSelection(self):
        """
            Resets the selected element
        """
        self.__drawMenuEntries()
        self.__addTitle()
        
    def select(self,entry):
        """
            Draws a rectangle around an element to mark it as the active element
            
            :param entry: Element that has to be marked with a green rectangle 
            :type entry: str
        """
        self.resetSelection()
        coords = self.__computeRectangle(entry)
        self.draw.rectangle(coords, outline="#00ff00")
        self.draw.rectangle(((coords[0][0]-2,coords[0][1]-2),(coords[1][0]+2,coords[1][1]+2)), outline="#00ff00")
        self.draw.rectangle(((coords[0][0]-1,coords[0][1]-1),(coords[1][0]+1,coords[1][1]+1)), outline="#00ff00")
        self.show()
        
    def __saveImage(self):
        """
            Saves the current created image object to a file
        """
        self.img.save(self.tmp_menu_path)

import collections
from inspect import getmembers
class BaxterMenuManager():
    """
        Manages the menus behind the GUI.
    """
    def __init__(self,composer,moveit=False):
        """
            :param composer: object to draw and select new entries
            :type composer: BaxterMenuComposer
            :param moveit: Flag if moveit is used to have different options for moveit
            :type moveit: bool  
        """
        self.composer= composer
        self.moveit = moveit
        self.page = {}
        self.inactive_pages = {}
        self.cur_page = None
        self.cur_mode = 0
        self.me = None
        self.modes = []
        self.handlers = collections.defaultdict(set)
        self.default_values = {}
        self.cur_box = None
        
    def addBaxterBehavior(self,bb):
        """
            Adds a BaxterBehavior class that all methods in the added class can be called
            from the menu
            
            :param bb: Custom class that contains specific methods used by the menu 
            :type bb: BaxterBehaviors
        """
        self.bb = bb
        self.me = BaxterMenuEntries(self,self.moveit) #add static menus
        
    def __register(self, event, callback, default=None):
        """
            Add a new entry, its callback and a default value to the available handlers
        """
        self.handlers[event].add(callback)
        #print "handlers",self.handlers
        self.default_values[event] = default 
        #print "default values",self.default_values 
         
    def __unregister(self,event):
        """
            Removes an entry from the handlers
        """
        self.handlers[event].remove()
        del self.default_values[event]
         
        
    def callMenuEntry(self,name,**kwargs):
        """
            Calls a menu entry by its name
        """
        for handler in self.handlers.get(name, []):
            #print "name",name,"handler",handler
            handler(**kwargs)
    
    def addGenericMenu(self,name,parent,title,entries):
        """
            Adds a freshly created menu to the availble menu pages
            
            :param name: Name of the menu page
            :type name: str
            :param parent: Name of the parent menu
            :type parent: str
            :param title: Text in the title bar of the menu
            :type title: str
            :param entries: Entries in a dictionary with their name and callback
            :type entries: dict({str:callback}) or dict({str:list(callback,default_value)}) 
        """
#         print "ADD GENERIC MENU"
        #print self.inactive_pages.keys()
        self.inactive_pages[name] = [parent,title,entries]
#        print entries
        
    def __loadRegister(self,entries):
        """
            Loads the callable entries
        """
        for entry,cb in entries.iteritems():
            if type(cb)==list:
                self.__register(entry,cb[0],cb[1])
            else:
                self.__register(entry,cb)
                                 
    def __unloadRegister(self,entries):
        """
            Unloads callable entries
        """
        #print "UNLOAD REGISTER"
        #print entries
        for entry in entries.keys():
            del self.handlers[entry]
        #print "REMAINING HANDLERS"
        #print self.handlers
    
    def __unloadMenu(self):
        """
            Unloads a menu and stores the current page in inactive pages to avoid name clashes
        """
        if self.cur_page != None:
            #print "current page", self.cur_page
            #print self.page.keys()
            self.inactive_pages[self.cur_page] = self.page[self.cur_page]
            self.__unloadRegister(self.page[self.cur_page][2])
            del self.page[self.cur_page]
        
    def changeMenuTitle(self,title):
        """
            Changes the title text of the current menu
            
            :param title: New text that should be display in the title bar
            :type title: str 
        """
        self.page[self.cur_page][1] = title
        self.composer.loadEntries(title,self.modes)
        self.composer.select(self.modes[self.cur_mode])
        
    def addEntriesToCurrentMenu(self,entries):
        """
            Adds additional entries to the current menu
            
            :param entries: Entry with name and callback to add to the current menu.
            :type entries: dict({str:callback}) or dict({str:list(callback,default_value)}) 
             
            The callback could also be a list of the callback and a custom default value that is saved to this entry
        """
        rospy.loginfo("adding additional menu entries")
        for name,callback in entries.iteritems():
            self.page[self.cur_page][2][name] = callback
        self.__loadRegister(self.page[self.cur_page][2])
        self.modes = sorted(self.page[self.cur_page][2].keys())
        self.composer.loadEntries(self.page[self.cur_page][1],self.modes)
        self.composer.select(self.modes[self.cur_mode])

    def removeEntriesFromCurrentMenu(self,names):
        """
            Removes entries form the current menu
            
            :param names: Name(s) of the entries to remove from the current menu
            :type names: list or str
            :attention: Do not remove the last element!
        """
#         print "removing  menu entries", names
        if type(names) is str:
            names = [names]
#         print self.page[self.cur_page][2].keys()
        for name in names:
#             print name
            if name in self.page[self.cur_page][2].keys():
#                 print "deleting page: ", name
                self.__unloadRegister({name:self.page[self.cur_page][2][name]})
                del self.page[self.cur_page][2][name]
#         print self.page[self.cur_page][2].keys()
        title = self.page[self.cur_page][1]
        self.cur_mode = 0
        self.modes = sorted(self.page[self.cur_page][2].keys())
        self.composer.loadEntries(title,self.modes)
        self.composer.select(self.modes[0])#not totally safe, in case there are not elements in the menu. this would require to load some other page (previous?)
    
    def loadMenu(self,name):
        """
            Unloads the current menu, saves, loads the new menu options and displays it
            
            :param name: New menu to load
            :type name: str 
        """
        #print "loadMenu:",name, "cur menu:",self.cur_page
        if name == self.cur_page:
            #print "no sense to load the active menu. skipping"
            return
#         print "inactive pages are:",self.inactive_pages.keys()
        self.substep = 0
        self.cur_mode = 0
        #load new page if it is not in pages
        if not name is None:
            self.page[name] = self.inactive_pages[name]
            self.__loadRegister(self.inactive_pages[name][2])
            del self.inactive_pages[name]
        # load new modes
        self.modes = sorted(self.page[name][2].keys())
#         print "modes",self.modes
        #print "New menu named",name, "called from",self.cur_page, "has modes:",self.modes,
        if len(self.modes) == 0:
            self.emptyMenu()
            rospy.logwarn("Warning: Page to load was empty. Reloading current page.")
            return False
        #store current page in inactive menu
        self.__unloadMenu()
        #print "active pages are",self.page.keys()
        #print "active modes are", self.modes
        self.parent = self.page[name][0]
        self.cur_page = name
        title = self.page[name][1]
        self.composer.loadEntries(title,self.modes)
        self.composer.select(self.modes[0])
        return True
    
    def loadDefaultMenu(self,**kwargs):
        """
            Loads the menu given in the default value 
        """ 
        #print "load menu with kwargs",kwargs
        try:
            menu_to_load = self.default_values[self.modes[self.cur_mode]]
            if not menu_to_load is None:
                self.loadMenu(menu_to_load)
            else:
                rospy.logwarn("default value was %s"%str(menu_to_load))
        except Exception,e:
            rospy.logerr("could not load default menu %s"%str(e))
        
    def loadPreviousMenu(self):
        """
            Loads the previous menu
        """
        self.substep = 0
        parent = self.page[self.cur_page][0]
        #print "loading previous menu %s, from current menu %s"%(parent,self.cur_page)
        if not parent is None:
            self.loadMenu(parent)
        else:
            self.loadMenu("main")
            
    def confirm(self):
        """
            Shows a confirmation message on the screen and reloads the current menu
        """
        self.addConfirmation()
        self.loadMenu("confirmation")
        rospy.sleep(1)
        self.loadPreviousMenu()
        
    def neglect(self):
        """
            Shows a failure message on the screen and reloads the current menu 
        """
        self.addNeglection()
        self.loadMenu("neglection")
        rospy.sleep(3)
        self.loadPreviousMenu()
    
    def emptyMenu(self):
        """
            Loads an empty menu
        """
        self.addEmpty()
        self.loadMenu("empty")
        rospy.sleep(1.5)
        self.loadPreviousMenu()
        
    def addEmpty(self):
        entries = {'Nothing':self.bb.nothing}
        self.addGenericMenu("empty", self.cur_page,"This menu is empty",entries)
    
    def addConfirmation(self):
        entries = {'Input confirmed':self.bb.nothing}
        self.addGenericMenu("confirmation", self.cur_page,"Success",entries)
        
    def addNeglection(self):
        entries = {'FAIL - There was a problem':self.bb.nothing}
        self.addGenericMenu("neglection", self.cur_page,"Fail",entries)
            
    def addGenericFunction(self,name,func):
        """
            This could link a string to a specific function
            
            :param name: New name of the methods
            :type name: str
            :param func: Existing methods or callback
            :type func: callback  
        """
        try:
            setattr(self, name, func)
        except Exception,e:
            rospy.logerr("could not set generic function with name %s, error: %s"%(name,str(e)))
        
    def __getMenuList(self):
        """
            Gets the list of all not private members/methods in BaxterMenuEntries
            
            :return: List of all methods
            :rtype: list
        """
        if self.me is None:
            rospy.logwarn("Menu entries are not loaded yet")
            return None
        menus = getmembers(self.me)
        menus=[m[0] for m in menus if not m[0].startswith("_")]
        return menus
              
    def addAllMenus(self):
        """
            Adds all static menus from BaxterMenuEntries
        """
        menus = self.__getMenuList()
        if menus is None:
            rospy.logwarn("List of menus to load is empty")
            return
        [getattr(self.me,m)() for m in menus]
        
        
        
class BaxterMenuEntries:
    """
        This class contains all fixed menus that are not changing during the execution. 
        Therefore they can be already defined beforehand
    """
    def __init__(self,mm,moveit):
        self.__mm = mm
        self.__moveit = moveit
    
############## ADD YOU DEFAULT MENU PAGES HERE
# each menu consists of:
# dict0: dictionary of your actual buttons in your menu
#    string0: name of the button
#    list/method1: list or method that should be called
#        method0: method to be called
#        string1: default value
# To add the entries to a generic menu, you have to define:
# string0: menu name
# string1: parent menu
# string2: menu title description
# dict3: entries
# 
# e.g.:
    def addMain(self):
        """
            Creates the main menu which is very important as it is loaded on startup
        """
        entries = {
                   'Teach Menu': [self.__mm.loadDefaultMenu,"teachMenu"],
#                   'Run Scenarios': self.__mm.bb.run,
#                    'Select Arms':self.__mm.bb.selectArms,
#                   'Show Camera': self.__mm.bb.showCamera,
#                   'Go To Init':self.__mm.bb.goToInit,
                   
                   } 
        if self.__moveit:
            entries['Manage Boxes']=[self.__mm.loadDefaultMenu,"addBox"]
            entries['Select Gripper']=self.__mm.bb.gripperType
        self.__mm.addGenericMenu("main", None,"Baxter Main Menu",entries)
        
    def addTeachMenu(self):
        """
            Creates the menu to teach simple trajectories
            Entries can be passed with their default values which will be saved in self.mm.default_values
        """
        entries = {
                   "1. Enter Parameters": self.__mm.bb.enterParameters,
                   "2. Enter Preconditions": [self.__mm.bb.enterPredicates,True],
                   "3. Enter Effects": [self.__mm.bb.enterPredicates,False],
                   "4. Demo Action": self.__mm.bb.demonstrate,
                   "5. Execute Action": self.__mm.bb.execute,
                   "6. Save Predicates": self.__mm.bb.savePredicates,
#                   "Learn": self.__mm.bb.teach,
#                   "Play" : [self.__mm.bb.executePath,False],
#                   "Play Looped": [self.__mm.bb.executePath,True],
                   }
        self.__mm.addGenericMenu("teachMenu","main","Learn a new operator by demonstrating an action", entries)
        
    def addBox(self):
        """
            Creates the menu to setup predefined boxes
        """
        entries = {
                    "Select Box Type":self.__mm.bb.selectBoxType,
                    "Done":self.__mm.bb.computeAndInsertBox,
                    }
        for i in range(0,2):
            entries["Point "+str(i)] = self.__mm.bb.addFramePoint # !!dont change the name of this menu entry!!
        self.__mm.addGenericMenu("addBox","main","Select 2 points, a box type and confirm", entries)

    def __someFunction(self):
        """
            If you need additional functions here, declare them private by two preceding underscores ("__"),
            otherwise it will be parsed as menu.
        """
        #if you need additional methods then start the name by a preceding "__" and it will be ignore by the parser that loads the menus
        pass
    

if __name__ == '__main__':  
    rospy.init_node('MenuComposerDemo')
    datapath=roslib.packages.get_pkg_dir('baxter_tasker') + "/data/"
    menu = BaxterMenuComposer(None, None,datapath)
    
    menu.loadEntries(title="Enter your title here",entries=["Function A","Function B"])
    menu.select("Function A")
