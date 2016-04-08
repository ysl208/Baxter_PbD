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
from Tkinter import *

import keyword

class PythonText(Text):
    """ Tk Text Widget that supports some Python syntax coloring"""

    def __init__(self, *args, **kwargs):
        Text.__init__(self, *args, **kwargs)
        self.patterns={}
        self.bind("<KeyRelease>",self.colorize)

        self.createTags()

        self.addPattern("#.*","comment")
        self.addPattern("'[^']*'","string")
        self.addPattern('"[^"]*"',"string")
        keywords=keyword.kwlist
        #~ for k in keywords:
            #~ self.addPattern("\w%s"%k,"normal")
        for k in keywords:
            self.addPattern("^%s(?!\\w)"%k,"keyword")
        for k in keywords:
            self.addPattern("[ \t]%s(?!\\w)"%k,"keyword")
        
        
    def createTags(self):
        self.tag_configure("normal",foreground="#000000")
        self.tag_configure("comment",foreground="#007f00")
        self.tag_configure("keyword",foreground="#0000Af")
        self.tag_configure("string",foreground="#7f007f")
        self.tag_lower("keyword")
        self.tag_raise("comment")

    def deleteTags(self):
        self.tag_delete("normal")
        self.tag_delete("comment")
        self.tag_delete("keyword")
        self.tag_delete("string")

    
    def addPattern(self,regexp,tag):
        self.patterns[regexp]=tag
    
    def colorize(self,*args,**kwargs):        
        self.deleteTags()
        self.createTags()
        for regexp in self.patterns.keys():
            self.highlight_pattern(regexp,self.patterns[regexp],regexp=True)
        
    def highlight_pattern(self, pattern, tag, start="1.0", end="end", regexp=False):
        
        '''Apply the given tag to all text that matches the given pattern

        If 'regexp' is set to True, pattern will be treated as a regular expression
        '''

        start = self.index(start)
        end = self.index(end)
        self.mark_set("matchStart",start)
        self.mark_set("matchEnd",start)
        self.mark_set("searchLimit", end)

        count = IntVar()
        while True:
            index = self.search(pattern, "matchEnd","searchLimit",
                                count=count, regexp=regexp)
            if index == "": break
            self.mark_set("matchStart", index)
            self.mark_set("matchEnd", "%s+%sc" % (index,count.get()))
            self.tag_add(tag, "matchStart","matchEnd")
            
if __name__=="__main__":

    class PythonWindow:

        def __init__(self, master):
            self.master=master
            
            self.defaultCode="""
# Use the 'chain' object to access motors
# Here is an example that assumes a motor on ID 1

id=1

chain.goto(id,0,speed=0) # Full speed to pos 0
chain.goto(id,1000,speed=100) # Low speed to pos 1000
chain.goto(id,500,blocking=False) # Current speed to pos 500
while chain.is_moving():
    print chain.get_reg_si(id,"present_position")
chain.goto(id,100,speed=0) # Full speed back to pos 100   
            """
            
            
            self.frame=Frame(self.master)
            
            self.pythonFrame=LabelFrame(self.frame,text="Python code")
            self.textTask=PythonText(self.pythonFrame,width=60,height=30)
            self.textTask.pack()
            self.pythonFrame.grid(row=0,column=0)
            self.textTask.insert(END,self.defaultCode)
            self.textTask.colorize()
            
            self.frame.pack()
            
            
        

    root = Tk()
    appname="DynamixelLab"
    root.title(appname)
    mainwindow = PythonWindow(root)
    #~ root.protocol("WM_DELETE_WINDOW", mainwindow.destroy)
    root.mainloop()

    