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
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw 

class ImageComposerElement:
    """
        Defines an image element that can be combined
    """
    def __init__(self,imgfile,xpos=0,ypos=0,layer=0,alpha=True):
        """
            :param imgfile: Path to the image
            :type imgfile: str
            :param xpos: x offset to the background, where the element is be drawn
            :type xpos: int
            :param ypos: y offset to the background, where the element is be drawn
            :type ypos: int
            :param layer: in which layer the element is drawn
            :type layer: int
            :param alpha:
            :type alpha: bool
        """
        self.imgfile=imgfile
        self.xpos=xpos
        self.ypos=ypos
        self.alpha=alpha
        self.layer=layer
        
        self.buffer=Image.open(self.imgfile)
        #~ print "loading "+self.imgfile
        #~ print self.buffer.mode
    
    def dispose(self):
        """
            Clear the buffer 
        """
        if self.buffer:
            del self.buffer    
        
    def drawOnto(self,buffer):
        """
             Add another element onto the current
             
             :param buffer: Element to add
             :type buffer: PIL.Image
        """
        if self.alpha:
            buffer.paste(self.buffer,(self.xpos,self.ypos),mask=self.buffer)
        else:
            buffer.paste(self.buffer,(self.xpos,self.ypos))
        
        
class ImageComposer:    
    """
        The image composer combines multiple images into one,
        to create interactive interfaces for the robot.
    """
    def __init__(self,bgcolor="white",width=1024,height=600):
        self.bgcolor=bgcolor
        
        self.display_width=width
        self.display_height=height
        self.elements={}
        self.buffer= Image.new("RGB", (self.display_width,self.display_height), self.bgcolor)
        self.draw = ImageDraw.Draw(self.buffer)
            
    def resetbg(self):
        """
            Draws the background
        """
        self.draw.rectangle( ((0,0),(self.display_width,self.display_height)) ,fill=self.bgcolor,outline=self.bgcolor)

        

    def clear(self):
        """
            Deletes all image elements
        """
        for e in self.elements.values():
            e.dispose()
        self.elements={}
        
            
    def dispose(self):
        """
            Deletes the current image element
        """
        del self.buffer


    def render(self):
        """
            Resets the background and draws all added element into the image
        """
        self.resetbg()
        elements=[v for v in self.elements.values() if v!=None]
        elements.sort(key=lambda x:x.layer)
        for e in elements:
            e.drawOnto(self.buffer)
            
    def save(self,imgfile):
        """
            Saves the created image to a file
            
            :param imgfile: Filename to save
            :type imgfile: str
        """
        self.buffer.save(imgfile)

    def set(self,label,element):
        """
            Adds a new element to the image to be combined
            
            :param label: Name of the new element
            :type label:  str
            :param element: Element to be drawn into the image (see example code)
            :type element: ImageComposerElement
        """
        self.elements[label]=element
        
    def unset(self,label):    
        """
            Deletes an element
            
            :param label: Element's name to be deleted
            :type label: str
        """
        del self.elements[label]
        

if __name__=="__main__":
    comp=ImageComposer()
    comp.set("background",ImageComposerElement("../data/starting.png")) # default layer=0
    comp.set("drawingblack",ImageComposerElement("../data/test-alpha.png",300,300,layer=1))
    comp.set("drawinggreen",ImageComposerElement("../data/test-alpha2.png",350,300,layer=2))
    comp.render()
    comp.save("tmp.png")
    
    