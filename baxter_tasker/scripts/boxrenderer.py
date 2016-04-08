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

import time

from baxter_image_composer import ImageComposer,ImageComposerElement

class AbstractBoxRenderer:
    def __init__(self,bgfile,switchfile,markfile,notpickedfile,nothandedfile):
        self.bgfile=bgfile
        self.switchfile=switchfile
        self.markfile=markfile
        self.notpickedfile=notpickedfile
        self.nothandedfile=nothandedfile

        bg=Image.open(bgfile)
        self.comp=ImageComposer("black",*bg.size)
        self.comp.set("background",ImageComposerElement(self.bgfile,layer=0))

    def  clear(self):
        self.comp.clear()
        self.comp.set("background",ImageComposerElement(self.bgfile,layer=0))
        

    def  addItem(self,number):
        coord=self.getCoord(number)
        self.comp.set(str(number),ImageComposerElement(self.switchfile,*coord,layer=1))

    def setMark(self,number):
        coord=self.getCoord(number)
        self.comp.set("mark",ImageComposerElement(self.markfile,*coord,layer=3))

    def setNotPicked(self,number):
        coord=self.getCoord(number)
        self.comp.set("notpicked"+str(number),ImageComposerElement(self.notpickedfile,*coord,layer=2))
        
    def setNotHanded(self,number):
        coord=self.getCoord(number)
        self.comp.set("notpicked"+str(number),ImageComposerElement(self.nothandedfile,*coord,layer=2))
        
    def unsetMark(self):
        self.comp.unset("mark")

    def removeItem(self,number):        
        try:
            self.comp.unset(str(number))
        except:
            pass

    def save(self,file):
        self.comp.render()
        self.comp.save(file)

    def getItems(self):
        items=range(7*4)
        items.remove(9)
        items.remove(11)
        return items
        
    def fill(self):
        self.clear()        
        items=self.getItems()
        for i in items:
            self.addItem(i)
            

class BoxRenderer1(AbstractBoxRenderer):
    def __init__(self,*args):
        AbstractBoxRenderer.__init__(self,*args)
        
    def getCoord(self,number):
        xoff=90
        yoff=-100
        zeroX=215
        zeroY=405     
        
        x=number%7
        y=int(number/7)
        return (zeroX+x*xoff,zeroY+y*yoff)

    def getItems(self):
        items=range(7*4)
        items.remove(9)
        items.remove(11)
        return items

class BoxRenderer3(AbstractBoxRenderer):
    def __init__(self,*args):
        AbstractBoxRenderer.__init__(self,*args)
        
    def getCoord(self,number):
        xoff=0
        yoff=72
        zeroX=184
        zeroY=95     
        
        x=0
        y=int(number)
        return (zeroX+x*xoff,zeroY+y*yoff)

    def getItems(self):
        return range(6)
        


if __name__=="__main__":

    br=BoxRenderer1("../data/box1.png","../data/element1.png","../data/mark1.png","../data/notpicked1.png","../data/nothanded1.png")
    i=0

    br.addItem(0)
    br.save("tmp1-%d.png"%i);    i+=1

    br.addItem(1)
    br.save("tmp1-%d.png"%i);    i+=1

    br.setMark(1)
    br.save("tmp1-%d.png"%i);    i+=1

    br.setNotPicked(20)
    br.setNotHanded(21)
    br.save("tmp1-%d.png"%i);    i+=1

    br.clear()
    br.save("tmp1-%d.png"%i);    i+=1

    br.fill()
    br.save("tmp1-%d.png"%i);    i+=1

    br.removeItem(10)
    br.save("tmp1-%d.png"%i);    i+=1



    br=BoxRenderer3("../data/box3.png","../data/element3.png","../data/mark3.png","../data/notpicked3.png","../data/nothanded3.png")
    i=0

    br.addItem(0)
    br.save("tmp3-%d.png"%i);    i+=1

    br.addItem(1)
    br.save("tmp3-%d.png"%i)
    i+=1

    br.fill()
    br.save("tmp3-%d.png"%i)
    i+=1

    br.setMark(2)
    br.save("tmp3-%d.png"%i)
    i+=1

