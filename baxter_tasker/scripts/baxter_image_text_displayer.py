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

import rospy

class ImageTextDisplayer:    
    """
        This class draws text inside an image to log terminal information
        onto the screen of baxter for fast debug sessions.
    """
    def __init__(self,fontfile):
        """
            :param fontfile: Path to the font to be used for the printed text
            :type fontfile: str
        """
        self.display_width=1024
        self.display_height=600
        self.char_width=8
        self.char_height=16
        self.text_width=self.display_width/self.char_width
        self.text_height=self.display_height/self.char_height
        
        self.font = ImageFont.truetype(fontfile, self.char_height)
        self.line_spacing=self.char_height
        
    def drawText(self,imgfile,text):        
        """
            Draws the text inside an image
            
            :param imgfile: Path where the image with the written text should be saved
            :type imgfile: str
            :param text: Text that should be writen into the file
            :type text: str
        """
        self.img = Image.new("RGB", (self.display_width,self.display_height), "black")
        self.draw = ImageDraw.Draw(self.img)
        lines=text.split("\n")
        splittedlines=[]
        for line in lines:            
            while not rospy.is_shutdown() and len(line)>self.text_width:
                l1=line[:self.text_width]
                splittedlines.append(l1)
                line=line[self.text_width:]
            splittedlines.append(line)

        if len(splittedlines)>self.text_height:
            extra=len(splittedlines)-self.text_height-1
            splittedlines=splittedlines[extra:]
            
        ypos=0
        for line in splittedlines:                
            self.draw.text((0, ypos),line,(255,255,255),font=self.font)
            ypos+=self.line_spacing
            
        self.img.save(imgfile)


if __name__=="__main__":
    fontfile="../data/font/UbuntuMono-R.ttf"
    td=ImageTextDisplayer(fontfile)
    text="""Hello guys
    What's up
0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789
    bla
    bli
    bla
    bli
    bla
    bli
    bla
    bli
    bla
    bli
    bla
    bli
    """
    td.drawText("sample-out.jpg",text)
    #~ td.drawText("sample-out.jpg","SQDFQSDFQSDF")
    