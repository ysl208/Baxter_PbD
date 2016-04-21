#!/usr/bin/env python

#####################################################################################
#                                                                                   #
# Copyright (c) 2016                                                                #
# All rights reserved.                                                              #
#                                                                                   #
# Author: Ying Siu Liang                                                            #
# Team MAGMA, LIG                                                                   #
#                                                                                   #
#####################################################################################

class TetrisBlock:
    def __init__(self, letter, colour, min_colour_range, max_colour_range,):
        self.letter = letter
        self.colour = colour
        self.center = (0,0)
        self.area = 34565
        self.max_colour_range = max_colour_range
        self.min_colour_range = min_colour_range

    def getMaxColourRange(self):
        return self.max_colour_range

    def getMinColourRange(self):
        return self.min_colour_range

    def getColour(self):
        return self.colour

    def setColour(self, colour):
        self.colour = colour

    def getCenter(self):
        return self.center

    def setCenter(self, center):
        self.center = center

    def getArea(self):
        return self.area

    def setArea(self, area):
        """
            Calculates the area in pixels depending on the height of the camera
        """
        self.area = area

