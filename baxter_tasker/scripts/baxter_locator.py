#!/usr/bin/env python

#####################################################################################
#                                                                                   #
# Copyright (c) 2014, Active Robots Ltd.                                            #
# All rights reserved.                                                              #
#                                                                                   #
# Redistribution and use in source and binary forms, with or without                #
# modification, are permitted provided that the following conditions are met:       #
#                                                                                   #
# 1. Redistributions of source code must retain the above copyright notice,         #
#    this list of conditions and the following disclaimer.                          #
# 2. Redistributions in binary form must reproduce the above copyright              #
#    notice, this list of conditions and the following disclaimer in the            #
#    documentation and/or other materials provided with the distribution.           #
# 3. Neither the name of the Active Robots nor the names of its contributors        #
#    may be used to endorse or promote products derived from this software          #
#    without specific prior written permission.                                     #
#                                                                                   #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"       #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE         #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE        #
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE          #
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR               #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF              #
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS          #
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN           #
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)           #
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        #
# POSSIBILITY OF SUCH DAMAGE.                                                       #
#                                                                                   #
#####################################################################################
import pdb

import rospy
import roslib

import cv;
import cv2;
import cv_bridge

import numpy 
import math
import os
import sys
import string
import time
import random
import tf
from sensor_msgs.msg import Image
import baxter_interface
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

# initialise ros node
#rospy.init_node("pick_and_place", anonymous = True)

# directory used to save analysis images
image_directory = ""

class TetrisBlock:
    def __init__(self):
        self.letter = "O"
        self.colour = "yellow"
        self.center = (0,0)
        self.area = 34565
        self.colour_range = [[[25, 100, 110], [135, 250, 250]]]

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


# locate class
class BaxterLocator:
    def __init__(self, baxter):
        # arm ("left" or "right")
        arm = "right"
        distance = 0.367
        self.limb           = arm
        self.limb_interface = baxter_interface.Limb(self.limb)
        self.baxter = baxter
        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)

        # image directory
        self.image_dir = os.path.dirname(os.path.abspath(__file__)) + "/"

        # flag to control saving of analysis images
        self.save_images = True
        self.publish_camera = True

        # required position accuracy in metres
        self.ball_tolerance = 0.02
        self.tray_tolerance = 0.02

        # number of balls found
        self.balls_found = 0

        # start positions
        self.ball_tray_x = 0.60                        # x     = front back
        self.ball_tray_y = -0.08                        # y     = left right
        self.ball_tray_z = 0.03                        # z     = up down
        self.golf_ball_x = 0.50                        # x     = front back
        self.golf_ball_y = -0.08                        # y     = left right
        self.golf_ball_z = 0.05                      # z     = up down
        self.roll        = -1.0 * math.pi              # roll  = horizontal
        self.pitch       = 0.0 * math.pi               # pitch = vertical
        self.yaw         = 0.0 * math.pi               # yaw   = rotation

        self.pose = [self.ball_tray_x, self.ball_tray_y, self.ball_tray_z,     \
                     self.roll, self.pitch, self.yaw]

        # camera parameters (NB. other parameters in open_camera)
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.039                      # camera gripper offset
        self.cam_y_offset = -0.02
        self.width        = 960                        # Camera resolution
        self.height       = 600

        # Hough circle accumulator threshold and minimum radius.
        self.hough_accumulator = 35
        self.hough_min_radius  = 15
        self.hough_max_radius  = 35

        # canny image
        self.canny = cv.CreateImage((self.width, self.height), 8, 1)

        # Canny transform parameters
        self.canny_low  = 45
        self.canny_high = 150

        self.colour_boundaries = {
                                  'red': [[[17, 15, 80], [90, 76, 200]]], 
                                  'blue': [[[63, 31, 4], [220, 88, 50]]],
                                  'green': [[[22, 73, 29], [53, 223, 60]]],
                                  'yellow': [[[25, 80, 85], [135, 250, 250]]],
                                  'gray': [[[103, 86, 65], [145, 133, 128]]]
        }

        # minimum qr code area
        self.min_area = 1000

        # callback image
        self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)

        # colours
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

        # ball tray corners - in Baxter coordinates
        self.ball_tray_corner = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]

        # ball tray places
        self.ball_tray_place = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]

        # Enable the actuators
        baxter_interface.RobotEnable().enable()

        # set speed as a ratio of maximum speed
        self.limb_interface.set_joint_position_speed(0.5)
        self.other_limb_interface.set_joint_position_speed(0.5)

        # create image publisher to head monitor
        self.pub = rospy.Publisher('/robot/xdisplay', Image)

        # calibrate the gripper
        self.gripper.calibrate()

        # display the start splash screen
        self.splash_screen("Tetris", "Blocks")

        # reset cameras
        self.reset_cameras()

        # close all cameras
        '''self.close_camera("head")
        self.close_camera("left")
        self.close_camera("right")'''

        # open required camera
        self.open_camera(self.limb, self.width, self.height)

        # subscribe to required camera
        self.subscribe_to_camera(self.limb)

        # distance of arm to table and block
        self.distance      = distance # 0.367 distance from camera to table
        self.tray_distance = distance - 0.075
        self.block_distance = distance - 0.064
        self.gripper_height      = 0.104
        self.block_height = 0.064
        self.approach_dist = 0.155 # for small gripper
#        self.approach_dist = 0.15 # for large gripper

        # move other arm out of harms way
        if arm == "left":
            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))

        for filename in glob.glob(self.image_dir + "*.jpg"):
			            os.remove(filename)

    # reset all cameras (incase cameras fail to be recognised on boot)
    def reset_cameras(self):
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()

    # open a camera and set camera parameters
    def open_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # close camera
        #cam.close()

        # set camera parameters
        cam.resolution          = (int(x_res), int(y_res))
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # open camera
        cam.open()

    # close a camera
    def close_camera(self, camera):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - close_camera - Invalid camera")

        # set camera parameters to automatic
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # close camera
        cam.close()

    # convert Baxter point to image pixel
    def baxter_to_pixel(self, pt, dist):
        x = (self.width / 2)                                                         \
          + int((pt[1] - (self.pose[1] + self.cam_y_offset)) / (self.cam_calib * dist))
        y = (self.height / 2)                                                        \
          + int((pt[0] - (self.pose[0] + self.cam_x_offset)) / (self.cam_calib * dist))

        return (x, y)

    # convert image pixel to Baxter point
    def pixel_to_baxter(self, px, dist):
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist)                \
          + self.pose[0] + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist)                 \
          + self.pose[1] + self.cam_y_offset

        return (x, y)

    # Not a tree walk due to python recursion limit
    def tree_walk(self, image, x_in, y_in):
        almost_black = (1, 1, 1)
        pixel_list = [(x_in, y_in)]                   # first pixel is black save position
        cv.Set2D(image, y_in, x_in, almost_black)     # set pixel to almost black
        to_do = [(x_in, y_in - 1)]                    # add neighbours to to do list
        to_do.append([x_in, y_in + 1])
        to_do.append([x_in - 1, y_in])
        to_do.append([x_in + 1, y_in])

        while len(to_do) > 0:
            x, y = to_do.pop()                             # get next pixel to test
            if cv.Get2D(image, y, x)[0] == self.black[0]:  # if black pixel found
                pixel_list.append([x, y])                  # save pixel position
                cv.Set2D(image, y, x, almost_black)        # set pixel to almost black
                to_do.append([x, y - 1])                   # add neighbours to to do list
                to_do.append([x, y + 1])
                to_do.append([x - 1, y])
                to_do.append([x + 1, y])

        return pixel_list

    # Remove artifacts and find largest object
    def look_for_ball_tray(self, canny):
        width, height = cv.GetSize(canny)

        centre   = (0, 0)
        max_area = 0

        # for all but edge pixels
        for x in range(1, width - 2):
            for y in range(1, height - 2):
                if cv.Get2D(canny, y, x)[0] == self.black[0]:       # black pixel found
                    pixel_list = self.tree_walk(canny, x, y)        # tree walk pixel
                    if len(pixel_list) < self.min_area:             # if object too small
                        for l in pixel_list:
                            cv.Set2D(canny, l[1], l[0], self.white) # set pixel to white
                    else:                                           # if object found
                        n = len(pixel_list)
                        if n > max_area:                            # if largest object found
                            sum_x  = 0                              # find centre of object
                            sum_y  = 0
                            for p in pixel_list:
                                sum_x  = sum_x + p[0]
                                sum_y  = sum_y + p[1]

                            centre = sum_x / n, sum_y / n           # save centre of object
                            max_area = n                            # save area of object

        if max_area > 0:                                            # in tray found
            cv.Circle(canny, (centre), 9, (250, 250, 250), -1)      # mark tray centre

        # display the modified canny
        cv.ShowImage("Modified Canny", canny)

        # 3ms wait
        cv.WaitKey(3)

        return centre                                        # return centre of object

    # flood fill edge of image to leave objects
    def flood_fill_edge(self, canny):
        width, height = cv.GetSize(canny)

        # set boarder pixels to white
        for x in range(width):
            cv.Set2D(canny, 0, x, self.white)
            cv.Set2D(canny, height - 1, x, self.white)

        for y in range(height):
            cv.Set2D(canny, y, 0, self.white)
            cv.Set2D(canny, y, width - 1, self.white)

        # prime to do list
        to_do = [(2, 2)]
        to_do.append([2, height - 3])
        to_do.append([width - 3, height - 3])
        to_do.append([width - 3, 2])

        while len(to_do) > 0:
            x, y = to_do.pop()                               # get next pixel to test
            if cv.Get2D(canny, y, x)[0] == self.black[0]:    # if black pixel found
                cv.Set2D(canny, y, x, self.white)            # set pixel to white
                to_do.append([x, y - 1])                     # add neighbours to to do list
                to_do.append([x, y + 1])
                to_do.append([x - 1, y])
                to_do.append([x + 1, y])

    # camera call back function
    def camera_callback(self, data, camera_name):
        # Convert image from a ROS image message to a CV image
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
            if self.publish_camera:
                self.pub.publish(data)
        except cv_bridge.CvBridgeError, e:
            print e

        # 3ms wait
        cv.WaitKey(3)

    # left camera call back function
    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")

    # right camera call back function
    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")

    # head camera call back function
    def head_camera_callback(self, data):
        self.camera_callback(data, "Head Camera")

    # create subscriber to the required camera
    def subscribe_to_camera(self, camera):
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        elif camera == "head":
            callback = self.head_camera_callback
            camera_str = "/cameras/head_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        camera_sub = rospy.Subscriber(camera_str, Image, callback)



    # Convert cv image to a numpy array
    def cv2array(self, im):
        depth2dtype = {cv.IPL_DEPTH_8U: 'uint8',
                       cv.IPL_DEPTH_8S: 'int8',
                       cv.IPL_DEPTH_16U: 'uint16',
                       cv.IPL_DEPTH_16S: 'int16',
                       cv.IPL_DEPTH_32S: 'int32',
                       cv.IPL_DEPTH_32F: 'float32',
                       cv.IPL_DEPTH_64F: 'float64'}
  
        arrdtype=im.depth
        a = numpy.fromstring(im.tostring(),
                             dtype = depth2dtype[im.depth],
                             count = im.width * im.height * im.nChannels)
        a.shape = (im.height, im.width, im.nChannels)

        return a

    # find next object of interest
    def find_next_golf_ball(self, ball_data, iteration):
        # if only one object then object found
        if len(ball_data) == 1:
            return ball_data[0]

        # sort objects right to left
        od = []
        for i in range(len(ball_data)):
            od.append(ball_data[i])

        od.sort()

        # if one ball is significantly to the right of the others
        if od[1][0] - od[0][0] > 30:       # if ball significantly to right of the others
            return od[0]                   # return right most ball
        elif od[1][1] < od[0][1]:          # if right most ball below second ball
            return od[0]                   # return lower ball
        else:                              # if second ball below right most ball
            return od[1]                   # return lower ball

    # find gripper angle to avoid nearest neighbour
    def find_gripper_angle(self, next_ball, ball_data):
        # if only one ball any angle will do
        if len(ball_data) == 1:
            return self.yaw

        # find nearest neighbour
        neighbour = (0, 0)
        min_d2    = float(self.width * self.width + self.height * self.height)

        for i in range(len(ball_data)):
            if ball_data[i][0] != next_ball[0] or ball_data[i][1] != next_ball[1]:
                dx = float(ball_data[i][0]) - float(next_ball[0])   # NB x and y are ushort
                dy = float(ball_data[i][1]) - float(next_ball[1])   # float avoids error
                d2 = (dx * dx) + (dy * dy)
                if d2 < min_d2:
                    neighbour = ball_data[i]
                    min_d2    = d2

        # find best angle to avoid hitting neighbour
        dx = float(next_ball[0]) - float(neighbour[0])
        dy = float(next_ball[1]) - float(neighbour[1])
        if abs(dx) < 1.0:
            angle = - (math.pi / 2.0)             # avoid divide by zero
        else:
            angle = math.atan(dy / dx)            # angle in radians between -pi and pi
        angle = angle + (math.pi / 2.0)           # rotate pi / 2 radians
        if angle > math.pi / 2.0:                 # ensure angle between -pi and pi
            angle = angle - math.pi

        return - angle                            # return best angle to grip golf ball

    # if ball near any of the ball tray places
    def is_near_ball_tray(self, ball):
        for i in self.ball_tray_place:
            d2 = ((i[0] - ball[0]) * (i[0] - ball[0]))           \
               + ((i[1] - ball[1]) * (i[1] - ball[1]))
            if d2 < 0.0004:
               return True

        return False

    # Use Hough circles to find ball centres (Only works with round objects)
    def hough_it(self, n_ball, iteration):
        cv_image = cv.fromarray(self.cv_image)
        # create gray scale image of balls
        gray_image = cv.CreateImage((self.width, self.height), 8, 1)
        cv.CvtColor(cv_image, gray_image, cv.CV_BGR2GRAY)

        # create gray scale array of balls
        gray_array = self.cv2array(gray_image)

        # find Hough circles
        circles = cv2.HoughCircles(gray_array, cv.CV_HOUGH_GRADIENT, 1, 40, param1=50,  \
                  param2=self.hough_accumulator, minRadius=self.hough_min_radius,       \
                  maxRadius=self.hough_max_radius)

        # Check for at least one ball found
        if circles is None:
            # display no balls found message on head display
            #self.splash_screen("no balls", "found")
            # no point in continuing so exit with error message
            sys.exit("ERROR - hough_it - No golf balls found")

        circles = numpy.uint16(numpy.around(circles))

        ball_data = {}
        n_balls   = 0

        circle_array = numpy.asarray(self.cv_image)

        # check if golf ball is in ball tray
        for i in circles[0,:]:
            # convert to baxter coordinates
            ball = self.pixel_to_baxter((i[0], i[1]), self.tray_distance)

            if self.is_near_ball_tray(ball):
                # draw the outer circle in red
                cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
                # draw the center of the circle in red
                cv2.circle(circle_array, (i[0], i[1]), 2, (0, 0, 255), 3)
            elif i[1] > 800:
                # draw the outer circle in red
                cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
                # draw the center of the circle in red
                cv2.circle(circle_array, (i[0], i[1]), 2, (0, 0, 255), 3)
            else:
                # draw the outer circle in green
                cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle in green
                cv2.circle(circle_array, (i[0], i[1]), 2, (0, 255, 0), 3)

                ball_data[n_balls]  = (i[0], i[1], i[2])
                n_balls            += 1

        circle_image = cv.fromarray(circle_array)

        cv.ShowImage("Hough Circle", circle_image)

        # 3ms wait
        cv.WaitKey(3)

        # display image on head monitor
        s = "Searching for golf balls"
        self.display_screen(circle_image, s)

        if self.save_images:
            # save image of Hough circles on raw image
            file_name = self.image_dir                                                 \
                      + "hough_circle_" + str(n_ball) + "_" + str(iteration) + ".jpg"
            cv.SaveImage(file_name, circle_image)

        # Check for at least one ball found
        if n_balls == 0:                    # no balls found
            # display no balls found message on head display
            #self.splash_screen("no balls", "found")
            # less than 12 balls found, no point in continuing, exit with error message
            sys.exit("ERROR - hough_it - No golf balls found")

        # select next ball and find it's position
        next_ball = self.find_next_golf_ball(ball_data, iteration)

        # find best gripper angle to avoid touching neighbouring ball
        angle = self.find_gripper_angle(next_ball, ball_data)

        # return next golf ball position and pickup angle
        return next_ball, angle

    # move a limb
    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")
        
        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if self.limb == limb:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            #self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if self.limb == limb:               # if working arm
            quaternion_pose = self.limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.pose = [position[0], position[1],                                \
                         position[2], self.pose[3], self.pose[4], self.pose[5]]

    # find distance of limb from nearest line of sight object
    def get_distance(self, limb):
        if limb == "left":
            dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        # convert mm to m and return distance
        return float(dist / 1000.0)

    # update pose in x and y direction
    def update_pose(self, dx, dy, dz):
        x = self.pose[0] + dx
        y = self.pose[1] + dy
        z = self.pose[2] + dz
        pose = [x, y, z, self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, pose)

    # used to place camera over the ball tray - ensure the focused point is in the middle
    def ball_tray_iterate(self, iteration, centre, colour):
        # print iteration number
        print "Iteration ", iteration
        # find displacement of object from centre of image
        pixel_dx    = (self.width / 2) - centre[0]
        pixel_dy    = (self.height / 2) - centre[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.block_distance)
        #pdb.set_trace()
        x_offset = - pixel_dy * self.cam_calib * self.block_distance
        y_offset = - pixel_dx * self.cam_calib * self.block_distance

        rospy.loginfo(error)
        # if error in current position too big
        if error > self.tray_tolerance:

            print error
            # correct pose
            self.update_pose(x_offset, y_offset, 0)
            # find new centre
            centre = self.detect_colour(iteration, colour)

            # find displacement of object from centre of image
            pixel_dx    = (self.width / 2) - centre[0]
            pixel_dy    = (self.height / 2) - centre[1]
            pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
            error       = float(pixel_error * self.cam_calib * self.block_distance)

        return True, centre, error

    # randomly adjust a pose to dither arm position
    # used to prevent stalemate when looking for ball tray
    def dither(self):
        x = self.ball_tray_x + (random.random() / 10.0)
        y = self.ball_tray_y + (random.random() / 10.0)
        pose = (x, y, self.pose[2], self.roll, self.pitch, self.yaw)

        return pose

    # find the ball tray
    def canny_it(self, iteration):
        cv_image = cv.fromarray(self.cv_image)
        if self.save_images:
            # save raw image of ball tray
            file_name = self.image_dir + "ball_tray_" + str(iteration) + ".jpg"
            
            cv.SaveImage(file_name, cv_image)
        
        # create an empty image variable, the same dimensions as our camera feed.
        gray = cv.CreateImage((cv.GetSize(cv_image)), 8, 1)

        # convert the image to a grayscale image
        cv.CvtColor(cv_image, gray, cv.CV_BGR2GRAY)

        # display image on head monitor
        s = "Looking for ball tray"
        self.display_screen(circle_image, s)

        # create a canny edge detection map of the greyscale image
        cv.Canny(gray, self.canny, self.canny_low, self.canny_high, 3)

        # display the canny transformation
        cv.ShowImage("Canny Edge Detection", self.canny)

        if self.save_images:
            # save Canny image of ball tray
            file_name = self.image_dir + "canny_tray_" + str(iteration) + ".jpg"
            cv.SaveImage(file_name, self.canny)

        # flood fill edge of image to leave only objects
        self.flood_fill_edge(self.canny)
        ball_tray_centre = self.look_for_ball_tray(self.canny)

        # 3ms wait
        cv.WaitKey(3)

        while ball_tray_centre[0] == 0:
            if random.random() > 0.6:
                self.baxter_ik_move(self.limb, self.dither())

            ball_tray_centre = self.canny_it(iteration)

        return ball_tray_centre

    # find places for golf balls
    def find_places(self, c):
        cv_image = cv.fromarray(self.cv_image)
        # find long side of ball tray
        l1_sq = ((c[1][0] - c[0][0]) * (c[1][0] - c[0][0])) +           \
                ((c[1][1] - c[0][1]) * (c[1][1] - c[0][1]))
        l2_sq = ((c[2][0] - c[1][0]) * (c[2][0] - c[1][0])) +           \
                ((c[2][1] - c[1][1]) * (c[2][1] - c[1][1]))

        if l1_sq > l2_sq:                     # c[0] to c[1] is a long side
            cc = [c[0], c[1], c[2], c[3]]
        else:                                 # c[1] to c[2] is a long side
            cc = [c[1], c[2], c[3], c[0]]

        # ball tray corners in baxter coordinates
        for i in range(4):
            self.ball_tray_corner[i] = self.pixel_to_baxter(cc[i], self.tray_distance)

        # ball tray places in pixel coordinates
        ref_x = cc[0][0]
        ref_y = cc[0][1]
        dl_x  = (cc[1][0] - cc[0][0]) / 8
        dl_y  = (cc[1][1] - cc[0][1]) / 8
        ds_x  = (cc[2][0] - cc[1][0]) / 6
        ds_y  = (cc[2][1] - cc[1][1]) / 6

        p     = {}
        p[0]  = (ref_x + (3 * dl_x) + (3 * ds_x), ref_y + (3 * dl_y) + (3 * ds_y))
        p[1]  = (ref_x + (5 * dl_x) + (3 * ds_x), ref_y + (5 * dl_y) + (3 * ds_y))
        p[2]  = (ref_x + (3 * dl_x) + (1 * ds_x), ref_y + (3 * dl_y) + (1 * ds_y))
        p[3]  = (ref_x + (5 * dl_x) + (1 * ds_x), ref_y + (5 * dl_y) + (1 * ds_y))
        p[4]  = (ref_x + (3 * dl_x) + (5 * ds_x), ref_y + (3 * dl_y) + (5 * ds_y))
        p[5]  = (ref_x + (5 * dl_x) + (5 * ds_x), ref_y + (5 * dl_y) + (5 * ds_y))
        p[6]  = (ref_x + (1 * dl_x) + (3 * ds_x), ref_y + (1 * dl_y) + (3 * ds_y))
        p[7]  = (ref_x + (7 * dl_x) + (3 * ds_x), ref_y + (7 * dl_y) + (3 * ds_y))
        p[8]  = (ref_x + (1 * dl_x) + (1 * ds_x), ref_y + (1 * dl_y) + (1 * ds_y))
        p[9]  = (ref_x + (7 * dl_x) + (1 * ds_x), ref_y + (7 * dl_y) + (1 * ds_y))
        p[10] = (ref_x + (1 * dl_x) + (5 * ds_x), ref_y + (1 * dl_y) + (5 * ds_y))
        p[11] = (ref_x + (7 * dl_x) + (5 * ds_x), ref_y + (7 * dl_y) + (5 * ds_y))

        for i in range(12):
            # mark position of ball tray places
            cv.Circle(cv_image, (int(p[i][0]), int(p[i][1])), 5, (0, 250, 0), -1)

            # ball tray places in baxter coordinates
            self.ball_tray_place[i] = self.pixel_to_baxter(p[i], self.tray_distance)

        # display the ball tray places
        cv.ShowImage("Egg tray", cv_image)

        if self.save_images:
            # save ball tray image with overlay of ball tray and ball positions
            file_name = self.image_dir + "ball_tray.jpg"
            cv.SaveImage(file_name, cv_image)

        # 3ms wait
        cv.WaitKey(3)

    # find four corners of the ball tray
    def find_corners(self, centre):
        cv_image = cv.fromarray(self.cv_image)
        # find bottom corner
        max_x  = 0
        max_y  = 0

        for x in range(100, self.width - 100):
            y = self.height - 20
            while y > 0 and cv.Get2D(self.canny, y, x)[0] > 100:
                y = y - 1
            if y > 20:
                cv.Set2D(cv_image, y, x, (0, 0, 255))
                if y > max_y:
                    max_x = x
                    max_y = y

        corner_1 = (max_x, max_y)

        # find left corner
        min_x  = self.width
        min_y  = 0

        for y in range(100, self.height - 100):
            x = 20
            while x < self.width - 1 and cv.Get2D(self.canny, y, x)[0] > 100:
                x = x + 1
            if x < self.width - 20:
                cv.Set2D(cv_image, y, x, (0, 255, 0, 0))
                if x < min_x:
                    min_x = x
                    min_y = y

        corner_2 = (min_x, min_y)

        # display corner image
        cv.ShowImage("Corner", cv_image)

        if self.save_images:
            # save Canny image
            file_name = self.image_dir + "egg_tray_canny.jpg"
            cv.SaveImage(file_name, self.canny)

            # mark corners and save corner image
            cv.Circle(cv_image, corner_1, 9, (0, 250, 0), -1)
            cv.Circle(cv_image, corner_2, 9, (0, 250, 0), -1)
            file_name = self.image_dir + "corner.jpg"
            cv.SaveImage(file_name, cv_image)

        # 3ms wait
        cv.WaitKey(3)

        # two corners found and centre known find other two corners
        corner_3 = ((2 * centre[0]) - corner_1[0], (2 * centre[1]) - corner_1[1])
        corner_4 = ((2 * centre[0]) - corner_2[0], (2 * centre[1]) - corner_2[1])

        # draw ball tray boundry
        c1 = (int(corner_1[0]), int(corner_1[1]))
        c2 = (int(corner_2[0]), int(corner_2[1]))
        c3 = (int(corner_3[0]), int(corner_3[1]))
        c4 = (int(corner_4[0]), int(corner_4[1]))

        cv.Line(cv_image, c1, c2, (255, 0, 0), thickness=3)
        cv.Line(cv_image, c2, c3, (255, 0, 0), thickness=3)
        cv.Line(cv_image, c3, c4, (255, 0, 0), thickness=3)
        cv.Line(cv_image, c4, c1, (255, 0, 0), thickness=3)
        file_name = self.image_dir + "ball_tray_boundary.jpg"
        cv.SaveImage(file_name, cv_image)

        return True, (corner_1, corner_2, corner_3, corner_4)

    def detect_square(self, square):
        colour_centre = (0,0)
        # compute the rotated bounding box of the largest contour
        rect = cv2.minAreaRect(square)
        box = numpy.int0(cv2.cv.BoxPoints(rect))

        # draw a bounding box arounded the detected square
        cv2.drawContours(self.cv_image, [box], -1, (0, 255, 0), 1)
        #cv2.imshow("Detected square", self.cv_image)
        #cv2.destroyAllWindows()
        l = zip(*box)
        centroid_x = int(sum(l[0])/len(l[0]))
        centroid_y = int(sum(l[1])/len(l[1]))
#        pdb.set_trace()
        approx = cv2.approxPolyDP(square,0.01*cv2.arcLength(square,True),True)
        
        # check the detected square has the right size
        print (box[0]-box[1])[1]
        print (box[0]-box[3])[0]
        #print abs((box[0]-box[1])[1]/(box[0]-box[3])[0])#width/height should be around 1 if it is a square
        #print abs((box[0]-box[1])[1])-abs((box[0]-box[3])[0])
        if 1 < 50: #abs(abs((box[0]-box[1])[1])-abs((box[0]-box[3])[0]))
            colour_centre = (centroid_x,centroid_y)
            cv_image = cv.fromarray(self.cv_image)

            # draw ball tray boundry
            cv.Circle(cv_image, colour_centre, 5, (0, 250, 0), -1)
            cv.ShowImage("Detected", cv_image)
            s = "Detected block"
            self.display_screen(cv_image, s)
            #cv2.imshow("Detected2", self.cv_image)
            cv.WaitKey(3)
            #cv2.destroyAllWindows()
            if self.save_images:
                file_name = self.image_dir + "centre.jpg"
                cv.SaveImage(file_name, cv_image)
        return colour_centre

    def detect_colour(self, iteration, colour):
        cv_image = cv.fromarray(self.cv_image)
        colour_centre = (0,0)
        if self.save_images:
            # save raw image of ball tray
            file_name = self.image_dir + colour + "_" + str(iteration) + ".jpg"
            cv.SaveImage(file_name, cv_image)

        # loop over the boundaries for colour detection
        for (lower, upper) in self.colour_boundaries[colour]:
            # create NumPy arrays from the boundaries
            lower = numpy.array(lower, dtype = "uint8")
            upper = numpy.array(upper, dtype = "uint8")
         
            # find the colors within the specified boundaries and apply
            # the mask - extracted colour image
            mask = cv2.inRange(self.cv_image, lower, upper)
            output = cv2.bitwise_and(self.cv_image, self.cv_image, mask = mask)
       
            # show the images
            s = "Looking for colour %s" % (colour)
            #cv2.imshow(s, numpy.hstack([self.cv_image, output]))
            self.display_screen(output, s)
            cv2.waitKey(3)
            #cv2.destroyAllWindows()
            if self.save_images:
                cv_image = cv.fromarray(output)
                file_name = self.image_dir + colour + "_detected_"  + str(iteration) + ".jpg"
                cv.SaveImage(file_name, cv_image)
                cv.WaitKey(3)

        # find the contours in the thresholded image, then sort the contours
        # by their area, keeping only the largest one
        (cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        sorted_cnts = sorted(cnts, key = cv2.contourArea, reverse = True)
        c = sorted_cnts[0]
        c_area = cv2.contourArea(c)
        print c_area
        #pdb.set_trace()
        # check if detected area is big enough, then approach
        if c_area > self.min_area:
            colour_centre = self.detect_square(c)
        # if detected area is not big enough, then walk around randomly
        while colour_centre[0] == 0:
            #if random.random() > 0.6:
            self.baxter_ik_move(self.limb, self.dither())
            colour_centre = self.detect_colour(iteration, colour)

        return colour_centre

    # find the ball tray
    def find_tetris_block(self, colour):
        ok = False
        s = "Look for block"
        self.display_screen(self.cv_image, s)
        while not ok:
            colour_centre = self.detect_colour(0, colour)

            error     = 2 * self.tray_tolerance
            iteration = 1

            # iterate until arm over centre of tray
            while error > self.tray_tolerance:
                ok, colour_centre, error = self.ball_tray_iterate(iteration,       \
                                          colour_centre, colour)
                #pdb.set_trace()
                
                iteration              += 1

        baxter_centre = self.pixel_to_baxter((colour_centre[1],colour_centre[0]), self.block_distance)
        pose_centre = self.baxter_to_pixel((self.pose[0],self.pose[1]), self.block_distance)
        print colour_centre

        

    # used to place camera over golf ball
    def golf_ball_iterate(self, n_ball, iteration, ball_data):
        # print iteration number
        print "GOLF BALL", n_ball, "ITERATION ", iteration

        # find displacement of ball from centre of image
        pixel_dx    = (self.width / 2) - ball_data[0]
        pixel_dy    = (self.height / 2) - ball_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.tray_distance)

        x_offset = - pixel_dy * self.cam_calib * self.tray_distance
        y_offset = - pixel_dx * self.cam_calib * self.tray_distance

        # update pose and find new ball data
        self.update_pose(x_offset, y_offset, 0)
        ball_data, angle = self.hough_it(n_ball, iteration)

        # find displacement of ball from centre of image
        pixel_dx    = (self.width / 2) - ball_data[0]
        pixel_dy    = (self.height / 2) - ball_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.tray_distance)

        return ball_data, angle, error

    # print all 6 arm coordinates (only required for programme development)
    def print_arm_pose(self):
        return
        pi = math.pi

        quaternion_pose = self.limb_interface.endpoint_pose()
        position        = quaternion_pose['position']
        quaternion      = quaternion_pose['orientation']
        euler           = tf.transformations.euler_from_quaternion(quaternion)

        print
        print "             %s" % self.limb
        print 'front back = %5.4f ' % position[0]
        print 'left right = %5.4f ' % position[1]
        print 'up down    = %5.4f ' % position[2]
        print 'roll       = %5.4f radians %5.4f degrees' %euler[0], 180.0 * euler[0] / pi
        print 'pitch      = %5.4f radians %5.4f degrees' %euler[1], 180.0 * euler[1] / pi
        print 'yaw        = %5.4f radians %5.4f degrees' %euler[2], 180.0 * euler[2] / pi

    def holdingObject(self):
        """
            Returns true if self.get_distance < 65.535 and gripper is open
        """
        rospy.sleep(0.2)
        
        return (self.get_distance(self.limb) < 65) and self.gripper.gripping()


    def approach(self):
        pdb.set_trace()
        while self.get_distance(self.limb) > 0.126:
            rospy.loginfo(self.get_distance(self.limb))
            dist = self.pose[2] + 0.11
            self.update_pose(0, 0, -dist)
            #print self.get_distance(self.limb)

    # find all the golf balls and place them in the ball tray
    def pick_and_place(self, offset_x, offset_y, offset_angle):
        n_ball = 0
        while True and n_ball < 1:              # assume no more than 12 golf balls
            n_ball          += 1
            iteration        = 0
            angle            = 0.0

            # use Hough circles to find balls and select one ball
            #next_ball, angle = self.hough_it(n_ball, iteration)

            error     = 2 * self.ball_tolerance

            s = "Pick up block"
            self.display_screen(self.cv_image, s)

            # slow down to reduce scattering of neighbouring golf balls
            #self.limb_interface.set_joint_position_speed(0.1)
#            pdb.set_trace()
            # adjust pose for camera/gripper offsets
            self.update_pose(self.cam_x_offset, self.cam_y_offset, 0)
            self.print_arm_pose()
            # move down to pick up ball
            self.approach()
            
            self.gripper.close()
            cv.WaitKey(10)
            s = "Moving to block to target location"
            self.display_screen(self.cv_image, s)
            print s
            self.update_pose(0,0,0.19)
            cv.WaitKey(10)
            # check if managed to grab object
            if not self.holdingObject():
                self.gripper.open()
                print "Failed to grab object"
                break
            # speed up again
            self.limb_interface.set_joint_position_speed(0.5)

            # display current image on head display
            self.display_screen(self.cv_image, s)
            dist = self.pose[2] + 0.101
            # move down
            pose = (self.pose[0] + offset_x,
                    self.pose[1] + offset_y,
                    self.pose[2] - dist,
                    self.pose[3],
                    self.pose[4],
                    angle + offset_angle)
            self.baxter_ik_move(self.limb, pose)
            pdb.set_trace()
            # display current image on head display
            s = "Placing tetris block down"
            self.display_screen(self.cv_image, s)
            cv.WaitKey(10)
            # open the gripper
            self.gripper.open()

            # prepare to look for next block
            pose = (self.ball_tray_x,
                    self.ball_tray_y,
                    self.ball_tray_z,
                    self.roll,
                    self.pitch,
                    self.yaw)
            self.baxter_ik_move(self.limb, pose)

        # display all balls found on head display
        self.splash_screen("all balls", "found")

        print "Tetris block placed"

    # display message on head display
    def display_screen(self, cv_image, s):
        cv_image = numpy.asarray(cv_image)
        position = (30, 60)
        cv2.putText(cv_image, s, position, cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, self.white, thickness = 3)

        # 3ms wait

        self.publish_camera = False
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.pub.publish(msg)
        #raw_input("Press Enter to continue: ")
        cv2.waitKey(500)
        self.publish_camera = True

    # display message on head display
    def splash_screen(self, s1, s2):
        splash_array = numpy.zeros((self.height, self.width, 3), numpy.uint8)
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 3.0, 3.0, 9)
        ((text_x, text_y), baseline) = cv.GetTextSize(s1, font)
        org = ((self.width - text_x) / 2, (self.height / 3) + (text_y / 2))
        cv2.putText(splash_array, s1, org, cv.CV_FONT_HERSHEY_SIMPLEX, 3.0,          \
                    self.white, thickness = 7)

        ((text_x, text_y), baseline) = cv.GetTextSize(s2, font)
        org = ((self.width - text_x) / 2, ((2 * self.height) / 3) + (text_y / 2))
        cv2.putText(splash_array, s2, org, cv.CV_FONT_HERSHEY_SIMPLEX, 3.0,          \
                    self.white, thickness = 7)

        # 3ms wait
        cv2.waitKey(3)
        # cv2_to_imgmsg must take numpy array
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(splash_array, encoding="bgr8")
        self.pub.publish(msg)

    def move_to_position(limb, pose, offsetx, offsety, offsetz):
        ikreq = SolvePositionIKRequest()
        # Add 5 cm offset in Z direction
        try:
            pose['position'] = Point(x=pose['position'][0] + offsetx,
                         y=pose['position'][1] + offsety,
                         z=pose['position'][2] + offsetz
                         )
        except Exception:
            pose['position'] = Point(x=pose['position'].x + offsetx,
                         y=pose['position'].y + offsety,
                         z=pose['position'].z + offsetz
                         )
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(pose_req)
        resp = iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))


    def search(self,joint):
        limb = self.limb_interface
        pose = limb.endpoint_pose()
        camera_name = joint + "_hand_camera"
        offsetx = 0.05
        block = 'NULL'
        print("Checking for block...")
        while block == 'NULL':

            new_angles = move_to_position(joint, pose,0,offsetx, 0)
            limb.move_to_joint_positions(new_angles)
            pose = limb.endpoint_pose()

            if len(new_angles) == 0:
                offsetx *= -1
                '''new_angles = move_to_position(joint, pose,0,0, -0.05)
                limb.move_to_joint_positions(new_angles)
                pose = limb.endpoint_pose()'''

                new_angles = move_to_position(joint, pose,0.05,0, 0.20)
                limb.move_to_joint_positions(new_angles)
                pose = limb.endpoint_pose()
            # camera controls
    
            # Define your image topic msg of type sensor_msgs.msg._Image.Image
            msg = rospy.wait_for_message('/cameras/' + camera_name + "/image", Image)        
            try:
                # Convert your ROS Image message to OpenCV2 of type numpy.ndarray
                cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite(filename, cv2_img)
            except CvBridgeError, e:
                print(e)
            else:
                #
                recognised_qr = final2.main(filename)

            if len(recognised_qr) > 0:
                break
            #image_listener(camera_name)
            #block = identify_block()
            #print block
        new_angles = move_to_position(joint, pose,0,offsetx*(-1), 0)
        limb.move_to_joint_positions(new_angles)
        pose = limb.endpoint_pose()

        return block


    # read the setup parameters from setup.dat
    def get_setup(self):
        file_name = self.baxter.datapath + "setup.dat"

        try:
            f = open(file_name, "r")
        except ValueError:
            sys.exit("ERROR: golf_setup must be run before golf")

        # find limb
        s = string.split(f.readline())
        if len(s) >= 3:
            if s[2] == "left" or s[2] == "right":
                limb = s[2]
            else:
                sys.exit("ERROR: invalid limb in %s" % file_name)
        else:
            sys.exit("ERROR: missing limb in %s" % file_name)

        # find distance to table
        s = string.split(f.readline())
        if len(s) >= 3:
            try:
                distance = float(s[2])
            except ValueError:
                sys.exit("ERROR: invalid distance in %s" % file_name)
        else:
            sys.exit("ERROR: missing distance in %s" % file_name)

        return limb, distance

    def main(self, **kwargs):
        # get setup parameters
        limb, distance = self.get_setup()
        print "limb     = ", limb
        print "distance = ", distance

        # move close to the ball tray
        self.pose = (self.ball_tray_x,
                    self.ball_tray_y,
                    self.ball_tray_z,
                    self.roll,
                    self.pitch,
                    self.yaw)
        self.baxter_ik_move(self.limb, self.pose)

        # find the ball tray
        #self.find_ball_tray()
        rospy.loginfo("Enter the colour for the block:")

        colour = sys.stdin.readline().strip()
        self.find_tetris_block(colour)
        print "Found Tetris block"
        # find all the golf balls and place them in the ball tray
        print self.pose
        raw_input("Pick and Place: Press Enter to continue: ")
        # move block by these coordinates/angles
        rospy.loginfo("Enter the rotation for the block: (90,180,270,360)")
        angle = sys.stdin.readline().strip()

        rospy.loginfo("Enter the number of cells to the right:")
        r = sys.stdin.readline().strip()
        offset_x = -0.05 *float(r)
        offset_y = -0.05*0
        #angle = -90
        offset_angle = float(angle) * (math.pi / 180)
        pdb.set_trace()
        self.pick_and_place(offset_x, offset_y, offset_angle)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

