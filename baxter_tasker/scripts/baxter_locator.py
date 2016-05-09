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
import math, operator
import os, glob
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
from tetris_block import TetrisBlock


# directory used to save analysis images
image_directory = ""

# locate class
class BaxterLocator:
    def __init__(self, baxter):
        # arm ("left" or "right")

        self.arm = "right"
        distance = 0.367
        self.limb           = self.arm
        self.limb_interface = baxter_interface.Limb(self.limb)
        self.baxter = baxter
        if self.arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(self.arm)

        # image directory
        self.image_dir = os.path.dirname(os.path.abspath(__file__)) + "/"

        # flag to control saving of analysis images
        self.save_images = False
        self.publish_camera = False

        # required position accuracy in metres - +/-5mm accuracy in specifications
        self.ball_tolerance = 0.05
        self.tray_tolerance = 0.05

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
        self.cam_x_offset = 0.036                      # camera gripper offset
        self.cam_y_offset = -0.018
        self.width        = 960                        # Camera resolution
        self.height       = 600
        self.pose_z_to_limb_dist_ratio = 0.72

        # Hough circle accumulator threshold and minimum radius.
        self.hough_accumulator = 35
        self.hough_min_radius  = 15
        self.hough_max_radius  = 35

        # canny image
        self.canny = cv.CreateImage((self.width, self.height), 8, 1)

        # Canny transform parameters
        self.canny_low  = 45
        self.canny_high = 150
        self.tetris_blocks = {}
        self.shade = 1 #multiply the colours by this number to include shade
        self.colour_boundaries = (0,0)

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
        #self.gripper.calibrate()

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

        for filename in glob.glob(self.image_dir + "*.jpg"):
			            os.remove(filename)
        if self.arm == "left":
            self.update_pose()
            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
            self.baxter_ik_move("left", (self.ball_tray_x,-self.ball_tray_y,self.ball_tray_z, -math.pi, 0.0, 0.0))
        else:
            self.update_pose()
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))
            self.baxter_ik_move("right", (self.ball_tray_x,self.ball_tray_y,self.ball_tray_z, -math.pi, 0.0, 0.0))

        self.initialiseBlocks()

    # reset arm positions 
    def reset_arms(self):
        self.update_pose()
        # move other arm out of the way
        if self.arm == "left":

            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
            self.baxter_ik_move("left", (self.pose[0],self.pose[1],self.pose[2], -math.pi, 0.0, 0.0))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))
            self.baxter_ik_move("right", (self.pose[0],self.pose[1],self.pose[2], -math.pi, 0.0, 0.0))



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

    # if ball near any of the ball tray places
    def is_near_ball_tray(self, ball):
        for i in self.ball_tray_place:
            d2 = ((i[0] - ball[0]) * (i[0] - ball[0]))           \
               + ((i[1] - ball[1]) * (i[1] - ball[1]))
            if d2 < 0.0004:
               return True

        return False

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
                self.splash_screen("Invalid", "move")
                # little point in continuing so exit with error message
                print "ERROR - no valid configuration found for requested move =", rpy_pose
                sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")
                return False
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")


        if self.limb == limb:               # if working arm
            self.update_pose()
        return True


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

    # used to place camera over the object - ensure the focused point is in the middle
    def object_iterate(self, iteration, centre, colour):
        # print iteration number
        print "Iteration ", iteration
        # find displacement of object from centre of image
        pixel_dx    = (self.width / 2) - centre[0]
        pixel_dy    = (self.height / 2) - centre[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.block_distance)
        ##pdb.set_trace()
        x_offset = - pixel_dy * self.cam_calib * self.block_distance
        y_offset = - pixel_dx * self.cam_calib * self.block_distance

        rospy.loginfo('object_iterate(): error = %f' % error)

        # if error in current position too big
        if error > self.tray_tolerance:

            print error
            # correct pose
            self.__moveBy((x_offset, y_offset, 0,0,0,0))
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
        x = self.pose[0] + (random.random() / 10.0)
        y = self.pose[1] + (random.random() / 10.0)
        pose = (x, y, self.pose[2], self.pose[3], self.pose[4], self.pose[5])

        return pose

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
#        #pdb.set_trace()
        approx = cv2.approxPolyDP(square,0.01*cv2.arcLength(square,True),True)
        
        # check the detected square has the right size
        print (box[0]-box[1])[1]
        print (box[0]-box[3])[0]

        if 1 < 50: 
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

    def get_colour_boundaries(self, colour, shade = 1):
        """
            Returns the colour boundary for the specified tetris block
        """
        max_bound = [i * shade for i in self.tetris_blocks[colour].getMaxColourRange()]
        min_bound = [i * shade for i in self.tetris_blocks[colour].getMinColourRange()]
        return [[min_bound, max_bound]]


    def detect_colour(self, iteration, colour):
        cv_image = cv.fromarray(self.cv_image)
        colour_centre = (0,0)
        if self.save_images:
            # save raw image of ball tray
            file_name = self.image_dir + colour + "_" + str(iteration) + ".jpg"
            cv.SaveImage(file_name, cv_image)
        ##pdb.set_trace()
        # loop over the boundaries for colour detection
        for (lower, upper) in self.colour_boundaries:
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
        ##pdb.set_trace()
        # check if detected area is big enough, then approach
        if c_area > self.min_area:
            colour_centre = self.detect_square(c)
        # if detected area is not big enough, then walk around randomly
        while colour_centre[0] == 0:
            #if random.random() > 0.6:
            if not self.baxter_ik_move(self.limb, self.dither()):
                break
            colour_centre = self.detect_colour(iteration, colour)


        return colour_centre


    # find the ball tray
    def find_tetris_block(self, colour):
        ok = False
        s = "Look for block"
        self.display_screen(self.cv_image, s)

        # get a better view of the board
        self.pose = (self.ball_tray_x,
                    self.ball_tray_y,
                    self.ball_tray_z,
                    self.roll,
                    self.pitch,
                    self.yaw)


        iteration = 0
        self.colour_boundaries = self.get_colour_boundaries(colour)
        while not ok and iteration < 10:
            colour_centre = self.detect_colour(0, colour)

            error     = 2 * self.tray_tolerance
            iteration = 1

            self.colour_boundaries = self.get_colour_boundaries(colour, self.shade)

            # iterate until arm over centre of tray
            while error > self.tray_tolerance:
                ok, colour_centre, error = self.object_iterate(iteration,       \
                                          colour_centre, colour)
                
                iteration              += 1

        baxter_centre = self.pixel_to_baxter((colour_centre[1],colour_centre[0]), self.block_distance)
        pose_centre = self.baxter_to_pixel((self.pose[0],self.pose[1]), self.block_distance)
        print colour_centre

        # adjust pose for camera/gripper offsets
        self.__moveBy((self.cam_x_offset, self.cam_y_offset, 0,0,0,0))
        print "Found Block"

        
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
        self.__moveBy((x_offset, y_offset, 0,0,0,0))
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
        rospy.loginfo('holdingObject(): limb distance = %f' % self.get_distance(self.limb))
        rospy.loginfo('holdingObject(): suction gripper activated = %s' % str(self.gripper.sucking()))
        return (self.get_distance(self.limb) < 0.135) and self.gripper.sucking()

    def moveBy(self, **kwargs):
        """
            Moves limb by offset pose
        """
        try:
            offset_pose = kwargs['offset_pose']
        except Exception,e:
            rospy.logerr("%s"%str(e))
            self.mm.neglect()
            return

        self.__moveBy(offset_pose)


    def __moveBy(self, offset_pose):
        """
            Moves limb by offset pose
        """

        # update self.pose to actual current pose
        self.update_pose()

        new_pose = tuple(map(operator.add, self.pose, offset_pose))
        self.baxter_ik_move(self.limb, new_pose)

    def update_pose(self):
        quaternion_pose = self.limb_interface.endpoint_pose()
        position        = quaternion_pose['position']

        quaternion = (quaternion_pose['orientation'].x,quaternion_pose['orientation'].y,quaternion_pose['orientation'].z,quaternion_pose['orientation'].w)
        orientation = tf.transformations.euler_from_quaternion(quaternion)

        rotation_angle = round(orientation[2]*180/math.pi,1)
        rospy.loginfo("update_pose:() Rotation angle is %s" % str(rotation_angle))

        # if working arm remember actual (x,y) position achieved
        self.pose = [position.x, position.y, position.z,                \
             orientation[0], orientation[1], orientation[2]]


    def approach(self, **kwargs):
       self.__approach()

    def __approach(self):
        """
            Approaches the object until the gripper is above it
        """
        i = 0
        self.update_pose()
        while self.get_distance(self.limb) > 65:		
	    self.__moveBy((0, 0, -0.05,0,0,0))
        
        dist = 0.175
        self.__moveBy((0, 0, -dist,0,0,0))
        # generic approach function: not used when height is never changed
        while False: #self.get_distance(self.limb) > 0.093:
            before_pose = self.pose[2]
            rospy.loginfo('approach(): limb distance = %f' % self.get_distance(self.limb))
            dist = max(0.019,(self.get_distance(self.limb) - 0.09)*self.pose_z_to_limb_dist_ratio) #0.115
            #dist = self.pose[2] + 0.115
            rospy.loginfo('approach(): self.pose.z = %f, dist = %f' % (self.pose[2],dist))
            self.__moveBy((0, 0, -dist,0,0,0))
            #print self.get_distance(self.limb)
            i += 1
            rospy.loginfo('approach(): before-after pose change = %f' % abs(before_pose - self.pose[2]))
            if abs(before_pose - self.pose[2]) < 0.01:
	        rospy.loginfo('approach(): limb blocked : self.pose.z = %f,  limb distance = %f' % (self.pose[2],self.get_distance(self.limb)))		
	        break


    def verticalMove(self, dist):
        """
            Vertically moves the gripper by the specified distance
        """

        self.__moveBy((0, 0, float(dist),0,0,0))


    # find all the blocks and place them
    def pick_and_place(self, offset_x, offset_y, offset_angle):
        n_ball = 0
        while True and n_ball < 1:              # assume no more than 12 golf balls
            n_ball          += 1
            iteration        = 0
            angle            = 0.0

            s = "Pick up block"
            self.display_screen(self.cv_image, s)
            cv.WaitKey(3)
            # slow down to reduce scattering of neighbouring golf balls
            #self.limb_interface.set_joint_position_speed(0.1)

            # move down to pick up ball
            self.__approach()
            
            self.gripper.close()
            cv.WaitKey(10)
            s = "Moving to block to target location"
            self.display_screen(self.cv_image, s)
            print s
            self.verticalMove(0.198)
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

            # rotate block
            self.__moveBy((offset_x, offset_y, 0,0,0,offset_angle))

            # place block
            self.verticalMove(0.185)

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


    def locate(self, colour, offset_pose):
        """
            looks for block, approaches and picks up, applies custom movement, drops
        """
        # get a better view of the board
        self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))
        if self.get_distance(self.limb) < 0.15 or self.get_distance(self.limb) < 75:
            # prepare to look for next block
            pose = (self.ball_tray_x,
                    self.ball_tray_y,
                    self.ball_tray_z,
                    self.roll,
                    self.pitch,
                    self.yaw)
            self.baxter_ik_move(self.limb, pose)

        self.publish_camera = True
        s = "Pick up block"
        self.display_screen(self.cv_image, s)
        cv.WaitKey(3)

        self.find_tetris_block(colour)
        attempt = 1
        while not self.holdingObject() and attempt < 4:

            self.limb_interface.set_joint_position_speed(0.8)
            self.__approach()
            self.gripper.close()
            self.verticalMove(0.175)
            cv.WaitKey(3)
            rospy.loginfo('locate(): holding object: %s ' % str(self.holdingObject()))
            if self.holdingObject():
                break
            s = "Failed to grab object, retry same position"
            rospy.loginfo(s)
            self.display_screen(self.cv_image, s)
            attempt += 1
           # self.__moveBy((random.randint()*0.01,random.randint()*0.01,0,0,0,0))

        success = False
        attempt = 1
        while self.holdingObject() and attempt < 3: #

            # custom movement
            s = "Moving block to target location"
            self.display_screen(self.cv_image, s)
            rospy.loginfo(s)
            rospy.loginfo('locate(): self.pose: %s' % str(self.pose))
            rospy.loginfo('locate(): offset_pose: %s' % str(offset_pose))
            self.__moveBy(offset_pose)

            self.verticalMove(-0.175)
            self.gripper.open()
            self.verticalMove(0.059)
            cv.WaitKey(2)
            success = True

            attempt += 1
        rospy.loginfo('Success: %s' % str(success))

        self.publish_camera = False
        
        if success:
            s = "Completed movement"

            self.baxter_ik_move(self.limb, pose)
            self.display_screen(self.cv_image, s)
            return True
        else:
            s = "Failed movement"
            self.display_screen(self.cv_image, s)
            return False

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


    def recognise_grid(self):
        """
           Recognises grid and object positions
           TO DO: use hough_lines.py to recognise grid
        """
        # get a better view of the board
        self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))

        pose = (self.ball_tray_x,
                    self.ball_tray_y,
                    self.ball_tray_z+0.30,
                    self.roll,
                    self.pitch,
                    self.yaw)
        self.baxter_ik_move(self.limb, pose)



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

    def initialiseBlocks(self):
        self.tetris_blocks['yellow'] = TetrisBlock('O','yellow',[80, 120, 100], [135, 250, 250])
        self.tetris_blocks['red'] = TetrisBlock('S','red',[17, 15, 80], [90, 80, 255])
        self.tetris_blocks['blue'] = TetrisBlock('I','blue',[100, 50, 0], [255, 185, 255])
        self.tetris_blocks['green'] = TetrisBlock('L','green',[22, 73, 29], [160, 255, 160])
        self.tetris_blocks['pink'] = TetrisBlock('T','pink',[120, 130, 140], [150, 160, 170])

    def main(self, **kwargs):
        # get setup parameters
        #limb, distance = self.get_setup()
        #print "limb     = ", limb
        #print "distance = ", distance
        self.initialiseBlocks()
        # move close to the ball tray
        self.pose = (self.ball_tray_x,
                    self.ball_tray_y,
                    self.ball_tray_z + 0.05,
                    self.roll,
                    self.pitch,
                    self.yaw)
        self.baxter_ik_move(self.limb, self.pose)
        pdb.set_trace()
        # find block
        rospy.loginfo("Enter the colour for the block:")
        colour = sys.stdin.readline().strip()

        #self.find_tetris_block(colour)

        raw_input("Pick and Place: Press Enter to continue: ")


        # move block by these coordinates/angles
        rospy.loginfo("Enter the rotation for the block: (90,180,270,360)")
        angle = sys.stdin.readline().strip()

        rospy.loginfo("Enter the number of cells to the right:")
        r = sys.stdin.readline().strip()
        offset_x = -0.05 *float(r)
        offset_y = -0.05*0
        offset_angle = float(angle) * (math.pi / 180)

        self.locate(colour,(offset_x, offset_y,0,0,0,0))
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("Baxter locator")
    baxter = BaxterLocator()
    baxter.main()


