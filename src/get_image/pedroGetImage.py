#!/usr/bin/env python

from __future__ import print_function
# -*- coding: utf-8 -*-
"""
:ABSTRACT:
This script is part of the Enhanced grasp project

:REQUIRES:

:
:AUTHOR:  Pedro Machado
:ORGANIZATION: Nottingham Trent University
:CONTACT: pedro.baptistamachado@ntu.ac.uk
:SINCE: <date>
:VERSION: 0.1

This file is part of <project> project.
the Robot 2 Robot interaction project can not be copied and/or distributed without the express
permission of Prof. Martin McGinnity <martin.mcginnity@ntu.ac.uk>

Copyright (C) 2019 All rights reserved, Nottingham Trent University
Computational Neuroscience and Cognitive Robotics Laboratory
email:  pedro.baptistamachado@ntu.ac.uk
website: https://www.ntu.ac.uk/research/groups-and-centres/groups/computational-neuroscience-and-cognitive-robotics-laboratory


"""
# ===============================================================================
# PROGRAM METADATA
# ===============================================================================
__author__ = 'Pedro Machado'
__contact__ = 'pedro.baptistamachado@ntu.ac.uk'
__copyright__ = 'Enhanced grasping project can not be copied and/or distributed \
without the express permission of Prof. Martin McGinnity <martin.mcginnity@ntu.ac.uk'
__license__ = '2019 (C) CNCR@NTU, All rights reserved'
__date__ = '13/02/2019'
__version__ = '0.1'
__file_name__ = 'getImage.py'
__description__ = 'image subscriber'
__compatibility__ = "Python 2"
__platforms__ = "Sawyer and AR10 hand"

# ===============================================================================
# IMPORT STATEMENTS
# ===============================================================================
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
import os
import numpy as np
import time
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
# ===============================================================================
# GLOBAL VARIABLES DECLARATIONS
# ===============================================================================
global recordFlag, images_counter
images_counter=1001
recordFlag=False

FOLDER = os.path.dirname(os.path.realpath(__file__))+"/capturedImages/"
# ===============================================================================
# METHODS
# ===============================================================================

def callback(rgb,depth,ai_rgb,ai_depth):
    global recordFlag, images_counter
    cv_image_rgb = bridge.imgmsg_to_cv2(rgb, "bgr8")
    cv_image_depth = bridge.imgmsg_to_cv2(depth, "bgr8")
    cv_image_ai_rgb = bridge.imgmsg_to_cv2(ai_rgb, "bgr8")
    cv_image_ai_depth = bridge.imgmsg_to_cv2(ai_depth, "bgr8")
    if recordFlag:
        cv2.imwrite(FOLDER + 'img' + str(images_counter) + '.jpg', cv_image_rgb)
        images_counter+=1
        images = np.hstack((cv_image_rgb, cv_image_depth))
        cv2.imshow("RGB and depth map", images)
    images1 = np.hstack((cv_image_ai_rgb, cv_image_ai_depth))
    cv2.imshow("AI - RGB and depth map", images1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('Quit')
        cv2.destroyAllWindows()

def listener():
    while not rospy.is_shutdown():
        try:
            image_ai_rgb_sub=message_filters.Subscriber("ai/rgb", Image)
            image_ai_depth_sub=message_filters.Subscriber("ai/depth", Image)
            image_rgb_sub = message_filters.Subscriber("d435/rgb", Image)
            image_depth_sub = message_filters.Subscriber("d435/depth", Image)
            ts = message_filters.ApproximateTimeSynchronizer([image_ai_rgb_sub, image_ai_depth_sub, image_rgb_sub, image_depth_sub], 10, 0.1)
            ts.registerCallback(callback)
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shuting down Enhanced Grasping!")
        except IOError:
            print("Shuting down Enhanced Grasping!")

# ===============================================================================
#  TESTING AREA
# ===============================================================================


# ===============================================================================
# MAIN METHOD
# ===============================================================================
if __name__ == '__main__':
    bridge = CvBridge()
    print("[Initializing Sawyer ROS node...]\n")
    rospy.init_node('dataRaw', anonymous=True)
    listener()
