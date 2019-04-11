#!/usr/bin/env python
import rospy
import roslib

import geometry_msgs.msg
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import message_filters

DEPTH_SENSOR_TOPIC = '/ai/depth'
RGB_SENSOR_TOPIC = '/ai/rgb'


class image_Viewer:

    def __init__(self, RGB,DEPTH):

        #Topics for camera feeds
        self._rgbTopic = RGB
        self._depthTopic = DEPTH

        #Numpy arrays for storing camera data
        self._rgbArr = None
        self._depthArr = None

        #OpenCV bridge
        self._BRIDGE = CvBridge()

        self._get_images()

 
        
    def callback(self,rgb,depth):
        while not rospy.is_shutdown():
            try:
                rgbImage = self._BRIDGE.imgmsg_to_cv2(rgb, "bgr8")

                depthImage = self._BRIDGE.imgmsg_to_cv2(depth, "32FC1")
                depthImage = cv2.cvtColor(depthImage, cv2.COLOR_GRAY2BGR)

                images1 = np.hstack((rgbImage, depthImage))
                cv2.imshow("AI - RGB and depth map", depthImage)

            except rospy.ROSInterruptException:
                print("Shuting down Enhanced Grasping!")
            except IOError:
                print("Shuting down Enhanced Grasping!")



    def _get_images(self):
        rospy.init_node('image', anonymous=True)

        rgbImage=message_filters.Subscriber(self._rgbTopic, Image)

        depthImage=message_filters.Subscriber(self._depthTopic, Image)
        
        ts = message_filters.ApproximateTimeSynchronizer([rgbImage, depthImage], 10, 0.1)
        ts.registerCallback(self.callback)
        rospy.spin()



if __name__ == '__main__':
    try:
        I = image_Viewer(RGB_SENSOR_TOPIC,DEPTH_SENSOR_TOPIC)

    except rospy.ROSInterruptException:
        pass
