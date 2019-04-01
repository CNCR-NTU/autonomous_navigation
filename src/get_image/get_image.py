#!/usr/bin/env python
import rospy
import roslib

import geometry_msgs.msg
import sensor_msgs.msg

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

DEPTH_SENSOR_TOPIC = '/orbbec_astra/depth/image'
RGB_SENSOR_TOPIC = '/orbbec_astra/rgb/image_raw'


class image_Viewer:

    def __init__(self, RGB,DEPTH):

        #Topics for camera feeds
        self.__rgbTopic = RGB
        self.__depthTopic = DEPTH

        #Numpy arrays for storing camera data
        self.__rgbArr = None
        self.__depthArr = None

        #OpenCV bridge
        self.__BRIDGE = CvBridge()

        self.__get_images()

    def __rgbCallback(self, Image):
        self.__rgbArr = self.__BRIDGE.imgmsg_to_cv2(Image, Image.encoding)

        #print Image.encoding

        #self.__rgbArr = cv2.cvtColor(self.__rgbArr, cv2.COLOR_BGR2RGB)



    def __depthCallback(self,Image):

        self.__depthArr = self.__BRIDGE.imgmsg_to_cv2(Image, Image.encoding)
        #print Image.encoding
        self.__depthArr = cv2.cvtColor(self.__depthArr, cv2.COLOR_GRAY2BGR)

        #if Image.encoding == 'rgb8':
            #self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        self.__combineArrays()
        
        

    def __combineArrays(self):
        #Combine into single image

        image = np.hstack((self.__rgbArr,self.__depthArr))

        self.__showImage(image)

    def __showImage(self,image):
        cv2.imshow("Summit_XL Cam Feed", image)
        cv2.waitKey(20)

 
    def __get_images(self):
        rospy.init_node('image', anonymous=True)
        rospy.Subscriber(self.__rgbTopic, sensor_msgs.msg.Image, self.__rgbCallback)
        rospy.Subscriber(self.__depthTopic, sensor_msgs.msg.Image, self.__depthCallback)


if __name__ == '__main__':
    try:
        I = image_Viewer(RGB_SENSOR_TOPIC,DEPTH_SENSOR_TOPIC)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
