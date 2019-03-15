#!/usr/bin/env python
import rospy
import roslib

import geometry_msgs.msg
import sensor_msgs.msg

import cv2
from cv_bridge import CvBridge, CvBridgeError

DEPTH_SENSOR_TOPIC = '/orbbec_astra/depth/image'
RGB_SENSOR_TOPIC = '/orbbec_astra/rgb/image_raw'

class image_Viewer:

    def __init__(self,T):
        self.topic = T
        self.get_image()

    def callback(self,Image):
        BRIDGE = CvBridge()

        self.image = BRIDGE.imgmsg_to_cv2(Image,Image.encoding)

        if Image.encoding == 'rgb8':
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        print(self.image[0][0])

        winname = "Test"
        cv2.namedWindow(winname)
        cv2.imshow(winname, self.image)
        cv2.waitKey(20)


    def get_image(self):
        rospy.init_node('image', anonymous=True)
        rospy.Subscriber(self.topic, sensor_msgs.msg.Image, self.callback)



if __name__ == '__main__':
    try:
        I = image_Viewer(RGB_SENSOR_TOPIC)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass