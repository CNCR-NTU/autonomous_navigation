#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.animation
import numpy as np

import rospy
import roslib

import geometry_msgs.msg
import sensor_msgs.msg
from autonomous_navigation.msg import Position

POS_TOPIC = '/summit_xl_controller/position'


class create_map:

    def __init__(self,POS_TOPIC):
        self.__positionTopic = POS_TOPIC

        rospy.init_node('create_map', anonymous=True)


        #Scatter Plot
        self.fig, self.ax = plt.subplots()

        self.__xArr,self.__zArr = [],[]

        self.sc = self.ax.scatter(self.__xArr,self.__zArr,c='green',edgecolors='none')
        plt.xlim(-10,10)
        plt.ylim(-10,10)

        self.__Position = Position()

        self.__getPosition()

 
    def __getPosition(self):
        rate = rospy.Rate(30) #30hz

        while not rospy.is_shutdown():
            rospy.Subscriber(self.__positionTopic, Position, self.__callback)

            self.__plotMap()
            rate.sleep()



    def __callback(self,value):
        self.__Position = value

        self.__xArr.append(value.xPos)
        self.__zArr.append(value.zPos)

        
    def __plotMap(self):

        colors = ("green","red")
        groups = ("Summit_XL Path","Detected Objects")

        self.sc.set_offsets(np.c_[self.__xArr,self.__zArr])
        self.fig.canvas.draw_idle()
        plt.pause(0.1)



if __name__ == '__main__':
    try:
        cm = create_map(POS_TOPIC)

    except rospy.ROSInterruptException:
                pass