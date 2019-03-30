#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.animation
import numpy as np

import rospy
import roslib

import math


import geometry_msgs.msg
import sensor_msgs.msg
from autonomous_navigation.msg import Position
from autonomous_navigation.msg import ScanAtPosition

POS_TOPIC = '/summit_xl_controller/position'


class create_map:

    def __init__(self,POS_TOPIC):
        self.__positionTopic = POS_TOPIC

        rospy.init_node('create_map', anonymous=True)

        #Scatter Plot
        self.fig, self.ax = plt.subplots()
        self.__xArr,self.__zArr = [],[]
        self.laserX = []
        self.laserZ = []

        self.sc = self.ax.scatter(self.__xArr,self.__zArr,c='green',edgecolors='none')
        self.other = self.ax.scatter(self.laserX,self.laserZ,c='red',edgecolors='none')
        plt.xlim(-10,10)
        plt.ylim(-10,10)

        self.__Position = Position()

        self.__getPosition()

 
    def __getPosition(self):
        rate = rospy.Rate(30) #30hz

        while not rospy.is_shutdown():
            rospy.Subscriber(self.__positionTopic, ScanAtPosition, self.__callback)

            self.__plotMap()
            #plt.waitforbuttonpress()
            #plt.show()
            rate.sleep()


    def __callback(self,value):
        self.__Position = value.pos

        self.__xArr.append(value.pos.xPos)
        self.__zArr.append(value.pos.zPos)

        calcLaserCoord(value.LaserPoint,value.maxDist)

    
    def calcLaserCoord(self,laserList,maxDist):
        for scan in maxDist:
            ang = scan.angle
            if scan < maxDist:
                if ang > 0:
                    if ang > math.pi/2: #Top RIght
                        pass
                    else: # Bottom RIght
                        ang -= math.pi/2
                else:
                    if ang < -(math.pi/2): # Top left
                        ang += math.pi/2
                    else: #bottom left
                        pass
            #calc x
            self.laserX.append(scan.distance*math.sin(ang))
            #calc z
            self.laserZ.append(scan.distance*math.cos(ang))

    def __plotMap(self):

        self.sc.set_offsets(np.c_[self.__xArr,self.__zArr])
        self.fig.canvas.draw_idle()
        plt.pause(0.01)



if __name__ == '__main__':
    try:
        cm = create_map(POS_TOPIC)

    except rospy.ROSInterruptException:
                pass