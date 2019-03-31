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

import datetime

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
        plt.xlim(-30,30)
        plt.ylim(-30,30)

        self.__Position = Position()

        self.__getPosition()

 
    def __getPosition(self):

        rospy.Subscriber(self.__positionTopic, ScanAtPosition, self.__callback,queue_size=1)
        
        plt.show(block=True)

    def __callback(self,value):

        print "Recieved at: {}".format(datetime.datetime.now())

        self.__Position = value.pos

        self.__xArr.append(value.pos.xPos)
        self.__zArr.append(value.pos.zPos)

        self.calcLaserCoord(value.scan,value.maxDist)

        self.__plotMap()

    
    def calcLaserCoord(self,laserList,maxDist):
        print len(laserList)

        x,z = 0,0

        for scan in laserList:        
            ang = scan.angle

            if scan.distance < maxDist:
                if abs(ang) <= math.pi/2:    #Top Quadrants
                    x = math.sin(ang)*scan.distance
                    z = math.cos(ang)*scan.distance
                else:    #Bottom Quadrants
                    if ang > 0:
                        #BR
                        x = -(math.sin(ang)*scan.distance)
                        z = (math.cos(ang)*scan.distance)
                    else:
                        #BL
                        x = (math.sin(ang)*scan.distance)
                        z = -(math.cos(ang)*scan.distance)


                #calc x
                self.laserX.append(x)
                #calc z
                self.laserZ.append(z)


    def __plotMap(self):
        self.sc.set_offsets(np.c_[self.__xArr,self.__zArr])
        self.other.set_offsets(np.c_[self.laserX,self.laserZ])
        self.fig.canvas.draw_idle()
        plt.pause(0.01)



if __name__ == '__main__':
    try:
        cm = create_map(POS_TOPIC)

    except rospy.ROSInterruptException:
                pass