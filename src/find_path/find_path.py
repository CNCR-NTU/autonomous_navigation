#!/usr/bin/env python

#==============================
#     Import Libraries 
#==============================
import matplotlib.animation
import matplotlib.pyplot as plt
import numpy as np

import roslib
import rospy

import math
#==============================
#     Import Messages 
#==============================

import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
from autonomous_navigation.msg import Position
from autonomous_navigation.msg import ScanAtPosition


#==============================
#     Set Topic Variables 
#==============================

POS_TOPIC = '/summit_xl_controller/position'
GOAL_TOPIC = '/goal/position'
CMD_TOPIC = '/summit_xl_control/cmd_vel'

MIN_DISTANCE = 0.1

#==============================
#     find_path class 
#==============================

#0.6
#9.5

class find_path:
    """Calculates path to navigate from current position to the goal
    """
    
    def __init__(self,posTop,goalTop,cmdTop):
        #rospy.init_node('find_path')

        self.__PositionX = [4]
        self.__PositionY = [5]

        self.__goalPositionX = [10]
        self.__goalPositionY = [10]

        # Scatter Plot
        self.__fig, self.__scatPlot = plt.subplots()

        # Create plots
        self.__summitPoints = self.__scatPlot.scatter(
            self.__PositionX, self.__PositionY, c='green', edgecolors='none',s=50)

        #self.__laserPoints = self.__scatPlot.scatter(
         #   self.__laserX, self.__laserZ, c='red', edgecolors='none')

        self.__goalPoints = self.__scatPlot.scatter(
            self.__goalPositionX,self.__goalPositionY,c='blue',edgecolors='none',s=50)

        #Plot Annotations
        self.__summitAnnotation = None
        self.__goalAnnotation = None

        plt.xlim(0, 15)
        plt.ylim(0, 15)

        self.__plotMap()

    def __plotMap(self):
        """Plots all points from lists onto a scatter graph."""

        if self.__summitAnnotation is not None:
            self.__summitAnnotation.remove()
            self.__goalAnnotation.remove()



        self.__summitPoints.set_offsets(np.c_[self.__PositionX, self.__PositionY])
        self.__summitAnnotation = self.__scatPlot.annotate("Summit",(self.__PositionX[0],self.__PositionY[0]))

        #self.__laserPoints.set_offsets(np.c_[self.__laserX, self.__laserZ])


        self.__goalPoints.set_offsets(np.c_[self.__goalPositionX, self.__goalPositionY])
        self.__goalAnnotation = self.__scatPlot.annotate("Goal",(self.__goalPositionX[0],self.__goalPositionY[0]))

        self.__fig.canvas.draw_idle()
        plt.show()

        
class aStar:


class Node:
    

if __name__ == '__main__':
    try:
        nav = find_path(POS_TOPIC,GOAL_TOPIC,CMD_TOPIC)

    except rospy.ROSInterruptException:
        pass