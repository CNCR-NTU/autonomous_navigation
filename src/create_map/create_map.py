#!/usr/bin/env python

#==============================
#     Import Libraries
#==============================

import matplotlib.animation
import matplotlib.pyplot as plt
import numpy as np

import math

#==============================
#     Import Messages
#==============================

import geometry_msgs.msg
import roslib
import rospy
import sensor_msgs.msg
from autonomous_navigation.msg import Position, ScanAtPosition

#==============================
#   Set Topic Variables 
#==============================
POS_TOPIC = '/summit_xl_controller/PostionWithLaser'
GOAL_TOPIC = 'goal/position'

#===============================
#    create_map class
#===============================
class create_map:
    """Creates a scatter plot of objects in the vicinity of the robot.
    Scatter plot data is obtained through LaserScan and trained neural network.
    Plot enables localisation features of the system.
    """

    def __init__(self, POS_TOPIC,GOAL_TOPIC):
        """Creates instance of create_map class.
        POS_TOPIC = Topic of msg type ScanAtPosition.
        ScanAtPosition msg contains a list of scans at a given x,z coord
        """

        self.__positionTopic = POS_TOPIC
        self.__goalTopic = GOAL_TOPIC
        # Stores position of Robot
        self.__Position = Position()
        self.__goalPosition = Position()

        rospy.init_node('create_map', anonymous=True)

        # Scatter Plot
        self.__fig, self.__scatPlot = plt.subplots()
        # Lists for storing data
        self.__xArr, self.__zArr = [], []
        self.__laserX, self.__laserZ = [], []
        self.__goalX,self.__goalZ = [],[]

        # Create plots
        self.__summitPoints = self.__scatPlot.scatter(
            self.__xArr, self.__zArr, c='green', edgecolors='none',s=50)

        self.__laserPoints = self.__scatPlot.scatter(
            self.__laserX, self.__laserZ, c='red', edgecolors='none')

        self.__goalPoints = self.__scatPlot.scatter(
            self.__goalX,self.__goalZ,c='blue',edgecolors='none',s=50)

        #Plot Annotations
        self.__summitAnnotation = None
        self.__goalAnnotation = None

        # Set Limits
        self.__setLimits(8)

        self.__getPosition()

    def __getPosition(self):
        """Gets latest message from position topic, calls callback with msg.
        Node keeps subsribing due to plot loop."""
        rospy.Subscriber(self.__goalTopic, Position,self.__goalCallback,queue_size=1)

        rospy.Subscriber(self.__positionTopic, ScanAtPosition,
                         self.__callback, queue_size=1)

        plt.show(block=True)

    def __goalCallback(self,value):
        self.__goalX,self.__goalZ = [],[]


        self.__goalX.append(self.__Position.xPos - abs(self.__Position.xPos - value.xPos))
        self.__goalZ.append(self.__Position.zPos + abs(self.__Position.zPos - value.zPos))


    def __callback(self, value):
        """Processes information from ROS msg. Sets position of Robot """
        self.__Position = value.pos

        self.__xArr, self.__zArr = [], []
        self.__xArr.append(value.pos.xPos)
        self.__zArr.append(value.pos.zPos)

        self.__calcLaserCoord(value.scan, value.maxDist)

        self.__plotMap()

    def __calcLaserCoord(self, laserList, maxDist):
        """Calculates all coordinates of LaserScans given their distance and angle.
        """
        self.__laserX, self.__laserZ = [], []

        self.__setLimits(maxDist)

        for scan in laserList:
            ang = scan.angle + self.__Position.orientation
            dist = scan.distance
            if dist < maxDist:
                dist *= 1000.0
                if abs(ang) <= math.pi/2:
                    x = math.sin(ang)*dist
                    z = math.cos(ang)*dist
                else:
                    if ang > 0:
                        x = -(math.sin(ang)*dist)
                        z = (math.cos(ang)*dist)
                    else:
                        x = (math.sin(ang)*dist)
                        z = -(math.cos(ang)*dist)

                self.__laserX.append(self.__Position.xPos - x)
                self.__laserZ.append(self.__Position.zPos + z)

    def __plotMap(self):
        """Plots all points from lists onto a scatter graph."""

        if self.__summitAnnotation is not None:
            self.__summitAnnotation.remove()
            self.__goalAnnotation.remove()



        self.__summitPoints.set_offsets(np.c_[self.__xArr, self.__zArr])
        self.__summitAnnotation = self.__scatPlot.annotate("Summit",(self.__xArr[0],self.__zArr[0]))

        self.__laserPoints.set_offsets(np.c_[self.__laserX, self.__laserZ])


        self.__goalPoints.set_offsets(np.c_[self.__goalX, self.__goalZ])
        self.__goalAnnotation = self.__scatPlot.annotate("Goal",(self.__goalX[0],self.__goalZ[0]))

        self.__fig.canvas.draw_idle()
        plt.pause(0.01)

    def __setLimits(self,dist):
        """Set limits of plots based on position of Robot"""
        
        plt.xlim(self.__Position.xPos - dist*1000.0, self.__Position.xPos + dist*1000.0)
        plt.ylim(self.__Position.zPos - dist*1000.0, self.__Position.zPos + dist*1000.0)

if __name__ == '__main__':
    try:
        cm = create_map(POS_TOPIC,GOAL_TOPIC)

    except rospy.ROSInterruptException:
        pass
