#!/usr/bin/env python
import math

import matplotlib.animation
import matplotlib.pyplot as plt
import numpy as np

import geometry_msgs.msg
import roslib
import rospy
import sensor_msgs.msg
from autonomous_navigation.msg import Position, ScanAtPosition

POS_TOPIC = '/summit_xl_controller/position'


class create_map:
    """Creates a scatter plot of objects in the vicinity of the robot.
    Scatter plot data is obtained through LaserScan and trained neural network.
    Plot enables localisation features of the system.
    """

    def __init__(self, POS_TOPIC):
        """Creates instance of create_map class.
        POS_TOPIC = Topic of msg type ScanAtPosition.
        ScanAtPosition msg contains a list of scans at a given x,z coord
        """

        self.__positionTopic = POS_TOPIC
        # Stores position of Robot
        self.__Position = Position()

        rospy.init_node('create_map', anonymous=True)

        # Scatter Plot
        self.__fig, self.__scatPlot = plt.subplots()
        # Lists for storing data
        self.__xArr, self.__zArr = [], []
        self.__laserX, self.__laserZ = [], []
        # Create plots
        self.__summitPoints = self.__scatPlot.scatter(
            self.__xArr, self.__zArr, c='green', edgecolors='none')
        self.__laserPoints = self.__scatPlot.scatter(
            self.__laserX, self.__laserZ, c='red', edgecolors='none')

        # Set Limits
        self.__setLimits()

        self.__getPosition()

    def __getPosition(self):
        """Gets latest message from position topic, calls callback with msg.
        Node keeps subsribing due to plot loop."""

        rospy.Subscriber(self.__positionTopic, ScanAtPosition,
                         self.__callback, queue_size=1)

        plt.show(block=True)

    def __callback(self, value):
        """Processes information from ROS msg. Sets position of Robot """
        self.__Position = value.pos

        self.__xArr.append(value.pos.xPos)
        self.__zArr.append(value.pos.zPos)

        self.__calcLaserCoord(value.scan, value.maxDist)

        self.__plotMap()

    def __calcLaserCoord(self, laserList, maxDist):
        """Calculates all coordinates of LaserScans given their distance and angle.
        """
        x, z = 0, 0

        for scan in laserList:
            ang = scan.angle

            if scan.distance < maxDist:
                if abs(ang) <= math.pi/2:
                    x = math.sin(ang)*scan.distance
                    z = math.cos(ang)*scan.distance
                else:
                    if ang > 0:
                        x = -(math.sin(ang)*scan.distance)
                        z = (math.cos(ang)*scan.distance)
                    else:
                        x = (math.sin(ang)*scan.distance)
                        z = -(math.cos(ang)*scan.distance)

                self.__laserX.append(x)
                self.__laserZ.append(z)

    def __plotMap(self):
        """Plots all points from lists onto a scatter graph."""

        self.__summitPoints.set_offsets(np.c_[self.__xArr, self.__zArr])
        self.__laserPoints.set_offsets(np.c_[self.__laserX, self.__laserZ])
        self.__fig.canvas.draw_idle()
        plt.pause(0.01)

    def __setLimits(self):
        plt.xlim(-30, 30)
        plt.ylim(-30, 30)

if __name__ == '__main__':
    try:
        cm = create_map(POS_TOPIC)

    except rospy.ROSInterruptException:
        pass
