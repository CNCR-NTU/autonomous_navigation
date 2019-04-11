#!/usr/bin/env python

# ==============================
#     Import Libraries
# ==============================

import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
from autonomous_navigation.msg import Position, ScanAtPosition

# ==============================
#     Import Messages
# ==============================

# ==============================
#   Set Topic Variables 
# ==============================
POS_TOPIC = '/summit_xl_controller/PostionWithLaser'
GOAL_TOPIC = 'goal/position'


# ===============================
#    create_map class
# ===============================
class create_map:
    """Creates a scatter plot of objects in the vicinity of the robot.
    Scatter plot data is obtained through LaserScan and trained neural network.
    Plot enables localisation features of the system.
    """

    def __init__(self, POS_TOPIC, GOAL_TOPIC, invertX=False, invertY=False):
        """Creates instance of create_map class.
        POS_TOPIC = Topic of msg type ScanAtPosition.
        ScanAtPosition msg contains a list of scans at a given x,z coord
        """

        self._positionTopic = POS_TOPIC
        self._goalTopic = GOAL_TOPIC
        # Stores position of Robot
        self._Position = Position(0.0, 0.0, 0.0)
        self._goalPosition = Position()

        rospy.init_node('create_map', anonymous=True)

        # Scatter Plot
        self._fig, self._scatPlot = plt.subplots()
        # Lists for storing data
        self._xArr, self._zArr = [], []
        self._laserX, self._laserZ = [], []
        self._goalX, self._goalZ = [], []

        # Inverters for Plot
        self._invertX = invertX
        self._invertY = invertY

        # Create plots
        self._summitPoints = self._scatPlot.scatter(
            self._xArr, self._zArr, c='green', edgecolors='none', s=100, marker="s")

        self._laserPoints = self._scatPlot.scatter(
            self._laserX, self._laserZ, c='red', edgecolors='none')

        self._goalPoints = self._scatPlot.scatter(
            self._goalX, self._goalZ, c='blue', edgecolors='none', s=150, marker="*")

        # Plot Annotations
        self._summitAnnotation = None
        self._goalAnnotation = None

        # Set Limits
        self._setLimits(8)

        self._getPosition()

    def _getPosition(self):
        """Gets latest message from position topic, calls callback with msg.
        Node keeps subsribing due to plot loop."""
        rospy.Subscriber(self._goalTopic, Position, self._goalCallback, queue_size=1)

        rospy.Subscriber(self._positionTopic, ScanAtPosition,
                         self._callback, queue_size=1)

        plt.show(block=True)

    def _goalCallback(self, value):
        self._goalX, self._goalZ = [], []

        self._goalX.append(self._Position.xPos + abs(self._Position.xPos - value.xPos) * (-1 if self._invertX else 1))
        self._goalZ.append(self._Position.zPos + abs(self._Position.zPos - value.zPos) * (-1 if self._invertY else 1))

    def _callback(self, value):
        """Processes information from ROS msg. Sets position of Robot """
        self._Position = value.pos

        self._xArr, self._zArr = [], []
        self._xArr.append(value.pos.xPos)
        self._zArr.append(value.pos.zPos)

        self._calcLaserCoord(value.scan, value.maxDist)

        self._plotMap()

    def _calcLaserCoord(self, laserList, maxDist):
        """Calculates all coordinates of LaserScans given their distance and angle.
        """
        self._laserX, self._laserZ = [], []

        self._setLimits(maxDist)

        for scan in laserList:
            ang = scan.angle + self._Position.orientation

            dist = scan.distance
            if dist < maxDist:
                dist *= 1000.0  # Convert to mm
                if abs(ang) <= math.pi / 2:
                    x = math.sin(ang) * dist
                    z = math.cos(ang) * dist
                else:
                    if ang > 0:
                        x = (math.sin(ang) * dist)
                        z = (math.cos(ang) * dist)
                    else:
                        x = (math.sin(ang) * dist)
                        z = -(math.cos(ang) * dist)

                self._laserX.append(self._Position.xPos + x * (-1 if self._invertX else 1))
                self._laserZ.append(self._Position.zPos + z * (-1 if self._invertY else 1))

    def _plotMap(self):
        """Plots all points from lists onto a scatter graph."""

        if self._summitAnnotation is not None:
            self._summitAnnotation.remove()
            self._goalAnnotation.remove()

        self._summitPoints.set_offsets(np.c_[self._xArr, self._zArr])
        self._summitAnnotation = self._scatPlot.annotate("Summit", (self._xArr[0], self._zArr[0]))

        self._laserPoints.set_offsets(np.c_[self._laserX, self._laserZ])

        self._goalPoints.set_offsets(np.c_[self._goalX, self._goalZ])
        self._goalAnnotation = self._scatPlot.annotate("Goal", (self._goalX[0], self._goalZ[0]))

        self._fig.canvas.draw_idle()
        plt.pause(0.01)

    def _setLimits(self, dist):
        """Set limits of plots based on position of Robot"""

        plt.xlim(self._Position.xPos - dist * 1000.0, self._Position.xPos + dist * 1000.0)
        plt.ylim(self._Position.zPos - dist * 1000.0, self._Position.zPos + dist * 1000.0)


if __name__ == '__main__':
    try:
        cm = create_map(POS_TOPIC, GOAL_TOPIC, invertX=True)

    except rospy.ROSInterruptException:
        pass
