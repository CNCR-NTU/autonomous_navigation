#!/usr/bin/env python

# ==============================
#     Import Libraries
# ==============================

import matplotlib.pyplot as plt
import numpy as np
import rospy
from autonomous_navigation.msg import Position, ObjectsAtPosition, LaserAtPosition

# ==============================
#     Import Messages
# ==============================

# ==============================
#   Set Topic Variables 
# ==============================
OBJECT_TOPIC = '/summit_xl/PosWithObj'
LASER_TOPIC = '/summit_xl/PosWithLaser'
GOAL_TOPIC = 'goal/position'


# ===============================
#    create_map class
# ===============================
class create_map:
    """Creates a scatter plot of objects in the vicinity of the robot.
    Scatter plot data is obtained through LaserScan and trained neural network.
    Plot enables localisation features of the system.
    """

    def __init__(self, obj_top, goal_top, las_top, invertX=False, invertY=False):
        """Creates instance of create_map class.
        POS_TOPIC = Topic of msg type ObjectsAtPosition.
        ObjectsAtPosition msg contains a list of scans at a given x,z coord
        """

        rospy.init_node('create_map', anonymous=True)

        # Topics
        self._objectTopic = obj_top
        self._goalTopic = goal_top
        self._laserTopic = las_top

        # Stores position of Robot and Goal
        self._Position = Position(orientation=0.0, xPos=0.0, zPos=0.0)
        self._goalPosition = Position()

        # Scatter Plot
        self._fig, self._scatPlot = plt.subplots()

        # Inverters
        self._invertX = invertX
        self._invertY = invertY

        # =========================================
        #           Lists for storing data
        # =========================================

        self._summitX, self._summitZ = [], []
        self._laserX, self._laserZ = [], []
        self._goalX, self._goalZ = [], []
        self._objectX, self._objectZ = [], []
        self._objectNames = []
        self._objectDetected = False
        # =========================================
        #               Create Plots
        # =========================================

        self._summitPoints = self._scatPlot.scatter(
            self._summitX, self._summitZ, c='green', edgecolors='none', s=100, marker="s")

        self._laserPoints = self._scatPlot.scatter(
            self._laserX, self._laserZ, c='red', edgecolors='none')

        self._goalPoints = self._scatPlot.scatter(
            self._goalX, self._goalZ, c='blue', edgecolors='none', s=150, marker="*")

        self._objectPoints = self._scatPlot.scatter(
            self._laserX, self._laserZ, c='magenta', edgecolors='none')

        # =========================================
        #       Lists for Annotation Objects
        # =========================================
        self._summitAnnotation = None
        self._goalAnnotation = None
        self._objectAnnotations = []

        # Set Limits
        self._set_plot_limits(4)

        self._get_position()

    def _get_position(self):
        """Gets latest message from position topic, calls callback with msg.
        Node keeps subscribing due to plot loop."""

        rospy.Subscriber(self._goalTopic, Position, self._goal_callback, queue_size=1)
        rospy.Subscriber(self._objectTopic, ObjectsAtPosition,
                         self._object_callback, queue_size=1)

        rospy.Subscriber(self._laserTopic, LaserAtPosition,
                         self._laser_callback, queue_size=1)

        plt.show(block=True)

    # =======================
    #       Callbacks
    # =======================

    def _goal_callback(self, value):
        """Set values of Goal coordinates"""
        self._objectDetected = False

        self._goalX, self._goalZ = [], []

        self._goalX.append(self._Position.xPos + abs(self._Position.xPos - value.xPos) * (-1 if self._invertX else 1))
        self._goalZ.append(self._Position.zPos + abs(self._Position.zPos - value.zPos) * (-1 if self._invertY else 1))

    def _object_callback(self, data):

        self._objectDetected = True

        self._objectX, self._objectZ, self._objectNames = [], [], []

        # Set Object Positions and Labels
        self._objectX = data.objectsX
        self._objectZ = data.objectsY
        self._objectNames = data.objects

    def _laser_callback(self, data):

        # Clear out Arrays
        self._summitX, self._summitZ = [], []
        self._laserX, self._laserZ = [], []

        # Set Summit Position
        self._summitX.append(data.pos.xPos)
        self._summitZ.append(data.pos.zPos)
        self._Position = data.pos
        # Set Laser Positions
        self._laserX = data.laserX
        self._laserZ = data.laserY

        self._plot_map()

    # =======================
    #   Plotting Functions
    # =======================

    def _plot_laser(self):
        """Plots Laser coordinates"""

        # Set Offsets
        self._laserPoints.set_offsets(np.c_[self._laserX, self._laserZ])

    def _plot_summit(self):
        """Plots Summit coordinates with annotation"""

        # Remove Existing Annotation
        if self._summitAnnotation is not None:
            self._summitAnnotation.remove()

        # Set Offsets
        self._summitPoints.set_offsets(np.c_[self._summitX, self._summitZ])

        # Update Annotation
        self._summitAnnotation = self._scatPlot.annotate("Summit", (self._summitX[0], self._summitZ[0]))

    def _plot_goal(self):
        """Plots Goal coordinates with annotation"""

        # Remove Existing Annotation
        if self._goalAnnotation is not None:
            self._goalAnnotation.remove()

        # Set Offsets
        self._goalPoints.set_offsets(np.c_[self._goalX, self._goalZ])

        # Update Annotation
        self._goalAnnotation = self._scatPlot.annotate("Goal", (self._goalX[0], self._goalZ[0]))

    def _plot_objects(self):
        """Plots Object coordinates with respective annotation"""

        # Remove Existing Annotations
        for annotation in self._objectAnnotations:
            annotation.remove()
        self._objectAnnotations[:] = []

        # Set Offsets
        self._objectPoints.set_offsets(np.c_[self._objectX, self._objectZ])

        # Update Annotations
        for name, x, z in zip(self._objectNames, self._objectX, self._objectZ):
            self._objectAnnotations.append(self._scatPlot.annotate(name, (x, z)))

    def _plot_map(self):
        """Plots all points from lists onto a scatter graph."""

        # Plot points if they exist

        if len(self._laserX) > 0:
            self._plot_laser()

        if len(self._goalX) > 0:
            self._plot_goal()

        if len(self._summitX) > 0:
            self._plot_summit()

        self._plot_objects()

        # Update Plot
        self._fig.canvas.draw_idle()

        plt.pause(0.01)

    def _set_plot_limits(self, dist):
        """Set limits of plots based on position of Robot"""

        plt.xlim(self._Position.xPos - dist * 1000.0, self._Position.xPos + dist * 1000.0)
        plt.ylim(self._Position.zPos - dist * 1000.0, self._Position.zPos + dist * 1000.0)


if __name__ == '__main__':
    try:
        cm = create_map(OBJECT_TOPIC, GOAL_TOPIC, LASER_TOPIC, invertX=True)

    except rospy.ROSInterruptException:
        pass
