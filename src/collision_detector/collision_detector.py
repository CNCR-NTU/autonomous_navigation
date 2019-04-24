#!/usr/bin/env python

# ==============================
#     Import Libraries
# ==============================

import sys

import math
import rospy

sys.path.append('/home/josh/catkin_ws/src/autonomous_navigation')

from src.Lib.ScanSegment import ScanSegment
from src.Lib.WarningLevel import WarnLVL
from object_detection.msg import Objects

# ==============================
#     Import Messages
# ==============================

import sensor_msgs.msg
from std_msgs.msg import Float32

# ==============================
#     Set Topic Variables 
# ==============================

LASER_TOPIC = "/summit_xl/front_laser/scan"
VELOCITY_TOPIC = "/summit_xl_control/cmd_vel"
OBJECT_TOPIC = "/ai/objects"
# ==============================
#   Set Collision Variables 
# ==============================

# Level of tolerance for anomalies (%)
LOT = 3

# Minimum disance for colision detection(M)
WARN_DISTANCE = 0.75
STOP_DISTANCE = 0.3


# ==============================
#   collision_detector class 
# ==============================

class collision_detector:
    """ A class used for processing information from a ROS topic LaserScan.
    The processed information is used to determine the distances of obejcts,
    and use these distances to stop should a collision occur.
    """

    # 0 Is path clear, 1 is Collsion, 2 is crashed
    _WarnLvls = [WarnLVL('\033[92m', 0.02, 'Path Clear'),
                 WarnLVL('\033[93m', 0.01, 'Object Detected, Slowing Down'),
                 WarnLVL('\033[91m', 0, 'Collision Imminent, Stopping Vehicle')]

    # Dimensions used for calculations
    _ROBOT_WIDTH = 0.7
    _SCANNER_TO_FRONT_DIST = 0.45

    def __init__(self, lasTop, objTop, velTop, wDis, sDis, tol):
        """Creates instance of collision_detector:
        lasTop = ROSTopic for laser
        velTop = ROSTopic for moving robot
        wDis = Object distance before warning (m)
        sDis = Object distance before stopping (m)
        tol = Tolerance in the data"""

        # Topic
        self._laserTopic = lasTop
        self._objectTopic = objTop

        # Specified distances for slowing and stopping
        self._warnDis = wDis
        self._stopDis = sDis

        self._frontFoV = ScanSegment(-self.get_FoV(), self.get_FoV())
        self._tolerance = tol / 100.0

        # Array of Laser values
        self._WarnLVL = 0

        self._objects = []

        rospy.init_node('laser', anonymous=True)
        self._velocityNode = rospy.Publisher(
            velTop, Float32, queue_size=1)

        self._get_scan()

    def get_FoV(self):
        """Calculates FoV angle to detect objects at specified range"""

        return math.atan((self._ROBOT_WIDTH / 2) / (self._SCANNER_TO_FRONT_DIST + self._stopDis))

    def _setSpeed(self):
        """Automatically sets the speed of the robot based on obstacles encountered"""

        speed = self._WarnLvls[self._WarnLVL].get_velocity()
        self._velocityNode.publish(Float32(speed))

    def _laser_callback(self, scan):
        """Called upon receiving new scan, handles scan information"""

        # Set scan angle increment and laser range
        self._frontFoV.setAng(scan.angle_increment)
        self._frontFoV.setLaserMax(scan.range_max)

        # get array indices for min and max of range
        rangeMin = int((len(scan.ranges) / 2) + self._frontFoV.minAngScans())
        rangeMax = int((len(scan.ranges) / 2) + self._frontFoV.maxAngScans())

        # Fill array with values from range
        self._frontFoV.setLaserRange(scan.ranges[rangeMin:rangeMax])

        self._checkScan()

        self._setSpeed()

        self._print_info()

    def _object_callback(self, data):
        self._objects = []

        self._objects = data.objects

    def _checkScan(self):
        """Check stopping and warning distances from the given scan. Test against tolerance level"""

        lvl2Count, lvl3Count = 0, 0

        objectDecteted = self._checkObjects()

        for Dist in self._frontFoV.getAbsRange():

            if Dist - self._SCANNER_TO_FRONT_DIST < self._stopDis:
                lvl3Count += 1
                if lvl3Count > self._frontFoV.totalScans() * self._tolerance:
                    self._WarnLVL = 2
                    break

            if Dist - self._SCANNER_TO_FRONT_DIST < self._warnDis:
                lvl2Count += 1
                if lvl2Count > self._frontFoV.totalScans() * self._tolerance:
                    self._WarnLVL = 1
                    break

            if not objectDecteted:
                self._WarnLVL = 0

    def _checkObjects(self):
        for obj in self._objects:
            if obj.z < self._stopDis:
                self._WarnLVL = 2
                return True
            if obj.z < self._warnDis:
                self._WarnLVL = 1
                return True

        return False

    def _print_obj_list(self):
        print('\033[1mDetected Objects:\033[0m')
        for obj in self._objects:
            print("{}: {} m").format(obj.name, obj.z)
        self._objects = []

    def _print_info(self):
        """Print system info for user"""

        print('\n\033[4m' + '\033[1m' + 'Summit_XL Collision Detection' + '\033[0m')
        print('\nWarn Distance: {}m \t Stop Distance: {}m'.format(self._warnDis, self._stopDis))
        print('\033[1mLaser Values\033[0m')
        print('Average Distance: {} \t Minimum Distance: {}'.format(
            self._frontFoV.getAbsAvg() - self._SCANNER_TO_FRONT_DIST,
            self._frontFoV.getAbsMin() - self._SCANNER_TO_FRONT_DIST))
        self._print_obj_list()
        print(self._WarnLvls[self._WarnLVL].get_Warning())
        print('\n---------------------------------------------------------------')

    def _get_scan(self):
        """Subscribes to the LaserScan topic and calls the callback"""
        rospy.Subscriber(self._laserTopic,
                         sensor_msgs.msg.LaserScan, self._laser_callback)

        rospy.Subscriber(self._objectTopic, Objects, self._object_callback)
        rospy.spin()


if __name__ == '__main__':
    try:
        L = collision_detector(LASER_TOPIC, OBJECT_TOPIC, VELOCITY_TOPIC,
                               WARN_DISTANCE, STOP_DISTANCE, LOT)

    except rospy.ROSInterruptException:
        pass
