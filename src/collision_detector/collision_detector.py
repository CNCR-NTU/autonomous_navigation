#!/usr/bin/env python
import rospy
import roslib

import math
import numpy

import geometry_msgs.msg
import sensor_msgs.msg

from warning_level import WarnLVL
from ScanSegment import ScanSegment

LASER_TOPIC = "/hokuyo_base/scan"
VELOCITY_TOPIC = "/summit_xl_control/cmd_vel"

# Level of tolerance for anomalies (%)
LOT = 3

# Minimum disance for colision detection(M)
WARN_DISTANCE = 0.75
STOP_DISTANCE = 0.3


class collision_detector:
    """ A class used for processing information from a ROS topic LaserScan.
    The processed information is used to determine the distances of obejcts,
    and use these distances to stop should a collision occur.
    """

    # 0 Is path clear, 1 is Collsion, 2 is crashed
    __WarnLvls = [WarnLVL('\033[92m', 0.35, 'Path Clear'),
                  WarnLVL('\033[93m', 0.2, 'Object Detected, Slowing Down'),
                  WarnLVL('\033[91m', 0, 'Collision Imminent, Stopping Vehicle')]

    # Dimensions used for calculations
    __ROBOT_WIDTH = 0.7
    __SCANNER_TO_FRONT_DIST = 0.45

    def __init__(self, lasTop, velTop, wDis, sDis, tol):
        """Creates instance of collision_detector:
        lasTop = ROSTopic for laser
        velTop = ROSTopic for moving robot
        wDis = Object distance before warning (m)
        sDis = Object distance before stopping (m)
        tol = Tolerance in the data"""

        # Topic
        self.__laserTopic = lasTop
        # Specified distances for slowing and stopping
        self.__warnDis = wDis
        self.__stopDis = sDis

        self.__frontFoV = ScanSegment(-self.get_FoV(), self.get_FoV())
        self.__tolerance = tol/100.0

        # Array of Laser values
        self.__WarnLVL = 0

        rospy.init_node('laser', anonymous=True)
        self.__velocityNode = rospy.Publisher(
            velTop, geometry_msgs.msg.Twist, queue_size=1)

        self.__get_scan()

    def get_FoV(self):
        "Calculates FoV angle to detect objects at specified range"

        return math.atan((self.__ROBOT_WIDTH/2)/(self.__SCANNER_TO_FRONT_DIST+self.__stopDis))

    def __move(self):
        "Automatically moves the robot by publishing to the cmd topic"

        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = self.__WarnLvls[self.__WarnLVL].get_velocity()
        self.__velocityNode.publish(cmd)

    def __callback(self, scan):
        "Called upon recieving new scan, handles scan information"

        # Set scan angle increment and laser range
        self.__frontFoV.setAng(scan.angle_increment)
        self.__frontFoV.setLaserMax(scan.range_max)

        # get array indices for min and max of range
        rangeMin = int((len(scan.ranges)/2) + self.__frontFoV.minAngScans())
        rangeMax = int((len(scan.ranges)/2) + self.__frontFoV.maxAngScans())

        # Fill array with values from range
        self.__frontFoV.setLaserRange(scan.ranges[rangeMin:rangeMax])

        self.__checkScan()

        self.__move()

        self.__printInfo()

    def __checkScan(self):
        "Check stopping and warning distances from the given scan. Test against tolerance level"

        lvl2Count, lvl3Count = 0, 0

        for Dist in self.__frontFoV.getAbsRange():

            if Dist-self.__SCANNER_TO_FRONT_DIST < self.__stopDis:
                lvl3Count += 1
                if lvl3Count > self.__frontFoV.totalScans()*self.__tolerance:
                    self.__WarnLVL = 2
                    break

            if Dist-self.__SCANNER_TO_FRONT_DIST < self.__warnDis:
                lvl2Count += 1
                if lvl2Count > self.__frontFoV.totalScans()*self.__tolerance:
                    self.__WarnLVL = 1
                    break

            self.__WarnLVL = 0

    def __printInfo(self):
        "Print system info for user"

        print '\n\033[4m' + '\033[1m' + 'Summit_XL Collision Detection' + '\033[0m'
        print '\nWarn Distance: {}m \t Stop Distance: {}m'.format(self.__warnDis, self.__stopDis)
        print 'Average Distance: {} \t Minimum Distance: {}'.format(self.__frontFoV.getAbsAvg(), self.__frontFoV.getAbsMin())
        print self.__WarnLvls[self.__WarnLVL].get_Warning()
        print('\n---------------------------------------------------------------')

    def __get_scan(self):
        "Subscribes to the LaserScan topic and calls the callback"

        rospy.Subscriber(self.__laserTopic,
                         sensor_msgs.msg.LaserScan, self.__callback)
        rospy.spin()


if __name__ == '__main__':
    try:
        L = collision_detector(LASER_TOPIC, VELOCITY_TOPIC,
                          WARN_DISTANCE, STOP_DISTANCE, LOT)

    except rospy.ROSInterruptException:
        pass
