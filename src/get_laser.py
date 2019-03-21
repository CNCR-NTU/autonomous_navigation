#!/usr/bin/env python
import rospy
import roslib

import math
import numpy

import geometry_msgs.msg
import sensor_msgs.msg

from warning_level import WarnLVL
from LaserPoint import LaserPoint
from ScanSegment import ScanSegment

LASER_TOPIC = "/hokuyo_base/scan"
VELOCITY_TOPIC = "/summit_xl_control/cmd_vel"

#Level of tolerance for anomalies (%)
LOT = 3

#Minimum disance for colision detection(M)
WARN_DISTANCE = 0.75
STOP_DISTANCE = 0.3

class laser_scanner:
    # 0 Is path clear, 1 is Collsion, 2 is crashed
    WarnLvls = [WarnLVL('\033[92m',0.35,'Path Clear'),
                WarnLVL('\033[93m',0.2,'Object Detected, Slowing Down'),
                WarnLVL('\033[91m',0,'Collision Imminent, Stopping Vehicle')]

    #Dimensions used for calculations
    ROBOT_WIDTH = 0.7
    SCANNER_TO_FRONT_DIST = 0.45


    def __init__(self,lasTop,velTop,wDis,sDis,tol):

        #Topic
        self.laserTopic = lasTop
        #Specified distances for slowing and stopping
        self.warnDis = wDis
        self.stopDis = sDis

        self.frontFoV = ScanSegment(-self.get_FoV(),self.get_FoV())
        self.tolerance = tol/100.0

        #Array of Laser values
        self.WarnLVL = 0

        rospy.init_node('laser', anonymous=True)
        self.velocityNode = rospy.Publisher(velTop, geometry_msgs.msg.Twist,queue_size=1)

        self.get_scan()


    def get_FoV(self):
        "Calculates FoV angle to detect objects at specified range"

        return math.atan((self.ROBOT_WIDTH/2)/(self.SCANNER_TO_FRONT_DIST+self.stopDis))

    def move(self):
        "Automatically moves the robot by publishing to the cmd topic"

        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = self.WarnLvls[self.WarnLVL].get_velocity()
        self.velocityNode.publish(cmd)


    def callback(self,scan):
        "Called upon recieving new scan, handles information"
        
        #Set scan angle increment and laser range
        self.frontFoV.setAng(scan.angle_increment)
        self.frontFoV.setLaserMax(scan.range_max)

        #get array indices for min and max of range
        rangeMin = int((len(scan.ranges)/2) + self.frontFoV.minAngScans())
        rangeMax = int((len(scan.ranges)/2) + self.frontFoV.maxAngScans())

        #Fill array with values from range
        self.frontFoV.setLaserRange(scan.ranges[rangeMin:rangeMax])

        self.checkScan()

        self.move()

        self.printInfo()

    def checkScan(self):
        "Check stopping and warning distances from the given scan. Test against tolerance level"

        lvl2Count,lvl3Count = 0,0

        for Dist in self.frontFoV.getAbsRange():
            
            if Dist-self.SCANNER_TO_FRONT_DIST < self.stopDis:
                lvl3Count += 1
                if lvl3Count > self.frontFoV.totalScans()*self.tolerance:
                    self.WarnLVL = 2
                    break

            if Dist-self.SCANNER_TO_FRONT_DIST < self.warnDis:
                lvl2Count += 1
                if lvl2Count > self.frontFoV.totalScans()*self.tolerance:
                    self.WarnLVL = 1
                    break

            self.WarnLVL = 0           


    def printInfo(self):
        "Print system info for user"

        print '\n\033[4m' + '\033[1m' + 'Summit_XL Collision Detection' + '\033[0m'
        print '\nWarn Distance: {}m \t Stop Distance: {}m'.format(self.warnDis,self.stopDis)
        print 'Average Distance: {} \t Minimum Distance: {}'.format(self.frontFoV.getAbsAvg(),self.frontFoV.getAbsMin())
        print self.WarnLvls[self.WarnLVL].get_Warning()
        print('\n---------------------------------------------------------------')

    
    def get_scan(self):
        "Subscribes to the LaserScan topic and calls the callback"

        rospy.Subscriber(self.laserTopic, sensor_msgs.msg.LaserScan, self.callback)
        rospy.spin()


if __name__ == '__main__':
    try:
        L = laser_scanner(LASER_TOPIC,VELOCITY_TOPIC,WARN_DISTANCE,STOP_DISTANCE,LOT)

    except rospy.ROSInterruptException:
        pass