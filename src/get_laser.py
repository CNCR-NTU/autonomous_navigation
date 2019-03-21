#!/usr/bin/env python
import rospy
import roslib

import math
import numpy

import geometry_msgs.msg
import sensor_msgs.msg

from warning_level import WarnLVL
from LaserPoint import LaserPoint

LASER_TOPIC = "/hokuyo_base/scan"
#Field of view that laser is checking in each direction(DEG)
FIELD_OF_VIEW = 50
#Level of tolerance for anomalies (%)
LOT = 3
#Minimum disance for colision detection(M)
WARN_DISTANCE = 0.75
STOP_DISTANCE = 0.3

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class laser_scanner:
    # 0 Is path clear, 1 is Collsion, 2 is crashed
    WarnLvls = [WarnLVL('\033[92m',0.35,'Path Clear'),
                WarnLVL('\033[93m',0.2,'Object Detected, Slowing Down'),
                WarnLVL('\033[91m',0,'Collision Imminent, Stopping Vehicle')]

    ROBOT_WIDTH = 0.7
    SCANNER_TO_FRONT_DIST = 0.45

    def __init__(self,top,wDis,sDis,fov,tol):
        self.Topic = top
        self.warnDis = wDis
        self.stopDis = sDis
        self.FoV = self.get_FoV()
        self.tolerance = tol/100.0

        #Array of Laser values
        self.FrontLaserRange = []
        self.WarnLVL = 0

        self.avgDist = None
        self.minDist = None
        self.angleInc = None

        rospy.init_node('laser', anonymous=True)
        self.Velocity = rospy.Publisher('/summit_xl_control/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

        self.get_scan()

    def get_FoV(self):
        "Calculates FoV angle to detect objects at specified range"

        return math.atan((self.ROBOT_WIDTH/2)/(self.SCANNER_TO_FRONT_DIST+self.stopDis))

    def callback(self,scan):
        #Get scan angle increment and scan range
        self.angleInc = scan.angle_increment
        self.scanRange = scan.range_max

        #get array indices for min and max of range
        rangeMin = int((len(scan.ranges)/2) - self.FoV/self.angleInc)
        rangeMax = int((len(scan.ranges)/2) + self.FoV/self.angleInc)

        #Fill array with values from range
        i = 0
        self.FrontLaserRange = []


        for j in range(rangeMin,rangeMax):
            self.FrontLaserRange.append(LaserPoint(scan.ranges[j],-self.FoV+(i*self.angleInc)))
            i += 1
        
        self.checkScan()
        self.getFrontDistance()
        self.move()

        self.printInfo()
      
    def move(self):
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = self.WarnLvls[self.WarnLVL].get_velocity()
        self.Velocity.publish(cmd)

    def calcAngle(self):
        #Scans from right to left
        Angle = abs(self.FoV - self.angleInc)
        return Angle

    def checkScan(self):
        lvl2Count = 0
        lvl3Count = 0

        for i in self.FrontLaserRange:
            
            if i.Abs_Dist()-self.SCANNER_TO_FRONT_DIST < self.stopDis:
                lvl3Count += 1
                if lvl3Count > len(self.FrontLaserRange)*self.tolerance:
                    self.WarnLVL = 2
                    break

            if i.Abs_Dist()-self.SCANNER_TO_FRONT_DIST < self.warnDis:
                lvl2Count += 1
                if lvl2Count > len(self.FrontLaserRange)*self.tolerance:
                    self.WarnLVL = 1
                    break

            self.WarnLVL = 0           

    def getFrontDistance(self):
        avg = 0
        count = 0
        self.minDist = self.scanRange

        for scan in self.FrontLaserRange:
            if scan.Abs_Dist() < self.scanRange:
                avg += scan.Abs_Dist()
                count += 1

                if scan.Abs_Dist() < self.minDist:
                    self.minDist = scan.Abs_Dist()

        if count != 0:
            self.avgDist = avg/count



    def printInfo(self):
        print '\n\033[4m' + '\033[1m' + 'Summit_XL Collision Detection' + '\033[0m'
        print '\nWarn Distance: {}m \t Stop Distance: {}m'.format(self.warnDis,self.stopDis)
        print 'Average Distance: {} \t Minimum Distance: {}'.format(self.avgDist,self.minDist )
        print self.WarnLvls[self.WarnLVL].get_Warning()
        print('\n---------------------------------------------------------------')

    
    def get_scan(self):
        rospy.Subscriber(self.Topic, sensor_msgs.msg.LaserScan, self.callback)
        rospy.spin()


def Deg2Rad(D):
    return D*(math.pi/180)

if __name__ == '__main__':
    try:
        L = laser_scanner(LASER_TOPIC,WARN_DISTANCE,STOP_DISTANCE,FIELD_OF_VIEW,LOT)

    except rospy.ROSInterruptException:
        pass