#!/usr/bin/env python
import rospy
import roslib

import math
import numpy

import geometry_msgs.msg
import sensor_msgs.msg



LASER_TOPIC = "/hokuyo_base/scan"
#Field of view that laser is checking in each direction(DEG)
FIELD_OF_VIEW = 50
#Level of tolerance for anomalies (%)
LOT = 3
#Minimum disance for colision detection(M)
WARN_DISTANCE = 1.2
STOP_DISTANCE = 0.75

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

    COLLISION_LEVEL = ['Path Clear','Collision Imminent','Crashed']
    WARN_COLOURS = ['\033[92m','\033[93m','\033[91m']
    VELOCITY_LEVEL = [0.35,0.2,0]

    def __init__(self,_Topic,_warnDis,_stopDis,_FoV,_LoT):
        self.Topic = _Topic
        self.warnDis = _warnDis
        self.stopDis = _stopDis
        self.FoV = _FoV
        self.tolerance = _LoT/100.0

        #Array of Laser values
        self.LaserRange = None
        self.WarnLVL = 0

        self.avgDist = None

        rospy.init_node('laser', anonymous=True)
        self.Velocity = rospy.Publisher('/summit_xl_control/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

        self.get_scan()


    def callback(self,scan):
        
        #get array indices for min and max of range
        rangeMin = int((len(scan.ranges)/2) - (Deg2Rad(self.FoV))/scan.angle_increment)
        rangeMax = int((len(scan.ranges)/2) + (Deg2Rad(self.FoV))/scan.angle_increment)

        #Fill array with values from range
        self.LaserRange = scan.ranges[rangeMin:rangeMax]

        self.checkScan()
        self.getFrontDistance(scan.range_max)
        self.move()

        self.printInfo()
      
    def move(self):
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = self.VELOCITY_LEVEL[self.WarnLVL]
        self.Velocity.publish(cmd)

    def checkScan(self):
        lvl2Count = 0
        lvl3Count = 0

        for i in self.LaserRange:
            if i < self.stopDis:
                lvl3Count += 1
                if lvl3Count > len(self.LaserRange)*self.tolerance:
                    self.WarnLVL = 2
                    break

            if i < self.warnDis:
                lvl2Count += 1
                if lvl2Count > len(self.LaserRange)*self.tolerance:
                    self.WarnLVL = 1
                    break

                self.WarnLVL = 0           

    def getFrontDistance(self,maxDist):
        avg = 0
        count = 0
        for i in self.LaserRange:#[(len(self.LaserRange)/2-30):(len(self.LaserRange)/2+30)]:
            if i < maxDist:
                avg += i
                count += 1

        if count != 0:
            self.avgDist = avg/count



    def printInfo(self):
        print '\n\033[4m' + '\033[1m' + 'Summit_XL Collision Detection' + '\033[0m'
        print '\nWarn Distance: {}m \t Stop Distance: {}m'.format(self.warnDis,self.stopDis)
        print 'Average Distance: {}'.format(self.avgDist )
        print self.WARN_COLOURS[self.WarnLVL] + self.COLLISION_LEVEL[self.WarnLVL] + '\033[0m'
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