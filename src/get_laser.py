#!/usr/bin/env python
import rospy
import roslib

import math

import geometry_msgs.msg
import sensor_msgs.msg


LASER_TOPIC = "/hokuyo_base/scan"
#Field of view that laser is checking (DEG)
FIELD_OF_VIEW = 100
#Level of tolerance for anomalies (%)
LOT = 5
#Minimum disance for colision detection(M)
MIN_DISTANCE = 0.7

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

    def __init__(self,_Topic,_minDis,_FoV,_LoT):
        self.Topic = _Topic
        self.minDistance = _minDis
        self.FoV = _FoV
        self.tolerance = _LoT

        self.Collision_Imminent = None

        self.get_scan()


    def callback(self,scan):

        #get array indices for min and max of range
        rangeMin = int((len(scan.ranges)/2) - (Deg2Rad(self.FoV)/2)/scan.angle_increment)
        rangeMax = int((len(scan.ranges)/2) + (Deg2Rad(self.FoV)/2)/scan.angle_increment)

        LaserRange = scan.ranges[rangeMin:rangeMax]

        print '\nTotal Scans: {} \t Angle Increment: {}'.format(len(scan.ranges),scan.angle_increment)
        print '\nRange Min: {} \t Range Mid: {} \t Range Max: {}'.format(rangeMin,len(scan.ranges)/2,rangeMax)
        print LaserRange[0:10]
        print len(LaserRange)

        for i in LaserRange:
            if i < self.minDistance:
                print bcolors.WARNING + "Colision!!!" + bcolors.ENDC
                self.Collision_Imminent = True
                break

        print('\n---------------------------------------------------------------')    

    def get_scan(self):
        rospy.init_node('laser', anonymous=True)
        rospy.Subscriber(self.Topic, sensor_msgs.msg.LaserScan, self.callback)

        rospy.spin()


def Deg2Rad(D):
    return D*(math.pi/180)

if __name__ == '__main__':
    try:
        L = laser_scanner(LASER_TOPIC,MIN_DISTANCE,FIELD_OF_VIEW,LOT)

    except rospy.ROSInterruptException:
        pass