#!/usr/bin/env python
import geometry_msgs.msg
import roslib
import rospy
import sensor_msgs.msg
import std_msgs.msg
from autonomous_navigation.msg import Position
from autonomous_navigation.msg import ScanAtPosition
from std_msgs.msg import Float32
import math


POS_TOPIC = '/summit_xl_controller/position'
LASER_TOPIC = '/summit_xl_a/front_laser/scan'


class pub_laser:

    def __init__(self):
        self.__positionNode = rospy.Publisher('/summit_xl_a/front_laser/scan', sensor_msgs.msg.LaserScan,queue_size=1)
        rospy.init_node('laser_pub', anonymous=True)


        self.LaserScan = sensor_msgs.msg.LaserScan()

        self.LaserScan.angle_max = math.pi/2
        self.LaserScan.angle_min = -(math.pi/2)

        self.LaserScan.range_min = 0.0
        self.LaserScan.range_max = 8.0
        self.LaserScan.angle_increment = math.radians(1)

        for i in range(0,360):
            self.LaserScan.ranges.append(5)

        rate = rospy.Rate(2) #Every 2 seconds

        while not rospy.is_shutdown():
            self.__positionNode.publish(self.LaserScan)
            rate.sleep()
        

if __name__ == '__main__':
    try:
        PT = pub_laser()
        #rospy.spin()

    except rospy.ROSInterruptException:
                pass
