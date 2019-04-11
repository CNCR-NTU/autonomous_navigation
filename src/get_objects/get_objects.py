#!/usr/bin/env python

#==============================
#     Import Libraries 
#==============================
import roslib
import rospy
import math

from ScanSegment import ScanSegment

#==============================
#     Import Messages 
#==============================

import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
from autonomous_navigation.msg import Position
from autonomous_navigation.msg import ScanAtPosition

#==============================
#     Set Topic Variables 
#==============================

POS_TOPIC = '/summit_xl_controller/position'
POS_WITH_SCAN_TOPIC = '/summit_xl_controller/PostionWithLaser'
LASER_TOPIC = "/hokuyo_base/scan"

#==============================
#     get_objects class 
#==============================

class get_objects:
    """Subscribes to topics to obtain and group together the data required for plotting
    the Scatter Plot
    """

    def __init__(self, outputTopic, laserTopic,PosTopic):
        """Creates instance of get_objects class.
        outputTopic = Topic for resulting msg
        laserTopic = Topic for LaserScan
        PosTopic = Topic for Position of Robot
        """

        #Create publisher and Init Node
        self.__positionNode = rospy.Publisher(
            outputTopic, ScanAtPosition, queue_size=1)
        rospy.init_node('position_tracker', anonymous=True)

        #Set topic names
        self.__laserTopic = laserTopic
        self.__positionTopic = PosTopic

        #Postion and Laser variables
        self.__position = Position(0.0,0.0,0.0)
        self.__laserScan = None

        self.get_scan()

    def __publishData(self):
        """Create the message and publish to topic"""

        msg = ScanAtPosition(std_msgs.msg.Header(stamp=rospy.Time.now())
                ,self.__position,self.__laserScan.getLaserRange()
                ,self.__laserScan.getMaxDist())


        rate = rospy.Rate(30)  # Every 0.1 second(s)
        self.__positionNode.publish(msg)
        rate.sleep()

    def laserCallback(self, scan):
        """Processes data from LaserScan Topic"""

        self.__laserScan = ScanSegment(
            scan.angle_min, scan.angle_max, scan.range_max, scan.angle_increment, llinc=math.radians(1))

        self.__laserScan.setLaserRange(scan.ranges)
        self.__publishData()

    def positionCallback(self,pos):
        """Processes data from Postion Topic"""

        self.__position = pos

    def get_scan(self):
        "Subscribes to topics and calls callbacks"

        rospy.Subscriber(self.__positionTopic,Position,self.positionCallback,queue_size=1)

        rospy.Subscriber(self.__laserTopic,sensor_msgs.msg.LaserScan, self.laserCallback,queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    try:
        PT = get_objects(POS_WITH_SCAN_TOPIC, LASER_TOPIC,POS_TOPIC)

    except rospy.ROSInterruptException:
        pass
