#!/usr/bin/env python
import geometry_msgs.msg
import roslib
import rospy
import sensor_msgs.msg
import std_msgs.msg
from autonomous_navigation.msg import Position
from autonomous_navigation.msg import ScanAtPosition
import math

from ScanSegment import ScanSegment

POS_TOPIC = '/summit_xl_controller/position'
LASER_TOPIC = "/hokuyo_base/scan"


class position_tracker:

    def __init__(self, move, laserTopic):
        self.__positionNode = rospy.Publisher(
            move, ScanAtPosition, queue_size=1)
        rospy.init_node('position_tracker', anonymous=True)

        self.__laserTopic = laserTopic

        self.__position = Position(0.0,0.0,0.0)

        self.__laserScan = None

        self.get_scan()

    def __track_pos(self):

        msg = ScanAtPosition()
        msg.header.stamp = rospy.Time.now()
        msg.pos = self.__position
        msg.scan = self.__laserScan.getLaserRange()
        msg.maxDist = self.__laserScan.getMaxDist()

        rate = rospy.Rate(0.5)  # Every 2 seconds
        self.__positionNode.publish(msg)
        rate.sleep()

    def laserCallback(self, scan):
        self.__laserScan = ScanSegment(
            scan.angle_min, scan.angle_max, scan.range_max, scan.angle_increment, llinc=math.radians(1))

        self.__laserScan.setLaserRange(scan.ranges)
        self.__track_pos()

    def get_scan(self):
        "Subscribes to the LaserScan topic and calls the callback"

        rospy.Subscriber(self.__laserTopic,
                         sensor_msgs.msg.LaserScan, self.laserCallback,queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    try:
        PT = position_tracker(POS_TOPIC, LASER_TOPIC)
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass
