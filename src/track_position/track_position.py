#!/usr/bin/env python
import geometry_msgs.msg
import roslib
import rospy
import sensor_msgs.msg
import std_msgs.msg
from autonomous_navigation.msg import Position
from autonomous_navigation.msg import ScanAtPosition
from std_msgs.msg import Float32

from ScanSegment import ScanSegment


POS_TOPIC = '/summit_xl_controller/position'
LASER_TOPIC = '/summit_xl_a/front_laser/scan'


class position_tracker:

    def __init__(self,move,laserTopic):
        self.__positionNode = rospy.Publisher(move, ScanAtPosition,queue_size=1)
        rospy.init_node('position_tracker', anonymous=True)

        self.__laserTopic = laserTopic

        self.__xPos = 0.0
        self.__zPos = 0.0
        self.__orientation = 0.0

        self.__laserScan = None

        self.get_scan()

    def __track_pos(self):
        
        msg = ScanAtPosition()

        msg.pos.orientation = self.__orientation
        msg.pos.xPos = self.__xPos
        msg.pos.zPos = self.__zPos
        msg.scan = self.__laserScan.getLaserRange()
        msg.maxDist = self.__laserScan.getMaxDist()

        self.__positionNode.publish(msg)

    def laserCallback(self,scan):
        self.__laserScan = ScanSegment(scan.angle_min,scan.angle_max,scan.range_max,scan.angle_increment)

        self.__laserScan.setLaserRange(scan.ranges)
        self.__track_pos()

    def get_scan(self):
        "Subscribes to the LaserScan topic and calls the callback"

        rospy.Subscriber(self.__laserTopic, sensor_msgs.msg.LaserScan, self.laserCallback)
        rospy.spin()

if __name__ == '__main__':
    try:
        PT = position_tracker(POS_TOPIC,LASER_TOPIC)
        #rospy.spin()

    except rospy.ROSInterruptException:
                pass
