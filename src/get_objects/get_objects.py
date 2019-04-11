#!/usr/bin/env python

# ==============================
#     Import Libraries 
# ==============================
import roslib
import rospy
import math

from ScanSegment import ScanSegment

# ==============================
#     Import Messages 
# ==============================

import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
from autonomous_navigation.msg import Position
from autonomous_navigation.msg import ScanAtPosition

# ==============================
#     Set Topic Variables 
# ==============================

POS_TOPIC = '/summit_xl_controller/position'
POS_WITH_SCAN_TOPIC = '/summit_xl_controller/PostionWithLaser'
LASER_TOPIC = "/hokuyo_base/scan"


# ==============================
#     get_objects class 
# ==============================

class get_objects:
    """Subscribes to topics to obtain and group together the data required for plotting
    the Scatter Plot
    """

    def __init__(self, outputTopic, laserTopic, PosTopic):
        """Creates instance of get_objects class.
        outputTopic = Topic for resulting msg
        laserTopic = Topic for LaserScan
        PosTopic = Topic for Position of Robot
        """

        # Create publisher and Init Node
        self._positionNode = rospy.Publisher(
            outputTopic, ScanAtPosition, queue_size=1)
        rospy.init_node('position_tracker', anonymous=True)

        # Set topic names
        self._laserTopic = laserTopic
        self._positionTopic = PosTopic

        # Postion and Laser variables
        self._position = Position(0.0, 0.0, 0.0)
        self._laserScan = None

        self.get_scan()

    def _publishData(self):
        """Create the message and publish to topic"""

        msg = ScanAtPosition(std_msgs.msg.Header(stamp=rospy.Time.now())
                             , self._position, self._laserScan.getLaserRange()
                             , self._laserScan.getMaxDist())

        rate = rospy.Rate(30)  # Every 0.1 second(s)
        self._positionNode.publish(msg)
        rate.sleep()

    def laserCallback(self, scan):
        """Processes data from LaserScan Topic"""

        self._laserScan = ScanSegment(
            scan.angle_min, scan.angle_max, scan.range_max, scan.angle_increment, llinc=math.radians(1))

        self._laserScan.setLaserRange(scan.ranges)
        self._publishData()

    def positionCallback(self, pos):
        """Processes data from Postion Topic"""

        self._position = pos

    def get_scan(self):
        "Subscribes to topics and calls callbacks"

        rospy.Subscriber(self._positionTopic, Position, self.positionCallback, queue_size=1)

        rospy.Subscriber(self._laserTopic, sensor_msgs.msg.LaserScan, self.laserCallback, queue_size=1)
        rospy.spin()


if ___name___ == '___main___':
    try:
        PT = get_objects(POS_WITH_SCAN_TOPIC, LASER_TOPIC, POS_TOPIC)

    except rospy.ROSInterruptException:
        pass
