#!/usr/bin/env python

import sys

import math
# ==============================
#     Import Libraries
# ==============================
import rospy

sys.path.append('/home/josh/catkin_ws/src/autonomous_navigation')

from src.Lib import ScanSegment
# ==============================
#     Import Messages
# ==============================

import std_msgs.msg
from autonomous_navigation.msg import Position, ObjectsAtPosition
import message_filters
from object_detection.msg import Objects

# ==============================
#     Set Topic Variables
# ==============================

POS_TOPIC = '/summit_xl/position'
POS_WITH_LASER = '/summit_xl/PosWithLaser'
LASER_TOPIC = "/hokuyo_base/scan"

# ==============================
#       CAMERA VARIABLES
# ==============================
FIELD_OF_VIEW = 60.0  # (Degrees)
PIXEL_WIDTH = 640.0


# ==============================
#     get_objects class
# ==============================

class GetLaser:
    """Subscribes to topics to obtain and group together the data required for plotting
    the Scatter Plot
    """

    def __init__(self, output_top, las_top, pos_top, invertX=False, invertY=False):
        """Creates instance of GetObjects class.
        output_top = Topic for resulting msg
        las_top = Topic for Objects
        pos_top = Topic for Position of Robot
        invertX = Inverts generated coordinates in X plane
        invertY = Inverts generated coordinates in Y plane
        """

        # Create publisher and Init Node
        self._positionNode = rospy.Publisher(
            output_top, ObjectsAtPosition, queue_size=1)
        rospy.init_node('position_tracker', anonymous=True)

        # Set topic names
        self._laserTopic = las_top
        self._positionTopic = pos_top

        # Position and Laser variables
        self._position = None
        self._laserScan = None

        # Lists for publishing
        self._laserX, self._laserY = [], []

        # Sequence for header
        self._seq = 0

        # Begin subscribing
        self._get_scan()

    def _get_scan(self):
        """"Synchronises topics and subscribes when time stamp is same"""

        while not rospy.is_shutdown():
            try:
                position_sub = message_filters.Subscriber(self._positionTopic, Position)
                laser_sub = message_filters.Subscriber(self._laserTopic, Objects)

                ts = message_filters.ApproximateTimeSynchronizer(
                    [position_sub, laser_sub], 10, 0.1)
                ts.registerCallback(self._callback)
                rospy.spin()
            except rospy.ROSInterruptException:
                print("Shutting down...")
            except IOError:
                print("Shutting down...")

    def _callback(self, position, scan):

        # Set Current Position
        self._position = position

        # Calc Laser Coordinates
        scan_seg = ScanSegment(scan.angle_min, scan.angle_max, scan.range_max, scan.angle_increment,
                               llinc=math.radians(1))
        self._calc_laser_coord(scan_seg.getLaserRange(), scan_seg.getMaxDist())

        # Publish Coordinates
        self._publish()

    def _calc_laser_coord(self, laser_list, max_dist):
        """Calculates all coordinates of LaserScans given their distance and angle.
        """
        self._laserX, self._laserZ = [], []

        for scan in laser_list:
            ang = scan.angle + self._position.orientation

            dist = scan.distance
            if dist < max_dist:
                dist *= 1000.0  # Convert to mm
                if abs(ang) <= math.pi / 2:
                    x = math.sin(ang) * dist
                    z = math.cos(ang) * dist
                else:
                    if ang > 0:
                        x = (math.sin(ang) * dist)
                        z = (math.cos(ang) * dist)
                    else:
                        x = (math.sin(ang) * dist)
                        z = -(math.cos(ang) * dist)

                self._laserX.append(self._position.xPos + x * (-1 if self._invertX else 1))
                self._laserY.append(self._position.zPos + z * (-1 if self._invertY else 1))

    def _publish(self):
        """Create the message and publish to topic"""

        msg = ObjectsAtPosition(self._position, self._laserX,
                                self._laserY)
        msg.pos.header = std_msgs.msg.Header(stamp=rospy.Time.now(), seq=self._seq)

        self._seq += 1

        rate = rospy.Rate(30)
        self._positionNode.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        PT = GetLaser(POS_WITH_LASER, LASER_TOPIC, LASER_TOPIC, invertX=True)

    except rospy.ROSInterruptException:
        pass
