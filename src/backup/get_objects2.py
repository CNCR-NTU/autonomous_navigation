#!/usr/bin/env python

import math
import message_filters
# ==============================
#     Import Libraries
# ==============================
import rospy
import std_msgs.msg
from ScanSegment import ScanSegment
from autonomous_navigation.msg import Position, ObjectsAtPosition, MapData
from object_detection.msg import Objects
from sensor_msgs.msg import LaserScan

# ==============================
#     Import Messages
# ==============================

# ==============================
#     Set Topic Variables
# ==============================

POSITION_TOPIC = '/summit_xl/position'
LASER_TOPIC = "/hokuyo_base/scan"
CLASSIFIED_OBJECTS_TOPIC = "/ai/objects"

OUTPUT_TOPIC = '/summit_xl/positionWithObjects'


# ==============================
#     get_objects class
# ==============================


class GetObjects:
    """Subscribes to LaserScan, Classification Objects and Position. Calculates position of all objects at a given
    position. Publishes information to Topic
    """

    def __init__(self, output, laser, position, classified):
        """Creates instance of get_objects class.
        output = Topic for resulting msg
        laser = Topic for LaserScan
        position = Topic for Position of Robot
        classified = Topic for Classified Objects
        """

        # Create publisher and Init Node
        self._positionNode = rospy.Publisher(
            output, ObjectsAtPosition, queue_size=1)
        rospy.init_node('position_tracker', anonymous=True)

        # Set topic names
        self._laser_topic = laser
        self._position_topic = position
        self._object_topic = classified

        # Position and Object variables
        self._position = Position(orientation=0.0, xPos=0.0, zPos=0.0)
        self._laserScan = None
        self._classifiedObjects = None

        # Arrays For Results
        self._posX, self._posY = [], []
        self._laserX, self._laserY = [], []
        self._classes, self._objectX, self._objectY = [], [], []

        print("hi")

        self._get_objects()

    def _callback(self, position, laser, objects):
        print(objects)
        # Set Current Position
        self._position = position
        self._posX.append(position.xPos)
        self._posZ.append(position.zPos)

        # Calc Laser Coordinates
        laser_scan = ScanSegment(laser.angle_min, laser.angle_max, laser.range_max, laser.angle_increment,
                                 llinc=math.radians(1))
        self._calc_laser_coord(laser_scan.getLaserRange(), laser_scan.getMaxDist())

        # Calc Object Coordinates
        self._calc_object_coord(objects, 60, 640)

        self._publish_data()

    def _publish_data(self):
        """Create the message and publish to topic"""

        msg = MapData(std_msgs.msg.Header(stamp=rospy.Time.now())
                      , self._posX, self._posY,
                      self._laserX, self._laserY,
                      self._classes, self._objectX, self._objectY)

        rate = rospy.Rate(30)
        self._positionNode.publish(msg)
        rate.sleep()

    def _calc_laser_coord(self, laserList, maxDist):
        """Calculates all coordinates of LaserScans given their distance and angle.
        """
        self._laserX, self._laserZ = [], []

        # self._setLimits(maxDist)

        for scan in laserList:
            ang = scan.angle + self._position.orientation

            dist = scan.distance
            if dist < maxDist:
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

                self._laserX.append(self._position.xPos - x)  # * (-1 if self._invertX else 1))
                self._laserY.append(self._position.zPos + z)  # * (-1 if self._invertY else 1))

    def _calc_object_coord(self, obj_list, cam_fov, cam_width):

        for obj in obj_list:
            ang = (cam_width / 2 - obj.x) * (cam_fov / cam_width)

            x = math.sin(ang) * obj.z
            z = math.cos(ang) * obj.z

            self._classes.append(obj.name)
            self._objectX.append(self._position.xPos - x)
            self._objectY.append(self._position.zPos + z)

    def _pos_callback(self, data):

    def _get_objects(self):
        """"Subscribes to topics and calls callbacks"""

        while not rospy.is_shutdown():
            try:
                try:
                    rospy.Subscriber(self._position_topic, Position, self._pos_callback, queue_size=1)
                    # position_sub = message_filters.Subscriber(self._position_topic, Position)
                except:
                    print("No Position Data, is ", POSITION_TOPIC, " being published to")

                rospy.Subscriber(self._laser_topic, LaserScan, self._laser_callback, queue_size=1)
                # laser_sub = message_filters.Subscriber(self._laser_topic, LaserScan)
                rospy.Subscriber(self._object_topic, Objects, self._objects_callback, queue_size=1)
                # object_sub = message_filters.Subscriber(self._object_topic, Objects)
                ts = message_filters.ApproximateTimeSynchronizer(
                    [position_sub, laser_sub, object_sub], 10, 0.1)
                ts.registerCallback(self._callback)
                rospy.spin()
            except rospy.ROSInterruptException:
                print("Shutting down...")
            except IOError:
                print("Shutting down...")


if __name__ == '__main__':
    try:
        PT = GetObjects(OUTPUT_TOPIC, LASER_TOPIC, POSITION_TOPIC, CLASSIFIED_OBJECTS_TOPIC)

    except rospy.ROSInterruptException:
        pass
