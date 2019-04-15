#!/usr/bin/env python

import math
import message_filters
# ==============================
#     Import Libraries
# ==============================
import rospy
import std_msgs.msg
from autonomous_navigation.msg import Position, ObjectsAtPosition
from object_detection.msg import Objects

# ==============================
#     Import Messages
# ==============================

# ==============================
#     Set Topic Variables
# ==============================

POS_TOPIC = '/summit_xl/position'
POS_WITH_OBJ = '/summit_xl/PosWithObj'
OBJECTS_TOPIC = "/ai/objects"

# ==============================
#       CAMERA VARIABLES
# ==============================
FIELD_OF_VIEW = 60.0  # (Degrees)
PIXEL_WIDTH = 640.0


# ==============================
#     GetObjects class
# ==============================

class GetObjects:
    """Subscribes to topics to obtain and group together the data required for plotting
    the Scatter Plot
    """

    def __init__(self, output_top, object_top, pos_top, invertX=False, invertY=False):
        """Creates instance of GetObjects class.
        output_top = Topic for resulting msg
        object_top = Topic for Objects
        pos_top = Topic for Position of Robot
        invertX = Inverts generated coordinates in X plane
        invertY = Inverts generated coordinates in Y plane
        """

        # Create publisher and Init Node
        self._positionNode = rospy.Publisher(
            output_top, ObjectsAtPosition, queue_size=1)
        rospy.init_node('get_objects', anonymous=True)

        # Set topic names
        self._objectTopic = object_top
        self._positionTopic = pos_top

        # Position and Laser variables
        self._position = None
        self._Objects = None

        # Lists for publishing
        self._classes, self._objectX, self._objectY = [], [], []

        # Sequence for header
        self._seq = 0

        # Inverters
        self._invertX = invertX
        self._invertY = invertY

        # Begin subscribing
        self._get_objects()

    def _get_objects(self):
        """"Synchronises topics and subscribes when time stamp is same"""

        while not rospy.is_shutdown():
            try:
                position_sub = message_filters.Subscriber(self._positionTopic, Position)
                object_sub = message_filters.Subscriber(self._objectTopic, Objects)

                ts = message_filters.ApproximateTimeSynchronizer(
                    [position_sub, object_sub], 10, 0.1)
                ts.registerCallback(self._callback)
                rospy.spin()

            except rospy.ROSInterruptException:
                print("Shutting down...")
            except IOError:
                print("Shutting down...")

    def _callback(self, position, data):
        # Set Current Position
        self._position = position

        # Calc Object Coordinates
        self._calc_object_coord(data.objects)

        # Publish Coordinates
        self._publish()

    def _calc_object_coord(self, obj_list):
        self._classes, self._objectX, self._objectY = [], [], []

        for obj in obj_list:
            ang = math.radians(
                (PIXEL_WIDTH / 2.0 - (obj.x + obj.w / 2)) * (FIELD_OF_VIEW / PIXEL_WIDTH)) + self._position.orientation

            dist = obj.z * 1000  # Get in mm

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

            self._classes.append(obj.name)
            self._objectX.append(self._position.xPos + x * (-1 if self._invertX else 1))
            self._objectY.append(self._position.zPos + z * (-1 if self._invertY else 1))

    def _publish(self):
        """Create the message and publish to topic"""

        msg = ObjectsAtPosition(self._position, self._classes,
                                self._objectX,
                                self._objectY)
        msg.pos.header = std_msgs.msg.Header(stamp=rospy.Time.now(), seq=self._seq)

        self._seq += 1

        rate = rospy.Rate(30)
        self._positionNode.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        PT = GetObjects(POS_WITH_OBJ, OBJECTS_TOPIC, POS_TOPIC, invertX=True)

    except rospy.ROSInterruptException:
        pass
