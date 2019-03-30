#!/usr/bin/env python
import rospy
import roslib

import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Float32
from autonomous_navigation.msg import Position

POS_TOPIC = '/summit_xl_controller/position'

class position_tracker:

    def __init__(self,move):
        self.__positionNode = rospy.Publisher(move, Position,queue_size=1)
        rospy.init_node('position_tracker', anonymous=True)

        self.__xPos = Float32(float(0.0))
        self.__zPos = Float32(float(0.0))
        self.__orientation = Float32(float(0.0))

        self.__track_pos()

    def __track_pos(self):

        msg = Position()
        msg.orientation = 0.0
        msg.xPos = 0.0
        msg.zPos = 0.0

        r = rospy.Rate(10) #10hz

        while not rospy.is_shutdown():
            self.__positionNode.publish(msg)
            r.sleep()



if __name__ == '__main__':
    try:
        PT = position_tracker(POS_TOPIC)
        rospy.spin()

    except rospy.ROSInterruptException:
                pass