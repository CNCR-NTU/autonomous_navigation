#!/usr/bin/env python
import geometry_msgs.msg
import roslib
import rospy
import sensor_msgs.msg
import std_msgs.msg
from autonomous_navigation.msg import Position
from std_msgs.msg import Float32

POS_TOPIC = '/summit_xl_controller/position'

class position_tracker:

    def __init__(self,move):
        self.__positionNode = rospy.Publisher(move, Position,queue_size=1)
        rospy.init_node('position_tracker', anonymous=True)

        self.__xPos = 1.0
        self.__zPos = 1.0
        self.__orientation = 0.0

        self.__track_pos()

    def __track_pos(self):
        
        msg = Position()
        r = rospy.Rate(30) #30hz
        while not rospy.is_shutdown():
            msg.orientation = self.__orientation
            msg.xPos = self.__xPos
            msg.zPos = self.__zPos
            self.__xPos += 0.1
            self.__positionNode.publish(msg)
            r.sleep()



if __name__ == '__main__':
    try:
        PT = position_tracker(POS_TOPIC)
        #rospy.spin()

    except rospy.ROSInterruptException:
                pass
