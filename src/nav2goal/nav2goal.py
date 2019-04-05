#!/usr/bin/env python

#==============================
#     Import Libraries 
#==============================
import roslib
import rospy
import math

#==============================
#     Import Messages 
#==============================

import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
from autonomous_navigation.msg import Position
from autonomous_navigation.msg import ScanAtPosition

from turtlesim.msg import Pose

#==============================
#     Set Topic Variables 
#==============================

POS_TOPIC = '/turtle1/pose'
GOAL_TOPIC = '/turtle1/goal'
CMD_TOPIC = '/turtle1/cmd_vel'

#==============================
#     nav2goal class 
#==============================


class nav2goal:
    """Calculates path to navigate from current position to the goal
    """
    
    def __init__(self,posTop,goalTop,cmdTop):
        rospy.init_node('nav2goal')

        self.__positionTopic = posTop 
        self.__goalTopic = goalTop

        self.__cmdNode = rospy.Publisher(cmdTop,geometry_msgs.msg.Twist,queue_size=1)

        self.__summitPos = Position()
        self.__goalPos = Position()

        self.get_data()

    def get_data(self):
        rospy.Subscriber(self.__positionTopic,Pose,self.posCallback)
        rospy.Subscriber(self.__goalTopic,Position,self.goalCallback)
        rospy.spin()

    def posCallback(self,data):
        self.__summitPos = Position(data.theta,data.x,data.y)


    def goalCallback(self,data):
        self.__goalPos = data


        print "\nPosition:"
        print self.__summitPos
        print "\nGoal:"
        print self.__goalPos
        print "\n--------------------"

#Rotate to Goal
#Move to Goal


if __name__ == '__main__':
    try:
        nav = nav2goal(POS_TOPIC,GOAL_TOPIC,CMD_TOPIC)

    except rospy.ROSInterruptException:
        pass