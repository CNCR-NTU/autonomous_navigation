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

#0.6
#9.5

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


        self.__atGoal = False
        self.__isAligned = False

        self.__angleMin = 0
        self.__angleMax = 2*math.pi

        self.__goalAngle = 0.0

        self.get_data()

    def get_data(self):
        rospy.Subscriber(self.__positionTopic,Pose,self.posCallback)
        rospy.Subscriber(self.__goalTopic,Position,self.goalCallback)
        rospy.spin()

    def posCallback(self,data):
        self.__summitPos = Position((data.theta+math.pi*2)%(math.pi*2),data.x,data.y)


    def goalCallback(self,data):
        #Check Goal has not changed
        if self.__goalPos != data:
            self.__goalPos = data
            self.__isAligned = False
            
        else:
            if not self.__atGoal:
                print '\n-------------------------------------------'
                print '\n\033[4m\033[1mNavigating to Position\033[0m\n'
                if not self.__isAligned:
                    self.__rotateToPosition()
                else:
                    self.__moveToPosition()         
            else:
                print '\n\033[4m\033[1mGoal Reached\033[0m\n'        


    def __calcAngle(self):
        Dx = self.__goalPos.xPos - self.__summitPos.xPos
        Dz = self.__goalPos.zPos - self.__summitPos.zPos

        if Dz >= 0:
            Ang = math.atan(Dx/Dz)
        else:
            if Dx >= 0:
                Ang = math.pi + math.atan(Dx/Dz)
            else:
                Ang = -math.pi + math.atan(Dx/Dz)

        return Ang

    def __calcRotationMessage(self,toTurn):

        rotationMsg = geometry_msgs.msg.Twist()

        #Calculate Speed
        if abs(toTurn) > math.radians(15): # Check if need to turn
            TurnSpeed = 0.5
        else:
            if abs(toTurn) > math.radians(1):
                TurnSpeed = 0.1
            else:
                TurnSpeed = 0
                self.__isAligned = True

        #Calculate Direction
        rotationMsg.angular.z = TurnSpeed*(abs(toTurn)/toTurn)

        return rotationMsg

    def __rotateToPosition(self):
        
        self.__goalAngle = self.__calcAngle()
        
        #Change angle from -180 -> 180 to 0 -> 360 for turtleSim
        self.__goalAngle = (math.pi*2) - (math.pi*2 + self.__goalAngle -math.pi/2)%(math.pi*2)
        
        ToTurn = self.__goalAngle-self.__summitPos.orientation

        self.__cmdNode.publish(self.__calcRotationMessage(ToTurn))

        print '\n\033[1m\033[92mTurning...\033[0m'
        print "\n\033[1mGoal Angle:\033[0m {}".format(math.degrees(self.__goalAngle))
        print "\n\033[1mTo turn:\033[0m {}".format(math.degrees(ToTurn))

    def __moveToPosition(self):
        print '\n\033[1m\033[92mAligned - Moving to Position\033[0m'

        Distance = math.sqrt((self.__goalPos.xPos-self.__summitPos.xPos)**2 + 
                            (self.__goalPos.zPos-self.__summitPos.zPos)**2)

        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0.5
        if  Distance > 0.8: # Check if need to turn
            self.__cmdNode.publish(msg)
        else:
            if Distance > 0.5:
                msg.linear.x *= 0.5
                self.__cmdNode.publish(msg)
            else:
                self.__atGoal = True


        print "\n\033[1mGoal Position:\033[0m{}"
        print "\033[1mX:\033[0m{} \t\n\033[1mZ:\033[0m{}".format(self.__goalPos.xPos,self.__goalPos.zPos)
        print "\n\033[1mCurrent Position:\033[0m{}"
        print "\033[1mX:\033[0m{} \t\n\033[1mZ:\033[0m{}".format(self.__summitPos.xPos,self.__summitPos.zPos)
        
        print "\n\033[1mDistance:\033[0m {}".format(Distance)
        


if __name__ == '__main__':
    try:
        nav = nav2goal(POS_TOPIC,GOAL_TOPIC,CMD_TOPIC)

    except rospy.ROSInterruptException:
        pass