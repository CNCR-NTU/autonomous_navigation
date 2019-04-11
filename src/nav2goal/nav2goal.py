#!/usr/bin/env python

# ==============================
#     Import Libraries 
# ==============================
import roslib
import rospy
import math
from os import system
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
GOAL_TOPIC = '/goal/position'
CMD_TOPIC = '/summit_xl_control/cmd_vel'

MIN_DISTANCE = 0.6


# ==============================
#     nav2goal class 
# ==============================

# 0.6
# 9.5

class nav2goal:
    """Calculates path to navigate from current position to the goal
    """

    def __init__(self, posTop, goalTop, cmdTop):
        rospy.init_node('nav2goal')

        self._positionTopic = posTop
        self._goalTopic = goalTop

        self._cmdNode = rospy.Publisher(cmdTop, geometry_msgs.msg.Twist, queue_size=1)

        self._summitPos = Position()
        self._goalPos = Position()

        self._atGoal = False
        self._isAligned = False

        self._angleMin = 0
        self._angleMax = 2 * math.pi

        self._goalAngle = 0.0

        self.get_data()

    def get_data(self):
        rospy.Subscriber(self._positionTopic, Position, self.posCallback, queue_size=1)
        rospy.Subscriber(self._goalTopic, Position, self.goalCallback, queue_size=1)
        rospy.spin()

    def posCallback(self, data):
        self._summitPos = Position(data.orientation, data.xPos, data.zPos)

    def _calcGoalDifference(self, data):
        Difference = math.sqrt((self._goalPos.xPos - data.xPos) ** 2 +
                               (self._goalPos.zPos - data.zPos) ** 2)
        return Difference

    def goalCallback(self, data):
        # Check Goal has not changed
        if self._calcGoalDifference(data) > 75:
            self._goalPos = data
            self._isAligned = False
            self._atGoal = False

        else:
            if not self._atGoal:
                if not self._isAligned:
                    self._rotateToPosition()
                # else:
                # self._moveToPosition()
            else:
                print('\n\033[4m\033[1mGoal Reached\033[0m\n')

    def _calcAngle(self):
        Dx = (self._goalPos.xPos - self._summitPos.xPos)
        Dz = (self._goalPos.zPos - self._summitPos.zPos)

        if Dx >= 0:
            Ang = math.atan(Dz / Dx)
        else:
            if Dz >= 0:
                Ang = math.pi + math.atan(Dz / Dx)
            else:
                Ang = -math.pi + math.atan(Dz / Dx)

        Ang += math.pi / 2

        if Ang > math.pi:
            Ang = -2 * math.pi + Ang

        return Ang

    def _calcRotationMessage(self, toTurn):  # Clockwise is negative angle

        rotationMsg = geometry_msgs.msg.Twist()

        # Calculate Speed
        if abs(toTurn) > math.radians(15):  # Check if need to turn
            TurnSpeed = 0.2
        else:
            if abs(toTurn) > math.radians(1):
                TurnSpeed = 0.1
            else:
                TurnSpeed = 0
                self._isAligned = True

        # Calculate Direction
        rotationMsg.linear.y = TurnSpeed * (abs(toTurn) / toTurn)

        return rotationMsg

    def _rotateToPosition(self):

        self._goalAngle = self._calcAngle()

        ToTurn = self._goalAngle - self._summitPos.orientation

        self._cmdNode.publish(self._calcRotationMessage(ToTurn))

        print('\n\033[1m\033[92mTurning...\033[0m')
        print("\n\033[1mGoal Angle:\033[0m {}".format(math.degrees(self._goalAngle)))
        print("\n\033[1mTo turn:\033[0m {}".format(math.degrees(ToTurn)))

    def _moveToPosition(self):
        print('\n\033[1m\033[92mAligned - Moving to Position\033[0m')

        Distance = math.sqrt((self._goalPos.xPos - self._summitPos.xPos) ** 2 +
                             (self._goalPos.zPos - self._summitPos.zPos) ** 2)

        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0.3

        if Distance > 1000:  # Check if need to turn
            self._cmdNode.publish(msg)
        else:
            if Distance > MIN_DISTANCE * 1000:
                msg.linear.x *= 0.5
                self._cmdNode.publish(msg)
            else:
                self._atGoal = True

        print("\n\033[1mGoal Position:\033[0m{}")
        print("\033[1mX:\033[0m{} \t\n\033[1mZ:\033[0m{}".format(self._goalPos.xPos, self._goalPos.zPos))
        print("\n\033[1mCurrent Position:\033[0m{}")
        print("\033[1mX:\033[0m{} \t\n\033[1mZ:\033[0m{}".format(self._summitPos.xPos, self._summitPos.zPos))
        print("\n\033[1mDistance:\033[0m {}".format(Distance))


if __name__ == '__main__':
    try:
        nav = nav2goal(POS_TOPIC, GOAL_TOPIC, CMD_TOPIC)

    except rospy.ROSInterruptException:
        pass
