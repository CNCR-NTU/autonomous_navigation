#!/usr/bin/env python

import geometry_msgs.msg
import math
# ==============================
#     Import Libraries
# ==============================
import rospy
from autonomous_navigation.msg import Position
# ==============================
#     Import Messages
# ==============================
from std_msgs.msg import Float32

# ==============================
#     Set Topic Variables 
# ==============================

POS_TOPIC = '/summit_xl/position'
GOAL_TOPIC = '/goal/position'
CMD_TOPIC = '/summit_xl/cmd_vel'
SPEED_TOPIC = 'summit_xl/Speed'

MIN_DISTANCE = 0.6


# ==============================
#     nav2goal class 
# ==============================


class nav2goal:
    """Calculates path to navigate from current position to the goal
    """

    def __init__(self, posTop, goalTop, cmdTop, speedTop):
        rospy.init_node('nav2goal')

        self._positionTopic = posTop
        self._goalTopic = goalTop
        self._speedTopic = speedTop

        self._cmdNode = rospy.Publisher(cmdTop, geometry_msgs.msg.Twist, queue_size=1)

        self._summitPos = Position()
        self._goalPos = Position()

        self._atGoal = False
        self._isAligned = False
        self._speed = 0

        self._angleMin = 0
        self._angleMax = 2 * math.pi

        self._goalAngle = 0.0

        self.get_data()

    def get_data(self):
        """ Gets speed and position data from topics"""

        rospy.Subscriber(self._speedTopic, Float32, self._speedCallback, queue_size=1)
        rospy.Subscriber(self._positionTopic, Position, self.posCallback, queue_size=1)
        rospy.Subscriber(self._goalTopic, Position, self.goalCallback, queue_size=1)
        rospy.spin()

    def posCallback(self, data):
        """Sets Summit position from topic data"""

        self._summitPos = data

    def _calcGoalDifference(self, data):
        """Calculates distance to goal"""

        return math.sqrt((self._goalPos.xPos - data.xPos) ** 2 + (self._goalPos.zPos - data.zPos) ** 2)

    def goalCallback(self, data):
        """Sets goal position from topic data"""

        # Check Goal has not changed
        if self._calcGoalDifference(data) > 75:
            self._goalPos = data
            self._isAligned = False
            self._atGoal = False

        else:
            if not self._atGoal:
                if not self._isAligned:
                    self._rotateToPosition()
                else:
                    self._moveToPosition()
            else:
                print('\n\033[4m\033[1mGoal Reached\033[0m\n')

    def _speedCallback(self, speed):
        """Sets speed from topic data"""

        self._speed = float(speed.data)

    def _calcAngle(self):
        """Calculates the angle need to turn to align with the goal"""

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
        """Creates the ROS message to be published based on amount needed to turn"""

        rotationMsg = geometry_msgs.msg.Twist()

        # Calculate Speed
        if abs(toTurn) > math.radians(15):  # Check if need to turn
            TurnSpeed = self._speed
        else:
            if abs(toTurn) > math.radians(5):
                TurnSpeed = self._speed / 2
            else:
                TurnSpeed = 0
                self._isAligned = True

        # Calculate Direction
        rotationMsg.angular.z = TurnSpeed * (abs(toTurn) / toTurn)

        return rotationMsg

    def _rotateToPosition(self):
        """Rotates the robot to align with the goal"""

        self._goalAngle = self._calcAngle()

        ToTurn = self._goalAngle - self._summitPos.orientation

        self._cmdNode.publish(self._calcRotationMessage(ToTurn))

        print('\n\033[1m\033[92mTurning...\033[0m')
        print("\n\033[1mCurrent Angle:\033[0m {}".format(math.degrees(self._summitPos.orientation)))
        print("\n\033[1mGoal Angle:\033[0m {}".format(math.degrees(self._goalAngle)))
        print("\n\033[1mTo turn:\033[0m {}".format(math.degrees(ToTurn)))
        print("\n\033[1mCurrent Speed:\033[0m {}".format(self._speed))

    def _moveToPosition(self):
        """Moves the robot until the goal has been reached"""

        print('\n\033[1m\033[92mAligned - Moving to Position\033[0m')

        Distance = math.sqrt((self._goalPos.xPos - self._summitPos.xPos) ** 2 +
                             (self._goalPos.zPos - self._summitPos.zPos) ** 2)

        msg = geometry_msgs.msg.Twist()
        msg.linear.x = self._speed

        if Distance > 1500:  # Check if need to turn
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
        nav = nav2goal(POS_TOPIC, GOAL_TOPIC, CMD_TOPIC, SPEED_TOPIC)

    except rospy.ROSInterruptException:
        pass
