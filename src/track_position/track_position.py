#!/usr/bin/env python
import rospy
import roslib

import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

CMD_TOPIC = '/summit_xl_control/cmd_vel'
IS_MOVING_TOPIC = '/summit_xl_controller/in_motion'

class position_tracker:

    def __init__(self,cmd,move):
        self.__cmdTop = cmd
        self.__moveTop = move

        self.__xPos = 0
        self.__zPos = 0
        self.__orientation = 0

        self.__inMotion = False

        self.count = 0
        
        self.__track_pos()
        


    def __IMcallback(self,value):
        self.__inMotion = value

    def __CMDcallback(self,value):
        self.__orientation = (self.__orientation + value.linear.y/0.15 * 180/130)%360
        

        self.count += 1



        if self.__inMotion:
            print "\033[92m\n\033[1mIn Motion\033[0m"
            print "\n\033[1mCurrent Postition:\033[0m"
            print "\n\033[1mX:\033[0m {} \033[1mZ:\033[0m {} \033[1mOrientation\033[0m: {}".format(self.__xPos,self.__zPos,self.__orientation)    
            print "\n\033[1mVelocity:\033[0m"
            print "\n\033[1mLinear:\033[0m x: {} y: {} z: {}".format(value.linear.x,value.linear.y,value.linear.z)
            print "\033[1mAngular:\033[0m x: {} y: {} z: {}".format(value.angular.x,value.angular.y,value.angular.z)
            print "Count {}".format(self.count)
            print('\n---------------------------------------------------------------')


    def __track_pos(self):
        rospy.init_node('position_tracker', anonymous=True)
        rospy.Subscriber(self.__moveTop, std_msgs.msg.Bool, self.__IMcallback)
        rospy.Subscriber(self.__cmdTop,geometry_msgs.msg.Twist,self.__CMDcallback)
        rospy.spin()


if __name__ == '__main__':
    try:
        PT = position_tracker(CMD_TOPIC,IS_MOVING_TOPIC)

    except rospy.ROSInterruptException:
        pass