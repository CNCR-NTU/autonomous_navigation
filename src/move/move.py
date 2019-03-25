#!/usr/bin/env python
import rospy
import roslib

import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

#75 = 180

def turn_right(degrees):
    rospy.init_node('joshmv', anonymous=True)
    velocityNode = rospy.Publisher("/summit_xl_control/cmd_vel", geometry_msgs.msg.Twist,queue_size=10)
    print "hello"
    cmd = geometry_msgs.msg.Twist()
    rate = rospy.Rate(30)

    cmd.linear.y = -0.15

    totalcount = degrees*130/180

    count = 0

    while count < totalcount:
        velocityNode.publish(cmd)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        turn_right(360)

    except rospy.ROSInterruptException:
        pass
    
