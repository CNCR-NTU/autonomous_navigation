#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.animation
import numpy as np

import rospy
import roslib

import geometry_msgs.msg
import sensor_msgs.msg
from autonomous_navigation.msg import Position

POS_TOPIC = '/summit_xl_controller/position'


class create_map:

    def __init__(self,POS_TOPIC):
        self.__positionTopic = POS_TOPIC

        rospy.init_node('create_map', anonymous=True)

        self.__getPosition()

    
    def __getPosition(self):
        rospy.Subscriber(self.__positionTopic, Position, self.__callback)
        rospy.spin()

    def __callback(self,value):
        print "\nx: {}".format(value.xPos)
        print "z: {}".format(value.zPos)
        print "orientation: {}".format(value.orientation)
        print "-----------------------------------------"


if __name__ == '__main__':
    try:
        cm = create_map(POS_TOPIC)

    except rospy.ROSInterruptException:
                pass

#fig, ax = plt.subplots()
#x, y = [],[]
#sc = ax.scatter(x,y)
#plt.xlim(0,10)
#plt.ylim(0,10)

#def animate(i):
#    x.append(np.random.rand(1)*10)
#    y.append(np.random.rand(1)*10)
#    sc.set_offsets(np.c_[x,y])

#ani = matplotlib.animation.FuncAnimation(fig, animate, 
 #               frames=2, interval=100, repeat=True) 
#plt.show()