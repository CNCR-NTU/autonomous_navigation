#!/usr/bin/env python

#==============================
#     Import Libraries
#==============================

import rospy
import roslib

import socket
import struct

import math

#==============================
#     Import Messages
#==============================

import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from autonomous_navigation.msg import Position


#==============================
#       Set IP Address 
#==============================

UDP_IP = "192.168.2.108"
UDP_PORT = 51001

#==============================
#     Set Topic Variables 
#==============================

POS_TOPIC = '/summit_xl_controller/position'
GOAL_TOPIC = '/goal/position'

#==============================
#     vicon_tracker class 
#==============================

class vicon_tracker:
    """Listens to IP for object position information sent by the VICON SDK"""

    def __init__(self,ip,port,positionTopic,goalTopic):
        """Creates instance of vicon_tracker class.
        ip = IP Address of SDK
        port = Port of SDK
        positionTopic = Topic to publish position of Robot to
        goalTopic = Topic to publish position of Goal to
        """

        #Init Node
        rospy.init_node('vicon_tracker', anonymous=True)

        #Create UDP Topics
        self.__sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        
        self.__sock.bind(('',port))

        #Create Publisher Nodes
        self.SummitPositionNode = rospy.Publisher(positionTopic, Position, queue_size=1)
        self.GoalPositionNode = rospy.Publisher(goalTopic, Position, queue_size=1)

        print "\n\n\033[1mListening to port: {} on {} for VICON tracker information...\033[0m".format(port,ip)

        self.__get_data()


    def __get_data(self):
        """Loops until closed. Reads information packets from IP address, turns information into
        ROS Messages to be published"""
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            data, addr = self.__sock.recvfrom(1024) # buffer size is 1024 bytes
            frame = struct.unpack_from('I' , data, 0)

            #Get Object Names from Data
            object1name = data[8:14]
            object2name = data[83:87]

            if object1name == "Summit":
                #Unpack Variables
                xPos = (struct.unpack_from('d',data,32))[0]
                zPos = (struct.unpack_from('d',data,40))[0]
                Orientation = (struct.unpack_from('d',data,72))[0]

                #Send Position Message
                self.SummitPositionNode.publish(Position(Orientation,xPos,zPos))

            if object2name =="Goal":
                #Unpack Variables
                xPos = (struct.unpack_from('d',data,107))[0]
                zPos = (struct.unpack_from('d',data,115))[0]

                #Send Position Message
                self.GoalPositionNode.publish(Position(0.0,xPos,zPos))

            rate.sleep()

if __name__ == '__main__':
    try:
        vt = vicon_tracker(UDP_IP, UDP_PORT,POS_TOPIC,GOAL_TOPIC)

    except rospy.ROSInterruptException:
        pass