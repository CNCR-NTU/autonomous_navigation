#!/usr/bin/env python

#==============================
#     Import Libraries 
#==============================
import matplotlib.animation
import matplotlib.pyplot as plt
import numpy as np

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


#==============================
#     Set Topic Variables 
#==============================

POS_TOPIC = '/summit_xl_controller/position'
GOAL_TOPIC = '/goal/position'
CMD_TOPIC = '/summit_xl_control/cmd_vel'

MIN_DISTANCE = 1

#==============================
#     find_path class 
#==============================

#0.6
#9.5


class find_path:
    """Calculates path to navigate from current position to the goal
    """
    
    class Node:
        def __init__(self,position,parentNode=None):
            self.pos = position

            self.g = 0
            self.h = 0
            self.f = 0

            self.parentNode=None

        def getX(self):
            return self.pos.xPos

        def getY(self):
            return self.pos.zPos

        def show(self):
            print "\nX: {} \t Y: {}".format(self.getX(),self.getY())
            print "G: {} \t H: {} \t F: {}".format(self.g,self.h,self.f)
            print "\n--------------------------------------------------"

    def __init__(self,posTop,goalTop,cmdTop):
        #rospy.init_node('find_path')

        self.__PositionX = [4]
        self.__PositionY = [5]

        self.__goalPositionX = [10]
        self.__goalPositionY = [10]

        # Scatter Plot
        self.__fig, self.__scatPlot = plt.subplots()

        # Create plots
        self.__summitPoints = self.__scatPlot.scatter(
            self.__PositionX, self.__PositionY, c='green', edgecolors='none',s=50)

        #self.__laserPoints = self.__scatPlot.scatter(
         #   self.__laserX, self.__laserZ, c='red', edgecolors='none')

        self.__goalPoints = self.__scatPlot.scatter(
            self.__goalPositionX,self.__goalPositionY,c='blue',edgecolors='none',s=50)

        #Plot Annotations
        self.__summitAnnotation = None
        self.__goalAnnotation = None

        plt.xlim(0, 15)
        plt.ylim(0, 15)

    
        #Astar
        startPos = Position(0.0,4.0,5.0)
        endPos = Position(0.0,10.0,10.0)


        Path = self.aStar(startPos,endPos)

        self.xPath, self.yPath = [],[]

        for node in Path:
            xPath.append(node.getX)
            yPath.append(node.getY)

        self.__pathPoints = self.__scatPlot.scatter(
            xPath,yPath,c='black',edgecolors='none',s=50)
        

        self.__plotMap()

    def __plotMap(self):
        """Plots all points from lists onto a scatter graph."""

        if self.__summitAnnotation is not None:
            self.__summitAnnotation.remove()
            self.__goalAnnotation.remove()



        self.__summitPoints.set_offsets(np.c_[self.__PositionX, self.__PositionY])
        self.__summitAnnotation = self.__scatPlot.annotate("Summit",(self.__PositionX[0],self.__PositionY[0]))

        #self.__laserPoints.set_offsets(np.c_[self.__laserX, self.__laserZ])
        self.__pathPoints.set_offsets(np.c_[self.xPath, self.yPath])

        self.__goalPoints.set_offsets(np.c_[self.__goalPositionX, self.__goalPositionY])
        self.__goalAnnotation = self.__scatPlot.annotate("Goal",(self.__goalPositionX[0],self.__goalPositionY[0]))

        self.__fig.canvas.draw_idle()
        plt.show()

        
    def aStar(self,start,end):
        startNode = self.Node(start)
        endNode = self.Node(end)

        startNode.show()
        endNode.show()

        openList,closedList = [],[]

        openList.append(startNode)

        foundPath = False

        while not foundPath:
        #for k in (1,5):
            currentIndex = 0
            currentNode = openList[currentIndex]
        
            print "\n===========NEW LOOP==========",len(openList)

            for index,item in enumerate(openList):
                if item.f < currentNode.f:
                    currentIndex = index
                    currentNode = item

            print "currentNode:"
            currentNode.show()

            # Pop current off open list, add to closed list
            openList.pop(currentIndex)
            closedList.append(currentNode)

            #Found goal
            if self.calcDistance(currentNode,endNode) < MIN_DISTANCE:
                path = []
                current = current_node
                print "returning"
                while current is not None:
                    path.append(current.position)
                    current = current.parentNode

                return path[::-1] # Return reversed path#
            else:
                print "\nNot at goal"

            #Generate Children
            children = []
            print "\nCreate Children"
            for new_position in [(0, -MIN_DISTANCE), (0, MIN_DISTANCE), (-MIN_DISTANCE, 0), (MIN_DISTANCE, 0), (-MIN_DISTANCE, -MIN_DISTANCE), (-MIN_DISTANCE, MIN_DISTANCE), (MIN_DISTANCE, -MIN_DISTANCE), (MIN_DISTANCE, MIN_DISTANCE)]: # Adjacent poinrts
                
                nodePosition = Position(0.0,currentNode.getX()+new_position[0],currentNode.getY()+new_position[1])

                #Check not intersecting object

                #create node
                newNode = self.Node(nodePosition,currentNode)

                #Check in Closed list
                print   "\nCheck in Closed list"

                for closed_child in closedList:
                    if self.calcDistance(newNode,closed_child) <= MIN_DISTANCE/1:
                        print "Exists"
                        continue
                    
                   
                newNode.g = self.calcDistance(newNode,startNode)
                newNode.h = self.calcDistance(newNode,endNode)
                newNode.f = newNode.g + newNode.h

                for open_node in openList:
                    if self.calcDistance(newNode,open_node) <= MIN_DISTANCE and newNode.g > open_node.g:
                        continue

                #children.append(newNode)
                openList.append(newNode)
                newNode.show()


                

            #for child in children:

                # Child is on the closed list
                #for closed_child in closedList:
                 #   if self.calcDistance(child,closed_child) <= MIN_DISTANCE/1:
                  #      continue

                # Create the f, g, and h values
                #child.g = self.calcDistance(child,startNode)
                #child.h = self.calcDistance(child,endNode)
                #child.f = child.g + child.h
                
                # Child is already in the open list
                #for open_node in openList:
                 #   if child == open_node and child.g > open_node.g:
                  #      continue

                #openList.append(child)

    def calcDistance(self,currentNode,endNode):
        Dist = math.sqrt((currentNode.pos.xPos-endNode.pos.xPos)**2 
                + (currentNode.pos.zPos-endNode.pos.zPos)**2)

        #print Dist
        return Dist




if __name__ == '__main__':
    try:
        nav = find_path(POS_TOPIC,GOAL_TOPIC,CMD_TOPIC)

    except rospy.ROSInterruptException:
        pass