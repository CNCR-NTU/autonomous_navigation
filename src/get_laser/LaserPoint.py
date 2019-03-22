import math

class LaserPoint:
    """A structure for storing an instance of a LaserScan point. Stores the 
    distance of the scan and the angle at which the scan was taken."""


    def __init__(self,Dist,Ang):
        self.__Distance = Dist
        self.__Angle = Ang

    def get_Dist(self):
        "Returns Distance of scan."
        return self.__Distance

    def get_Angle(self):
        "Returns Angle of scan."
        return self.__Angle

    def Abs_Dist(self):
        "Returns distance of scan relative to angle scan was taken."
        return math.cos(self.__Angle)*self.__Distance

    def printPoint(self):
        "Prints information on a given point."

        print "Angle: {} \t Distance: {}".format(self.__Angle,self.__Distance)
