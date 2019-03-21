import math

class LaserPoint:

    def __init__(self,Dist,Ang):
        self.__Distance = Dist
        self.__Angle = Ang

    def get_Dist(self):
        return self.__Distance

    def get_Angle(self):
        return self.__Angle

    def Abs_Dist(self):
        return math.cos(self.__Angle)*self.__Distance
