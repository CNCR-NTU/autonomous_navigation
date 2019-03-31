#from LaserPoint import LaserPoint
from autonomous_navigation.msg import LaserPoint


class ScanSegment:
    """Structure for storing a LaserScan. Stores min and max angle of scan and all points within this range. 
    Provides functionality for working out information within this range."""

    def __init__(self, min, max, dist=None, ang=None, lRange=[], llinc=None):
        # Start and End angle
        self.__minAng = min
        self.__maxAng = max

        # Maximum distance laser can reach
        self.__laserMax = dist

        # Angle increment between scans
        self.__angleIncrement = ang

        # Angle increment for building the LaserPoint List
        self.__laserListIncrement = llinc

        if self.__laserListIncrement == None:
            self.__laserListIncrement = self.__angleIncrement

        # List of Distance values and ABS Distance values
        self.__LaserRange = self.setLaserRange(lRange)
        self.__AbsRange = []
        # self.__calcAbsRange()

        # Values for Average and Minimum distance over the range
        self.__minDist = None
        self.__avgDist = None

    # ------------------------------------
    #           Getters
    # ------------------------------------

    def getAngInc(self):
        "Returns angle increment between point."

        return self.__angleIncrement

    def getMinAng(self):
        "Return start angle for scan."

        return self.__minAng

    def getMaxAng(self):
        "Return end angle for scan."

        return self.__maxAng

    def getAbsRange(self):
        "Return a list of absolute distance values over the range."

        return self.__AbsRange

    def getAbsAvg(self):
        "Return average absolute distance over range."

        return self.__avgDist

    def getAbsMin(self):
        "Return smallest absolute value over the range."

        return self.__minDist

    def getLaserRange(self):
        "Return a list of of LaserPoints."

        return self.__LaserRange

    def getMaxDist(self):
        "Returns Max distance of Laser"

        return self.__laserMax

    # ------------------------------------
    #               Setters
    # ------------------------------------

    def setAng(self, ang):
        "Sets value of angle increment."

        self.__angleIncrement = ang

    def setLaserRange(self, DistArr):
        "Fills LaserPoint list with values"

        i = 0
        inc = int(self.__laserListIncrement/self.__angleIncrement)

        self.__LaserRange = []

        while i < len(DistArr):
            self.__LaserRange.append(LaserPoint(
                self.__minAng+i*self.__angleIncrement, DistArr[i]))
            i += inc

        # for val in DistArr:
            # self.__LaserRange.append(LaserPoint(self.__minAng+i*self.__angleIncrement,val))
            #i += 1

        # self.__calcAbsRange()
        # self.__calcAbsDistances()

    def setLaserMax(self, max):
        "Sets maximum laser value for refrence to prevent inf values."

        self.__laserMax = max

    # ------------------------------------
    #           Functions
    # ------------------------------------

    def totalScans(self):
        "Returns total number of scans"

        return len(self.__LaserRange)

    def minAngScans(self):
        "Returns number of scans done from origin angle and minimum angle."

        return self.__minAng/self.__angleIncrement

    def maxAngScans(self):
        "Returns number of scans done from origin angle and maximum angle."

        return self.__maxAng/self.__angleIncrement

    def __calcAbsRange(self):
        "Calculates absolute distance values over range."

        self.__AbsRange = []

        for scan in self.__LaserRange:
            self.__AbsRange.append(math.cos(scan.angle)*scan.distance)

    def __calcAbsDistances(self):
        "Calculates average absolute distance value and minimum absolute distance over range."

        total = 0
        count = 0
        self.__minDist = self.__laserMax
        print self.__LaserRange
        for scan in self.__LaserRange:
            scanAbs = math.cos(scan.angle)*scan.distance

            if scanAbs < self.__laserMax:
                total += scanAbs
                count += 1

                if scanAbs < self.__minDist:
                    self.__minDist = scanAbs

        if count != 0:
            self.__avgDist = total/count
