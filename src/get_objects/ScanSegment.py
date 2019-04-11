# from LaserPoint import LaserPoint
from autonomous_navigation.msg import LaserPoint


class ScanSegment:
    """Structure for storing a LaserScan. Stores min and max angle of scan and all points within this range. 
    Provides functionality for working out information within this range."""

    def __init__(self, min, max, dist=None, ang=None, lRange=[], llinc=None):
        # Start and End angle
        self._minAng = min
        self._maxAng = max

        # Maximum distance laser can reach
        self._laserMax = dist

        # Angle increment between scans
        self._angleIncrement = ang

        # Angle increment for building the LaserPoint List
        self._laserListIncrement = llinc

        if self._laserListIncrement == None:
            self._laserListIncrement = self._angleIncrement

        # List of Distance values and ABS Distance values
        self._LaserRange = self.setLaserRange(lRange)
        self._AbsRange = []
        # self._calcAbsRange()

        # Values for Average and Minimum distance over the range
        self._minDist = None
        self._avgDist = None

    # ------------------------------------
    #           Getters
    # ------------------------------------

    def getAngInc(self):
        "Returns angle increment between point."

        return self._angleIncrement

    def getMinAng(self):
        "Return start angle for scan."

        return self._minAng

    def getMaxAng(self):
        "Return end angle for scan."

        return self._maxAng

    def getAbsRange(self):
        "Return a list of absolute distance values over the range."

        return self._AbsRange

    def getAbsAvg(self):
        "Return average absolute distance over range."

        return self._avgDist

    def getAbsMin(self):
        "Return smallest absolute value over the range."

        return self._minDist

    def getLaserRange(self):
        "Return a list of of LaserPoints."

        return self._LaserRange

    def getMaxDist(self):
        "Returns Max distance of Laser"

        return self._laserMax

    # ------------------------------------
    #               Setters
    # ------------------------------------

    def setAng(self, ang):
        "Sets value of angle increment."

        self._angleIncrement = ang

    def setLaserRange(self, DistArr):
        "Fills LaserPoint list with values"

        i = 0
        inc = int(self._laserListIncrement / self._angleIncrement)

        self._LaserRange = []

        while i < len(DistArr):
            self._LaserRange.append(LaserPoint(
                self._minAng + i * self._angleIncrement, DistArr[i]))
            i += inc

        # for val in DistArr:
        # self._LaserRange.append(LaserPoint(self._minAng+i*self._angleIncrement,val))
        # i += 1

        # self._calcAbsRange()
        # self._calcAbsDistances()

    def setLaserMax(self, max):
        "Sets maximum laser value for refrence to prevent inf values."

        self._laserMax = max

    # ------------------------------------
    #           Functions
    # ------------------------------------

    def totalScans(self):
        "Returns total number of scans"

        return len(self._LaserRange)

    def minAngScans(self):
        "Returns number of scans done from origin angle and minimum angle."

        return self._minAng / self._angleIncrement

    def maxAngScans(self):
        "Returns number of scans done from origin angle and maximum angle."

        return self._maxAng / self._angleIncrement

    def _calcAbsRange(self):
        "Calculates absolute distance values over range."

        self._AbsRange = []

        for scan in self._LaserRange:
            self._AbsRange.append(math.cos(scan.angle) * scan.distance)

    def _calcAbsDistances(self):
        "Calculates average absolute distance value and minimum absolute distance over range."

        total = 0
        count = 0
        self._minDist = self._laserMax
        print
        self._LaserRange
        for scan in self._LaserRange:
            scanAbs = math.cos(scan.angle) * scan.distance

            if scanAbs < self._laserMax:
                total += scanAbs
                count += 1

                if scanAbs < self._minDist:
                    self._minDist = scanAbs

        if count != 0:
            self._avgDist = total / count
