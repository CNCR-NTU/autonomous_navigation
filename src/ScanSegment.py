from LaserPoint import LaserPoint

class ScanSegment:
    """Structure for storing a LaserScan. Stores min and max angle of scan and all points within this range. 
    Provides functionality for working out information within this range."""


    def __init__(self,min,max,dist=None,ang = None,lRange=[]):
        #Start and End angle
        self.__minAng = min
        self.__maxAng = max

        #Maximum distance laser can reach
        self.__laserMax = dist

        #Angle increment between scans
        self.__angleIncrement = ang

        #List of Distance values and ABS Distance values
        self.__LaserRange = lRange
        self.__AbsRange = []
        self.__calcAbsRange()

        #Values for Average and Minimum distance over the range
        self.__minDist = None
        self.__avgDist = None

    #------------------------------------
    #           Getters
    #------------------------------------

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
        "Rreturn smallest absolute value over the range."

        return self.__minDist

    #------------------------------------
    #               Setters
    #------------------------------------

    def setAng(self,ang):
        "Sets value of angle increment."

        self.__angleIncrement = ang

    def setLaserRange(self,DistArr):
        "Fills LaserPoint list with values"

        i = 0
        self.__LaserRange = []

        for val in DistArr:
            self.__LaserRange.append(LaserPoint(val,self.__minAng+i*self.__angleIncrement))
            i += 1

        self.__calcAbsRange()
        self.__calcAbsDistances()

    def setLaserMax(self,max):
        "Sets maximum laser value for refrence to prevent inf values."
        
        self.__laserMax = max

        
    #------------------------------------
    #           Functions
    #------------------------------------
    
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
            self.__AbsRange.append(scan.Abs_Dist())
    
    def __calcAbsDistances(self):
        "Calculates average absolute distance value and minimum absolute distance over range."

        total = 0
        count = 0
        self.__minDist = self.__laserMax

        for scan in self.__LaserRange:
            if scan.Abs_Dist() < self.__laserMax:
                total += scan.Abs_Dist()
                count += 1

                if scan.Abs_Dist() < self.__minDist:
                    self.__minDist = scan.Abs_Dist()

        if count != 0:
            self.__avgDist = total/count

