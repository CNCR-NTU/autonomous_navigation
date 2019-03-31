

class WarnLVL:
    """Structure for warnings, contains the warning, the colour to print to
     console and the speed of the system associated with this warning level."""

    def __init__(self, CC, V, war):
        self.__ConsoleColour = CC
        self.__Velocity = V
        self.__Warning = war

    def get_Warning(self):
        "Returns warning as a String to be printed."
        return self.__ConsoleColour + self.__Warning + '\033[0m'

    def get_velocity(self):
        "Returns velocity."
        return self.__Velocity
