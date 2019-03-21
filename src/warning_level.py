

class WarnLVL:

    def __init__(self,CC,V,war):
        self.__ConsoleColour = CC
        self.__Velocity = V
        self.__Warning = war

    def get_Warning(self):
        return self.__ConsoleColour + self.__Warning + '\033[0m'

    def get_velocity(self):
        return self.__Velocity 
    