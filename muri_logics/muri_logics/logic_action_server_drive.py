from enum import Enum
from muri_logics.logic_interface import LogicInterface, Out
from muri_logics.general_funcs import quaternion_to_yaw

class DriveStates(Enum):
    INIT = 0
    IDLE = 1
    FAILED = 2
    SUCCESS = 3
    RAEDY = 4
    DRIVEMOVE = 5



class Constants():
    MAXVELOSETY = 0.4



class DriveOut(Out):
    def __init__(self):
        self.__valeus = None
        self.__error = None
        self.__is_Valid = False


    @property
    def values(self):
        """Get the value."""
        return self.values


    @values.setter
    def values(self, val):
        """Set the value."""
        self.__valeus = val


    def resetOut(self):
        """Reset the output to its initial state."""
        self.__valeus = None
        self.__error = None
        self.__is_Valid = False


    def outValid(self) -> bool:
        """Check if the output is valid."""
        return self.__is_Valid


    def getError(self):
        """Retrieve any error state."""
        return self.__error



class DriveLogic(LogicInterface):
    def __init__(self):
        self.__output = DriveOut()
        self.__state = DriveStates.INIT
        self.state_machine()


    def getOut(self):
        """Retrieve the output of the logic processing."""
        return self.__output


    def setActive(self):
        """Set the active state of the logic processing."""
        if self.__state == DriveStates.IDLE:
            self.__state = DriveStates.RAEDY
            return True
        return False


    def getActiveState(self):
        """Retrieve the current active state."""
        return self.__state


    def reset(self):
        """Reset the logic processing to its initial state."""
        self.__position_X = 0.0
        self.__position_Y = 0.0
        self.__position_Theta = 0.0
        self.__first_Theta = None
        self.__output.resetOut()
        self.__state = DriveStates.INIT


    def setOdomData(self, x, y, t):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__position_X = x
        self.__position_Y = y
        self.__position_Theta = quaternion_to_yaw(t)


    def setCameraData(self, pToMid, pToMidPrev, pHeight, pHeightPrev, picWidth): 
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__pixel_To_Mid = pToMid
        self.__pixel_To_Mid_Prev = pToMidPrev
        self.__pixel_Heigth = pHeight
        self.__pixel_Heigth_Prev = pHeightPrev
        self.__picture_Widht = picWidth


    def clculate(self): #TODO
        """Calculate the Angle to Turn and set die Angelvelosity"""
        pass


    def state_machine(self):
            """Execute the state machine of the logic processing."""
            
            match self.__state:

                case DriveStates.INIT:
                    pass #TODO 

                case DriveStates.RAEDY:
                    pass #TODO 

                case DriveStates.DRIVEMOVE:
                    pass #TODO 

                case DriveStates.FAILED:
                    pass #TODO 

                case DriveStates.SUCCESS:
                    pass #TODO 