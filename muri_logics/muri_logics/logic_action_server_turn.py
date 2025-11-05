from enum import Enum
from  muri_logics.logic_interface import LogicInterface, Out
from general_funcs import quaternion_to_yaw


class TurnStates(Enum):
    INIT = 0
    IDLE = 1
    FAILED = 2
    SUCCESS = 3
    RAEDY = 4
    TURNMOVE = 5



class Constants(): #TODO für eine sinfolle Regelung
    MAXANGLEVELOSETY = 0.4



class TurnOut(Out):
    def __init__(self): #TODO überarbeiten?!
        self.__valeus = None
        self.__error = None
        self.__is_Valid = False

    @property
    def values(self):
        """Get the value."""
        return self.__valeus

    @values.setter 
    def values(self, lvx, lvy, avz, ta):
        """Set the value."""
        if lvx is not None:
            self.__valeus['linear_velocity_x'] = lvx
        
        if lvy is not None:
            self.__valeus['linear_velocity_y'] = lvy

        if avz is not None:
            self.__valeus['angular_velocity_Z'] = avz

        if ta is not None:
            self.__values['turened_angele'] = ta

    def resetOut(self):
        """Reset the output to its initial state."""
        self.__valeus = None #TODO Rückseteźen None oder 0 ?
        self.__is_Valid = False

    def outValid(self):
        """Check if the output is valid."""
        return self.__is_Valid

    def getError(self): #TODO Exeption?
        """Retrieve any error state."""
        return self.__error
    


class TurnLogic(LogicInterface):
    def __init__(self):
        self.__output = TurnOut()
        self.__state = TurnStates.INIT
        self.__first_Theta = None
        self.state_machine()

    
    def getOut(self):
        """Retrieve the output of the logic processing."""
        return self.__output

    
    def setActive(self):
        """Set the active state of the logic processing."""
        if self.__state == TurnStates.IDLE:
            self.__state = TurnStates.RAEDY
            return True
        return False

    
    def getActiveState(self):
        """Retrieve the current active state."""
        return self.__state

    
    def reset(slef):
        """Reset the logic processing to its initial state."""
        pass #TODO reset logic processing

    
    def setOdomData(self, x, y, t,):
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


    def calculate(slef):
        pass #TODO Calculate


    def state_machine(self):
        """Execute the state machine of the logic processing."""

        match self.__state:

            case TurnStates.INIT:
                self.__output.values = (0.0, 0.0, 0.0, 0.0)
                self.__position_X = 0.0
                self.__position_Y = 0.0
                self.__position_Theta = 0.0
                self.__camera_Data = None #TODO setze ich die camaradaten so richtig zurück?
                self.__state = TurnStates.IDLE

            case TurnStates.IDLE:
                pass 

            case TurnStates.RAEDY:
                if self.__first_Theta is None:
                    self.__first_Theta = self.__position_Theta
                self.__state = TurnStates.TURNMOVE

            case TurnStates.TURNMOVE:
                avz, ta = self.calculate()

            case TurnStates.FAILED:
                pass #TODO FAILED

            case TurnStates.SUCCESS:
                pass #TODO SUCCESS