from enum import Enum
from muri_logics.logic_interface import LogicInterface, Out
from muri_logics.general_funcs import quaternion_to_yaw
import math


class TurnStates(Enum):
    INIT = 0
    IDLE = 1
    FAILED = 2
    SUCCESS = 3
    RAEDY = 4
    TURNMOVE = 5



class Constants(): 
    ANGLETOLLERANCE = 0.1
    MAXANGLEVELOSETY = 0.4
    MAXANGLE = 2 * math.pi #TODO anpassen für den Winkl



class TurnOut(Out):
    def __init__(self): 
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
        self.__valeus = None 
        self.__is_Valid = False

    def outValid(self):
        """Check if the output is valid."""
        return self.__is_Valid

    def getError(self): #TODO Exeption?
        """Retrieve any error state."""
        return self.__error
    
    def setError(self, er):
        """Set the error"""
        self.__error = er


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

    
    def reset(self):
        """Reset the logic processing to its initial state."""
        self.__position_X = 0.0
        self.__position_Y = 0.0
        self.__position_Theta = 0.0
        self.__first_Theta = None
        self.__output.resetOut()
        self.__state = TurnStates.INIT

    
    def setOdomData(self, x, y, t,):
        """Sets the Data of the actual Position of the Robot, for the Processing Logic"""
        self.__position_X = x
        self.__position_Y = y
        self.__position_Theta = quaternion_to_yaw(t)

    
    def setCameraData(self, angleTM, distanceIM): 
        """Sets the Data of the actual Position of the Robot, for the Processing Logic"""
        self.__angle_to_Mid_in_Rad = angleTM
        self.__distance_in_meter = distanceIM


    def calculate(self): 
        """Calculate the Angle to Turn and set die Angle velocity"""
        angular_Velocity_Z = 0.0
        turned_Angle = self.__position_Theta - self.__first_Theta

        if abs(self.__angle_to_Mid_in_Rad) > Constants.ANGLETOLLERANCE:
            angular_Velocity_Z = (self.__angle_to_Mid_in_Rad / Constants.MAXANGLE) * Constants.MAXANGLEVELOSETY

        if turned_Angle < math.pi / 2 or self.__distance_in_meter == -1.0:
            angular_Velocity_Z = Constants.MAXANGLEVELOSETY

        return angular_Velocity_Z, turned_Angle


    def state_machine(self):
        """Execute the state machine of the logic processing."""

        match self.__state:

            case TurnStates.INIT:
                self.__output.values = (0.0, 0.0, 0.0, 0.0)
                self.__position_X = 0.0
                self.__position_Y = 0.0
                self.__position_Theta = 0.0
                self.__angle_to_Mid_in_Rad = None
                self.__distance_in_meter = None
                self.__state = TurnStates.IDLE

            case TurnStates.IDLE:
                pass 

            case TurnStates.RAEDY:
                if self.__first_Theta is None:
                    self.__first_Theta = self.__position_Theta
                self.__state = TurnStates.TURNMOVE

            case TurnStates.TURNMOVE:
                avz, ta = self.calculate()
                self.__output.values = (None, None, avz, ta)
                self.__output.__is_Valid = True
                if avz == 0.0:
                    self.__state = TurnStates.SUCCESS

            case TurnStates.FAILED:
                self.__output.setError(True)

            case TurnStates.SUCCESS:
                pass 