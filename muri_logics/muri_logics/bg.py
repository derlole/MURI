from enum import Enum
from muri_logics.logic_interface import LogicInterface, Out
from muri_logics.general_funcs import quaternion_to_yaw, p_regulator
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
    MAXANGLE = math.pi 



class TurnOut(Out):
    def __init__(self): 
        self.__values = {}
        self.__error = None
        self.__isValid = False


    @property
    def isValid(self):
        """Get the isValid"""
        return self.__isValid

    @isValid.setter
    def isValid(self, hiV):
        """Set the isValid"""
        self.__isValid = hiV


    @property
    def values(self):
        """Get the value."""
        return self.__values

    @values.setter 
    def values(self, data):
        """Set the value."""
        lvx, lvy, avz, ta = data

        if lvx is not None:
            self.__values['linear_velocity_x'] = lvx
        
        if lvy is not None:
            self.__values['linear_velocity_y'] = lvy

        if avz is not None:
            self.__values['angular_velocity_z'] = avz

        if ta is not None:
            self.__values['turened_angle'] = ta

    def resetOut(self):
        """Reset the output to its initial state."""
        self.__values['linear_velocity_x'] = 0.0
        self.__values['linear_velocity_y'] = 0.0
        self.__values['angular_velocity_z'] = 0.0
        self.__values['turned_angle'] = 0.0
        self.__isValid = False

    def outValid(self):
        """Check if the output is valid."""
        return self.__isValid

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
        self.__state = TurnStates.IDLE

    
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
        """Calculate commands for angular velocity based on the current orientation and also reurns the turnd angle.
        The function rotates the robot toward the target if the angular deviation exceeds a tolerance.
        A proportional controller is used to determine the angular velocity."""
        angular_Velocity_Z = 0.0
        turned_Angle = self.__position_Theta - self.__first_Theta

        if abs(self.__angle_to_Mid_in_Rad) > Constants.ANGLETOLLERANCE:
            angular_Velocity_Z = p_regulator(self.__angle_to_Mid_in_Rad, 0.2, Constants.MAXANGLEVELOSETY)

        if turned_Angle < 2 * math.pi / 3 or self.__distance_in_meter == -1.0:
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
                self.__output.isValid = True
                if abs(self.__angle_to_Mid_in_Rad) < Constants.ANGLETOLLERANCE and self.__distance_in_meter > 1.0:
                    self.__state = TurnStates.SUCCESS

            case TurnStates.FAILED:
                self.__output.setError(True)

            case TurnStates.SUCCESS:
                pass 