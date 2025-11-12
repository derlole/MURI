from enum import Enum
from muri_logics.logic_interface import LogicInterface, Out
from muri_logics.general_funcs import quaternion_to_yaw
import math

class DriveStates(Enum):
    INIT = 0
    IDLE = 1
    FAILED = 2
    SUCCESS = 3
    RAEDY = 4
    DRIVEMOVE = 5



class Constants():
    ANGLETOLLERANCE = 0.1
    MAXVELOSETY = 0.4
    MAXANGLEVELOSETY = 0.1
    MAXANGLE = 2 * math.pi #TODO maximalen winkel anpassen
    GOALDISTANCE = 0.25 #TODO Sinvolle distanz



class DriveOut(Out):
    def __init__(self):
        self.__values = {}
        self.__error = None
        self.__is_Valid = False


    @property
    def values(self):
        """Get the value."""
        return self.values


    @values.setter
    def values(self, data):
        """Set the value."""
        lvx, lvy, avz, dr = data
        
        if lvx is not None:
            self.__values['linear_velocity_x'] = lvx
        
        if lvy is not None:
            self.__values['linear_velocity_y'] = lvy

        if avz is not None:
            self.__values['angular_velocity_Z'] = avz

        if dr is not None:
            self.__values['distance_remaining'] = dr


    def resetOut(self):
        """Reset the output to its initial state."""
        self.__values = {}
        self.__error = None
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



class DriveLogic(LogicInterface):
    def __init__(self):
        self.__output = DriveOut()
        self.__state = DriveStates.INIT
        self.__first_Theta = None
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
        self.__state = DriveStates.IDLE


    def setOdomData(self, x, y, t):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__position_X = x
        self.__position_Y = y
        self.__position_Theta = quaternion_to_yaw(t)


    def setCameraData(self, angleIR, distanceIM): 
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__angle_to_Mid_in_Rad = angleIR
        self.__distance_in_Meter = distanceIM



    def calculate(self):
        """Calculate the Angle to Turn and set die Angelvelosity"""
        angular_Velocity = 0.0
        linear_Velocity = 0.0

        if abs(self.__angle_to_Mid_in_Rad) > Constants.ANGLETOLLERANCE and self.__distance_in_Meter > Constants.GOALDISTANCE:
            angular_Velocity = (self.__angle_to_Mid_in_Rad / Constants.MAXANGLE) * Constants.MAXANGLEVELOSETY

        if self.__distance_in_Meter > Constants.GOALDISTANCE: 
            linear_Velocity = Constants.MAXVELOSETY

        else:
            linear_Velocity = 0.0

        return angular_Velocity, linear_Velocity
    

    def state_machine(self):
            """Execute the state machine of the logic processing."""
            
            match self.__state:

                case DriveStates.INIT:
                    self.__output.values = (0.0, 0.0, 0.0, 0.0)
                    self.__position_X = 0.0
                    self.__position_Y = 0.0
                    self.__position_Theta = 0.0

                case DriveStates.IDLE:
                    pass

                case DriveStates.RAEDY:
                    if self.__first_Theta is None:
                        self.__first_Theta = self.__position_Theta
                    self.__state = DriveStates.DRIVEMOVE

                case DriveStates.DRIVEMOVE:
                    avz, lv = self.calculate()
                    self.__output.values = (lv, None, avz, self.__distance_in_Meter)
                    self.__output.__is_Valid = True
                    if self.__distance_in_Meter < Constants.GOALDISTANCE:
                        self.__state = DriveStates.SUCCESS

                case DriveStates.FAILED:
                    self.__output.setError(True)

                case DriveStates.SUCCESS:
                    pass 