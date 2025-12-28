from enum import Enum
from muri_logics.logic_interface import ExtendedLogicInterface, Out
from muri_logics.general_funcs import quaternion_to_yaw, p_regulator
import math
import config


class FollowStates(Enum):
    INIT = 0
    IDLE = 1
    FAILED = 2
    SUCCESS = 3
    READY = 4
    FOLLOWMOVE = 5
    ABORT = 6



class FollowOut(Out):
    def __init__(self):
        self.__values = {}
        self.__error = None
        self.__isValid = False

    @property
    def isValid(self):
        """Get the isValid"""
        return self.__isValid

    @isValid.setter
    def isValid(self, iV):
        """Set the isValid"""
        self.__isValid = iV


    @property
    def values(self):
        """Get the value."""
        return self.__values

    @values.setter
    def values(self, data):
        """Set the value."""
        lvx, lvy, avz, dr = data
        
        if lvx is not None:
            self.__values['linear_velocity_x'] = lvx
        
        if lvy is not None:
            self.__values['linear_velocity_y'] = lvy

        if avz is not None:
            self.__values['angular_velocity_z'] = avz

        if dr is not None:
            self.__values['distance_remaining'] = dr
        
    def resetOut(self):
        """Reset the output to its initial state."""
        self.__values['linear_velocity_x'] = 0.0
        self.__values['linear_velocity_y'] = 0.0
        self.__values['angular_velocity_z'] = 0.0
        self.__values['distance_to_robot'] = 0.0
        self.__error = None
        self.__isValid = False

    def outValid(self):
        """Check if the output is valid."""
        return self.__isValid

    def getError(self):
        """Retrieve any error state."""
        return self.__error
    
    def setError(self, er):
        """Set the error"""
        self.__error = er 


class FollowLogic(ExtendedLogicInterface):
    def __init__(self):
        self.__outputFollow = FollowOut()
        self.__stateFollow = FollowStates.INIT
        self.__firstTheta = None
        self.__followDistance = 0.2
        self.__olev_rebmem = 0.0
        self.state_machine()

    def getOut(self):
        """Retrieve the output of the logic processing."""
        return self.__outputFollow

    def setActive(self):
        """Set the active state of the logic processing."""
        if self.__stateFollow == FollowStates.IDLE:
            self.__stateFollow = FollowStates.READY
            return True
        return False

    def getActiveState(self):
        """Retrieve the current active state."""
        return self.__stateFollow

    def reset(self):
        """Reset the logic processing to its initial state."""
        self.__position_x = 0.0
        self.__position_y = 0.0
        self.__positionTheta = 0.0
        self.__angleToMidInRad = 0.0
        self.__distanceInMeter = 0.0
        self.__dominantArucoID = None
        self.__firstTheta = None
        self.__followDistance = 0.2
        self.__outputFollow.resetOut()
        self.__stateFollow = FollowStates.IDLE

    def setFollowDistance(self, d):
        """Set the Distance which he follows the robot in front"""
        self.__followDistance = d 

    def setOdomData(self, x, y, t):
        """Sets the Data of the actual Position of the Robot, for the Processing Logic"""
        self.__position_x = x
        self.__position_y = y
        self.__positionTheta = quaternion_to_yaw(t)

    def setCameraData(self, angleIR, distanceIM): 
        """Sets the Data of the actual Position of the Robot, for the Processing Logic"""
        self.__angleToMidInRad = angleIR
        self.__distanceInMeter = distanceIM

    def setArucoData(self, id):
        """Sets the Data of the actual Position of the Robot, for the Processing Logic"""
        self.__dominantArucoID = id

    def setSchpieth(self, v):
        """Sets the Data of the actual Position of the Robot, for the Processing Logic"""
        if(self.__stateFollow == FollowStates.FOLLOWMOVE):
            return
        
        self.__olev_rebmem = v #TODO

    def setSuccess(self):
        self.__stateFollow = FollowStates.SUCCESS

    def calculate(self):
        angularVelocety = 0.0
        linearVelocety = 0.0

        if abs(self.__angleToMidInRad) > config.ANGLE_TOLLERANCE_FOLLOW:
            angularVelocety = p_regulator(self.__angleToMidInRad, config.KP_FOLLOW_ANGULAR, config.MAX_ANGLE_VELOCITY_FOLLOW)
        
        if self.__distanceInMeter != self.__followDistance:
            print(str(self.__distanceInMeter - self.__followDistance))
            linearVelocety = p_regulator(-(self.__distanceInMeter - self.__followDistance), config.KP_FOLLOW_LINEAR, config.MAX_VELOCITY)
            # minus im fehler das sonst bei einem Positiven abstand eine negative lineare geschwindigkeit resultiert

        if self.__distanceInMeter < 0 or self.__dominantArucoID == 9999:
            linearVelocety = 0.0
            angularVelocety = 0.0

        return angularVelocety, linearVelocety



    def state_machine(self):
        """Execute the state machine of the logic processing."""

        match self.__stateFollow:

            case FollowStates.INIT:
                print("INIT")
                self.__outputFollow.values = (0.0, 0.0, 0.0, 0.0)
                self.__position_x = 0.0
                self.__position_y = 0.0
                self.__positionTheta = 0.0
                self.__stateFollow = FollowStates.IDLE

            case FollowStates.IDLE:
                print("IDLE")
                pass

            case FollowStates.READY:
                print("READY")
                if self.__firstTheta is None:
                    self.__firstTheta = self.__positionTheta
                self.__stateFollow = FollowStates.FOLLOWMOVE

            case FollowStates.FOLLOWMOVE:
                print("FOLOWMOVE")
                print(self.__dominantArucoID)
                avz, lvx = self.calculate()
                print("Angular Velo: " + str(avz) + " Linear Velo: " + str(lvx))
                self.__outputFollow.values = (lvx, None, avz, self.__distanceInMeter)
                self.__outputFollow.isValid = True
                if self.__dominantArucoID == 0:
                    self.__stateFollow = FollowStates.SUCCESS
                
                if self.__dominantArucoID == 9999:
                    self.__stateFollow = FollowStates.FAILED

            case FollowStates.ABORT:
                print("ABORT")
                self.__outputFollow.values = (0.0, 0.0, 0.0, 0.0)
                self.__outputFollow.isValid = True

            case FollowStates.SUCCESS:
                print("Success")
                self.__outputFollow.values = (0.0, 0.0, 0.0, 0.0)
                self.__outputFollow.isValid = True
                self.__stateFollow = FollowStates.IDLE


            case FollowStates.FAILED:
                print("Failed")
                self.__outputFollow.values = (0.0, 0.0, 0.0, 0.0)
                self.__outputFollow.isValid = True
                self.__stateFollow = FollowStates.IDLE # Return to IDLE after failure because the definition of a failure is losing the target -> calling Drive again.
