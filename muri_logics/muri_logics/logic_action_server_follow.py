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
        self.__values = (0.0, 0.0, 0.0, 0.0)
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
        self.__positionX = 0.0
        self.__positionY = 0.0
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
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__positionX = x
        self.__positionY = x
        self.__positionTheta = quaternion_to_yaw(t)

    def setCameraData(self, angleIR, distanceIM): 
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__angleToMidInRad = angleIR
        self.__distanceInMeter = distanceIM

    def setArucoData(self, id):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__dominantArucoID = id

    def calculate(self):
        angularVelocety = 0.0
        linearVelocety = 0.0

        if abs(self.__angleToMidInRad) > config.ANGLE_TOLLERANCE_FOLLOW:
            angularVelocety = p_regulator(self.__angleToMidInRad, config.KP_FOLLOW_ANGULAR, config.MAX_ANGLE_VELOCITY_FOLLOW)
        
        if self.__distanceInMeter < self.__followDistance:
            linearVelocety = p_regulator(self.__distanceInMeter, config.KP_FOLLOW_LINEAR, config.MAX_VELOCITY)

        if self.__distanceInMeter == -1: #TODO Evtl.
            linearVelocety = 0.0

        return angularVelocety, linearVelocety



    def state_machine(self):
        """Execute the state machine of the logic processing."""

        match self.__stateFollow:

            case FollowStates.INIT:
                self.__outputFollow = (0.0, 0.0, 0.0, 0.0)
                self.__positionX = 0.0
                self.__positionY = 0.0
                self.__positionTheta =0.0
                self.__stateFollow = FollowStates.IDLE

            case FollowStates.IDLE:
                pass

            case FollowStates.READY:
                if self.__firstTheta is None:
                    self.__firstTheta = self.__positionTheta
                self.__stateFollow = FollowStates.FOLLOWMOVE

            case FollowStates.FOLLOWMOVE:
                avz, lvx, dim = self.calculate()
                self.__outputFollow.values = (lvx, None, avz, dim)
                if self.__dominantArucoID == 0:
                    self.__stateFollow = FollowStates.SUCCESS
                
                if self.__dominantArucoID == 9999:
                    self.__stateFollow = FollowStates.FAILED

            case FollowStates.ABORT: #TODO Nützlich? 
                self.__outputFollow.values = config.STOP

            case FollowStates.SUCCESS:
                self.__outputFollow.setError(False)

            case FollowStates.FAILED:
                self.__outputFollow.values = config.STOP
                self.__outputFollow.setError(True)