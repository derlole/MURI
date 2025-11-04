import math

from enum import Enum
from muri_logics.logic_interface import LogicInterface, Out
from muri_logics.general_funcs import quaternion_to_yaw

class MainStates(Enum):
    INIT = 0
    IDLE = 1

    INIT_ROBOT = 4
    DRIVE = 5
    TURN = 6
    PAUSE = 7

    FAILED = 2
    SUCCESS = 3

class MainOut(Out):
    def __init__(self):
        self.__values
        self.__error
        self.__isValid = False

    @property
    def values(self):
        """Get the value."""
        return self.__values

    @values.setter
    def values(self, val):
        """Set the value."""
        self.__values = val

    def resetOut(self):
        """Reset the output to its initial state."""
        self.__values = None
        self.__error = None 
        self.__isValid = False

    def outValid(self):
        """Check if the output is valid."""
        return self.__isValid

    def getError(self):
        """Retrieve any error state."""
        return self.__error

class MainController(LogicInterface):
    def __init__(self):
        self.__state = MainStates.INIT
        self.__output = MainOut()
        self._o_l_x = 0.0
        self._o_l_z = 0.0
        self._o_t = 0.0
        self._pixel_to_mid_prev = 0.0
        self._pixel_height_prev = 0.0
        self._pic_width = 0.0
        self._picel_to_mid = 0.0
        self._pixel_height = 0.0

    def calculate_estimated_goal_pose(self, last_odom_x: float, last_odom_y: float, last_odom_quaternion):
        cur_yaw = quaternion_to_yaw(last_odom_quaternion)
        goal_x = last_odom_x + 6.0 * math.cos(cur_yaw)
        goal_y = last_odom_y + 6.0 * math.sin(cur_yaw)
        return goal_x, goal_y, cur_yaw

    def getOut(self):
        """Retrieve the output of the logic processing."""
        return self.__output
    
    def setActive(self):
        """Set the active state of the logic processing."""
        if self.__state == MainStates.IDLE:
            self.__state = MainStates.INIT_ROBOT
            return True
        return False

    def getActiveState(self):
        """Retrieve the current active state."""
        return self.__state

    def reset(self):
        """Reset the logic processing to its initial state."""
        self.__state = MainStates.IDLE
        self.__output.resetOut()
        self._o_l_x = 0.0
        self._o_l_z = 0.0
        self._o_t = 0.0
        self._pixel_to_mid_prev = 0.0
        self._pixel_height_prev = 0.0
        self._pic_width = 0.0
        self._pixel_to_mid = 0.0
        self._pixel_height = 0.0

    def setOdomData(self, x, z, t):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self._o_l_x = x
        self._o_l_z = z
        self._o_t = t

    def setCameraData(self, pToMid, pToMidPrev, pHeight, pHeightPrev, picWidth): 
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self._picel_to_mid = pToMid
        self._pixel_to_mid_prev = pToMidPrev
        self._pixel_height = pHeight
        self._pixel_height_prev = pHeightPrev
        self._pic_width = picWidth

    def state_machine(self):
        """Execute the state machine of the logic processing."""
        match self.__state:
            case MainStates.INIT:
                self.__state = MainStates.IDLE

            case MainStates.IDLE:
                pass # TODO

            case MainStates.INIT_ROBOT:
                pass # TODO

            case MainStates.DRIVE:
                pass # TODO

            case MainStates.TURN:
                pass # TODO

            case MainStates.PAUSE:
                pass # TODO

            case MainStates.FAILED:
                pass # TODO

            case MainStates.SUCCESS:
                pass # TODO

