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
        self.__values = {}
        self.__error = False
        self.isValid = False

    @property
    def values(self):
        """Get the value."""
        return self.__values

    @values.setter
    def values(self, ASToCall):
        """Set the value."""
        self.__values['ASToCall'] = ASToCall

    def resetOut(self):
        """Reset the output to its initial state."""
        self.__values = {}
        self.__error = None 
        self.isValid = False

    def outValid(self):
        """Check if the output is valid."""
        return self.isValid

    def getError(self):
        """Retrieve any error state."""
        return self.__error

class MainController(LogicInterface):
    def __init__(self):
        self.__state = MainStates.INIT
        self.__output = MainOut()
        self._o_l_x = 0.0
        self._o_l_y = 0.0
        self._o_t = 0.0
        self._angle_in_rad = 0.0
        self._distance_in_meters = 0.0
        self._goal_status_fin = False
        self._goal_success = False

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
            self.__output.values = 0
            self.__output.isValid = True
            return True
        return False

    def getActiveState(self):
        """Retrieve the current active state."""
        return self.__state

    def setGoalStautusFinished(self, gsf):
        self._goal_status_fin = gsf

    def setGoalSuccess(self, suc):
        self._goal_success = suc

    def reset(self):
        """Reset the logic processing to its initial state."""
        self.__state = MainStates.IDLE
        self.__output.resetOut()
        self._o_l_x = 0.0
        self._o_l_y = 0.0
        self._o_t = 0.0
        self._angle_in_rad = 0.0
        self._distance_in_meters = 0.0

    def setOdomData(self, x, y, t):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self._o_l_x = x
        self._o_l_y = y
        self._o_t = t

    def setCameraData(self, angle_in_rad, distance_in_meters): 
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self._angle_in_rad = angle_in_rad
        self._distance_in_meters = distance_in_meters

    def setPaused(self):
        self.__state = MainStates.PAUSE

    def calculateEstimatedProblems(self):
        pass #TODO
    
    def postInit(self):
        print('postInit')
        self.state_machine()
        self.setActive()

    def state_machine(self):
        """Execute the state machine of the logic processing."""
        self.__output.resetOut()
        match self.__state:
            case MainStates.INIT:
                self.__state = MainStates.IDLE

            case MainStates.IDLE:
                #print('mainstate idle')
                #self.__state = MainStates.INIT_ROBOT #remove this later
                #self.__output.values = 0 #remove this later
                #self.__output.isValid = True # remove this later
                pass

            case MainStates.INIT_ROBOT:
                print('mainstate init_robot' + str(self._goal_status_fin) + str(self._goal_success))
                self.__output.isValid = True
                
                if self._goal_status_fin and self._goal_success:
                    self.__state = MainStates.DRIVE
                    self._goal_status_fin = False
                    self._goal_success = False
                    self.__output.values = 1

            case MainStates.DRIVE:
                print('mainstate drive')
                self.__output.isValid = True
                self.calculateEstimatedProblems()
                if self._goal_status_fin and self._goal_success:
                    self.__state = MainStates.TURN
                    self._goal_status_fin = False
                    self._goal_success = False
                    self.__output.values = 2
                
            case MainStates.TURN:
                self.__output.isValid = True
                
                if self._goal_status_fin and self._goal_success:
                    self.__state = MainStates.DRIVE
                    self._goal_status_fin = False
                    self._goal_success = False
                    self.__output.values = 1

            case MainStates.PAUSE:
                pass # TODO

            case MainStates.FAILED:
                pass # TODO

            case MainStates.SUCCESS:
                pass # TODO

