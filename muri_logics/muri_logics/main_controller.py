import math

from enum import Enum
from  muri_logics.logic_interface import LogicInterface, Out
from muri_logics.general_funcs import quaternion_to_yaw

class MainStates(Enum):
    INIT = 0
    IDLE = 1
    

    FAILED = 3
    SUCCESS = 4

class MainOut(Out):
    pass

class MainController(LogicInterface):
    def __init__(self):
        super().__init__()
        self.state = MainStates.INIT

    def calculate_estimated_goal_pose(self, last_odom_x: float, last_odom_y: float, last_odom_quaternion):
        cur_yaw = quaternion_to_yaw(last_odom_quaternion)
        goal_x = last_odom_x + 6.0 * math.cos(cur_yaw)
        goal_y = last_odom_y + 6.0 * math.sin(cur_yaw)
        return goal_x, goal_y, cur_yaw
        