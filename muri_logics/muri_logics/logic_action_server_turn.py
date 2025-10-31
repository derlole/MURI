from enum import Enum
from  muri_logics.logic_interface import LogicInterface

class TurnStates(Enum):
    INIT = 0
    IDLE = 1


    FAILED = 2
    SUCCESS = 3

class TurnLogic(LogicInterface):
    pass