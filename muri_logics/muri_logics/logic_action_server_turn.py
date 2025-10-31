from enum import Enum
from  muri_logics.logic_interface import LogicInterface, Out

class TurnStates(Enum):
    INIT = 0
    IDLE = 1


    FAILED = 2
    SUCCESS = 3

class DriveOut(Out):
    pass

class TurnLogic(LogicInterface):
    pass