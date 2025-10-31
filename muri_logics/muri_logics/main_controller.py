from enum import Enum
from  muri_logics.logic_interface import LogicInterface, Out

class MainStates(Enum):
    INIT = 0
    IDLE = 1
    

    FAILED = 3
    SUCCESS = 4

class MainOut(Out):
    pass

class MainController(LogicInterface):
    pass