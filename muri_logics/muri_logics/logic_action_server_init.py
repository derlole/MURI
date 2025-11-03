from enum import Enum
from  muri_logics.logic_interface import LogicInterface, Out

class InitStates(Enum):
    INIT = 0
    IDLE = 1
    FAILED = 2
    SUCCESS = 3
    RUN = 4 #EVTL. Sinfolle namen für RUN und MOVE
    MOVE = 5

class InitOut(Out):
    def __init__(): 
        self.__values = None #TODO
        pass #TODO

    @property
    def values(self):
        """Get the value."""
        pass #TODO

    @values.setter
    def values(self, val):
        """Set the value."""
        pass #TODO

    def getState(self):
        """Retrieve the current state."""
        pass #TODO

    def resetOut(self):
        """Reset the output to its initial state."""
        pass #TODO
    
    def outValid(self):
        """Check if the output is valid."""
        pass #TODO

    def getError(self):
        """Retrieve any error state."""
        pass #TODO


class InitLogic(LogicInterface):
    def __init__():
        self.__output = InitOut()
        self.__state = InitStates.INIT


    def getOut(self):
        """Retrieve the output of the logic processing."""
        return self.__output

    def setActive(active): # TODO
        """Set the active state of the logic processing."""
        self.__state = InitStates.RUN
        return True

    def getActiveState(self):
        """Retrieve the current active state."""
        return self.__state

    def reset():
        """Reset the logic processing to its initial state."""
        self.__positionX = 0.0
        self.__positionY = 0.0
        self.__positionTheta = 0.0
        self.__camaeraData = None
        self.__output.resetOut()
        self.__state = InitStates.IDLE

    def setOdomData(self, x, z, t):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__positionX = x
        self.__positionY = y
        self.__positionTheta = t


    def setCameraData(self, data):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__camaeraData = data
    
    def calculate():
        pass #TODO

    
    def state_machine(self):
            """Execute the state machine of the logic processing."""
        
        match self.__state:

            case InitStates.INIT:
                self.__positionX = 0.0
                self.__positionY = 0.0
                self.__positionTheta = 0.0
                self.__camaeraData = None
                self.__state = InitStates.IDLE
            
            case InitStates.IDLE:
                pass

            case InitStates.RUN:
                self.__state = InitStates.MOVE

            case InitStates.MOVE:
                self.calculate()

            case InitStates.FAILED:
                pass #TODO

            case InitStates.SUCCESS:
                pass #TODO


                
