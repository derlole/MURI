from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional

class Out(ABC):

    @property
    @abstractmethod
    def values(self):
        """Get the value."""
        pass

    @values.setter
    @abstractmethod
    def values(self, val):
        """Set the value."""
        pass

    @abstractmethod
    def resetOut(self):
        """Reset the output to its initial state."""
        pass

    @abstractmethod
    def outValid(self) -> bool:
        """Check if the output is valid."""
        pass

    @abstractmethod
    def getError(self) -> Optional[Exception]:
        """Retrieve any error state."""
        pass
    
    

class LogicInterface(ABC):
    @abstractmethod
    def getOut() -> Out:
        """Retrieve the output of the logic processing."""
        pass

    @abstractmethod
    def setActive(active: bool) -> bool:
        """Set the active state of the logic processing."""
        pass

    @abstractmethod
    def state_machine():
        """Execute the state machine of the logic processing."""
        pass

    @abstractmethod
    def getActiveState() -> Enum:
        """Retrieve the current active state."""
        pass

    @abstractmethod
    def reset():
        """Reset the logic processing to its initial state."""
        pass

    @abstractmethod
    def setOdomData(x: float, z:float, t: float):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        pass

    @abstractmethod
    def setCameraData(data): #TODO
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        pass 