from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional

class Out(ABC):
    @abstractmethod
    def getState(self) -> Enum:
        """Retrieve the current state."""
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
        """Access the state machine of the logic processing."""
        pass

    @abstractmethod
    def getActiveState() -> Enum:
        """Retrieve the current active state."""
        pass