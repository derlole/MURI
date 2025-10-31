from abc import ABC, abstractmethod
from enum import Enum

class Out(ABC):
    @abstractmethod
    def getState(self) -> Enum:
        """Retrieve the current state."""
        pass

    def resetOut(self):
        """Reset the output to its initial state."""
        pass

    def outValid(self) -> bool:
        """Check if the output is valid."""
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