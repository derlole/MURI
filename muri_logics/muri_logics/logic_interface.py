from abc import ABC, abstractmethod

class LogicInterface(ABC):
    @abstractmethod
    def getOut():
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