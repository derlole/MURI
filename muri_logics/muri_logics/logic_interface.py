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
    def setActive() -> bool:
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
    def setOdomData(x: float, y:float, t: float):
        """Sets the data of the actual position of the robot, for the processing logic"""
        pass

    @abstractmethod
    def setCameraData(angle_in_rad: float, distance_in_meters: float): 
        """Sets the data of the camera, for the processing logic"""
        pass 

class ExtendedLogicInterface(LogicInterface):
    @abstractmethod
    def setArucoData(dominant_aruco_id: int):
        """Sets the data of the detected aruco-marker, for the processing logic"""
        pass
