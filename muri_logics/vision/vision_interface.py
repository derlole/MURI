from abc import ABC, abstractmethod
from typing import Tuple, Optional
import numpy as np

class AMDInterface(ABC):
    @abstractmethod
    def calibrate_from_image(self, image_path: str) -> None:
        pass

    @abstractmethod
    def aruco_detection(self, img_path: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        pass
