from enum import Enum
from  muri_logics.logic_interface import LogicInterface, Out


class InitStates(Enum):
    INIT = 0
    IDLE = 1
    FAILED = 2
    SUCCESS = 3
    RAEDY = 4 
    INITMOVE = 5


class InitOut(Out):
    def __init__(): 
        self.__values = None
        pass #TODO

    @property
    def values(self):
        """Get the value."""
        return self.__values

    @values.setter
    def values(self, data):
        """Set the value."""
        lvx, lvy, avz, ta = data

        if lvx is not None:
            self.__values['linear_velocity_x'] = lvx
        
        if lvy is not None:
            self.__values['lineat_velocity_y'] = lvy

        if avz is not None:
            self.__values['angular_velocity_z'] = avz

        if ta is not None:
            self.__values['turned_angle'] = ta

       
    def resetOut(self):
        """Reset the output to its initial state."""
        self.__values = None
    
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
        self.__output.resetOut()
        self.__state = InitStates.IDLE

    def setOdomData(self, x, z, t):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__positionX = x
        self.__positionY = y
        self.__positionTheta = t


    def setCameraData(self, pToMid, pToMidPrev, pHeight, pHeightPrev, picWidth):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__pixelToMid = pToMid
        self.__picelToMidPrev = pToMidPrev
        self.__pixelHeight = pHeight
        self.__pixelHeightPrev = pHeightPrev
        self.__pictureWidth = picWidth
    
    def calculate(self):
        
        PIXELTOLLERANCE = 5
        angularVelocityZ = 0

        if self.__pixelToMid > 0 and abs(self.__pixelToMid) > PIXELTOLLERANCE:
            angularVelocityZ = 1

        elif self.__pixelToMid < 0 and abs(self.__pixelToMid) > PIXELTOLLERANCE:
            angularVelocityZ = -1
        
        else:
            angularVelocityZ = 0
        
        return angularVelocityZ


    
    def state_machine(self):
            """Execute the state machine of the logic processing."""
        
        match self.__state:

            case InitStates.INIT:
                self.__.output.values = (0.0, 0.0, 0.0, 0.0)
                self.__positionX = 0.0
                self.__positionY = 0.0
                self.__positionTheta = 0.0
                self.__camaeraData = None
                self.__state = InitStates.IDLE
            
            case InitStates.IDLE:
                pass

            case InitStates.RAEDY:
                self.__state = InitStates.INITMOVE

            case InitStates.INITMOVE:
                avz, ta = self.calculate()
                self.__output.values = (None, None, avz, ta)
                if avz = 0:
                    self.__state = InitStates.SUCCESS
                
                


            case InitStates.FAILED:
                pass #TODO

            case InitStates.SUCCESS:
                pass #TODO


                
