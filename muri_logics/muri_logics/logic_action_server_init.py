from enum import Enum
from muri_logics.logic_interface import LogicInterface, Out
from general_funcs import quaternion_to_yaw



class InitStates(Enum):
    INIT = 0
    IDLE = 1
    FAILED = 2
    SUCCESS = 3
    RAEDY = 4 
    INITMOVE = 5



class Constants():
    PIXELTOLLERANCE = 5



class InitOut(Out):
    def __init__(self): 
        self.__values = None
        self.__error = None
        self.__isValid = False

    @property
    def isValid(self):
        """Get the isValid"""
        return self.__isValid

    @isValid.setter
    def isValid(self, hiV):
        """Set the isValid"""
        self.__isValid = hiV

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
        self.__isValid = False
    
    def outValid(self):
        """Check if the output is valid."""
        return self.__isValid

    def getError(self): #TODO Exeption?
        """Retrieve any error state."""
        return self.__error 
    
    def setError(self, er):
        """Set the error"""
        self.__error = er



class InitLogic(LogicInterface):
    def __init__(self):
        self.__output = InitOut()
        self.__state = InitStates.INIT
        self.__firstTheta = None
        self.state_machine()

    def getOut(self):
        """Retrieve the output of the logic processing."""
        return self.__output

    def setActive(self):
        """Set the active state of the logic processing."""
        if self.__state == InitStates.IDLE:
            self.__state = InitStates.RAEDY
            return True
        return False

    def getActiveState(self):
        """Retrieve the current active state."""
        return self.__state

    def reset(self):
        """Reset the logic processing to its initial state."""
        self.__positionX = 0.0
        self.__positionY = 0.0
        self.__positionTheta = 0.0
        self.__firstTheta = None
        self.__output.resetOut()
        self.__state = InitStates.IDLE

    def setOdomData(self, x, y, t):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__positionX = x
        self.__positionY = y
        self.__positionTheta = quaternion_to_yaw(t)


    def setCameraData(self, pToMid, pToMidPrev, pHeight, pHeightPrev, picWidth):
        """Sets the Data of the actual Position of the Robot, for the Processing Locig"""
        self.__pixelToMid = pToMid
        self.__picelToMidPrev = pToMidPrev
        self.__pixelHeight = pHeight
        self.__pixelHeightPrev = pHeightPrev
        self.__pictureWidth = picWidth
    
    def calculate(self):
        """Calculate the Angle to Turn and set die Angelvelosity"""
        angularVelocityZ = 0.0
        tuerndAngle = self.__positionTheta - self.__firstTheta

        if self.__pixelToMid > 0 and abs(self.__pixelToMid) > Constants.PIXELTOLLERANCE:
            angularVelocityZ = 0.4 #TODO Auswahl Geschwindigkeit / Regelung

        elif self.__pixelToMid < 0 and abs(self.__pixelToMid) > Constants.PIXELTOLLERANCE:
            angularVelocityZ = -0.4
        
        else:
            angularVelocityZ = 0.0
        
        return angularVelocityZ, tuerndAngle


    
    def state_machine(self):
        """Execute the state machine of the logic processing."""

        match self.__state:

            case InitStates.INIT:
                self.__output.values = (0.0, 0.0, 0.0, 0.0)
                self.__positionX = 0.0
                self.__positionY = 0.0
                self.__positionTheta = 0.0
                self.__camaeraData = None #TODO setze ich die camaradaten so richtig zurück?
                self.__state = InitStates.IDLE
            
            case InitStates.IDLE:
                pass

            case InitStates.RAEDY:
                if self.__firstTheta is None:
                    self.__firstTheta = self.__positionTheta
                self.__state = InitStates.INITMOVE

            case InitStates.INITMOVE:
                avz, ta = self.calculate()
                self.__output.values = (None, None, avz, ta)
                self.__output.isValid = True
                if avz == 0.0:
                    self.__state = InitStates.SUCCESS
                
                #TODO wenn zu weit gedreht gehe in FAILED

            case InitStates.FAILED:
                self.__output.setError(True)

            case InitStates.SUCCESS:
                pass


                
