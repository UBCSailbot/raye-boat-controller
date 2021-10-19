from control_modes import ControlModes

MIN_TACKING_SPEED = 0.5

class HeadingController:

    def __init__(self, initialControlMode=ControlModes.UNKNOWN):
        pass
        

    def getControlMode(self):
        return self.controlMode
    
    def switchControlMode(self, sensors):
        pass

    def switchControlModeByID(self, modeID):
        pass
