from jibe_controller import JibeController
from rudder_controller import RudderController
from control_modes import ControlModes
import sailbot_constants
import math
from sailbot_msg.msg import actuation_angle, heading, Sensors


class ControllerSelector:

     self.controllerMappings = {
        ControlModes.JIBE_ONLY : JibeController.
        ControlModes.TACKABLE : RudderController
    }

    self.controlMode = None
    self.controlModeID = None
    self.lastSwitchTime = None

    def __init__(self, unix_timestamp, initialControlMode=ControlModes.UNKNOWN):
        # Error handling
        if (not self.isValidModeID(initialControlMode)):
           raise ValueError("An invalid control mode was passed as an argument to the heading controller")
        
        #TODO: Call a switch function that switches to a controller based on sensor readings
        if(initialControlMode == ControlModes.UNKNOWN):
            pass
        else:
            self.controlMode = self.controllerMappings.get(initialControlMode, RudderController)
        self.controlModeID = ControlModes(self.controlMode)
        self.lastSwitchTime = unix_timestamp

    def switchControlMode(self, sensors):
        pass

    def __switchFromUnknown(self):
        pass

    def __switchFromTacking(self):
        pass

    def __switchFromJibing(self):
        pass

    def switchControlModeByID(self, modeID):
        if (not self.isValidModeID(modeID)):
           raise ValueError("An invalid control mode was passed as an argument to the heading controller")

        self.controlMode = self.controllerMappings.get(modeID, self.controlMode)
        return self.controlMode

    @staticmethod
    def getController(sensors, desiredHeading):
        if sensors.gps_can_groundspeed_knots < 0.5:
            return JibeController
        else:
            return RudderController

    @staticmethod
    def isValidModeID(modeID):
        return (ControlModes.JIBE_ONLY <= modeID <= ControlModes.UNKNOWN) and
               (isinstance(modeID, int))

