from jibe_controller import JibeController
from rudder_controller import RudderController
import sailbot_constants
import math
from sailbot_msg.msg import actuation_angle, heading, Sensors


class ControllerSelector:
    def __init__(self, initialControlMode):
        pass

    def getController(self, sensors, desiredHeading):
        if sensors.gps_can_groundspeed_knots < 0.5:
            return JibeController
        else:
            return RudderController
