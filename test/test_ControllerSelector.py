#!/usr/bin/env python
import local_imports
# import rostest
import unittest
# import sailbot_constants
from rudder_controller import RudderController
# from math import pi
from controller_selector import ControllerSelector
# from sailbot_msg.msg import actuation_angle
from sailbot_msg.msg import heading, Sensors
from control_modes import ControlModes
from jibe_controller import JibeController


# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_ControllerSelector(unittest.TestCase):
    def test_boatInUnknownMode_boatVelocityFast_simpleModeOfControlReturned(self):
        controllerSelector = ControllerSelector(ControlModes.UNKNOWN)
        sensors = Sensors(gps_can_groundspeed_knots=100000)
        desired_heading = heading(5)
        self.assertEqual(controllerSelector.getController(
            sensors, desired_heading), RudderController)

    def test_boatInUnknownMode_boatVelocitySlow_jibeOnlyControlModeReturned(self):
        controllerSelector = ControllerSelector(ControlModes.UNKNOWN)
        sensors = Sensors(gps_can_groundspeed_knots=0.01)
        desired_heading = heading(5)
        self.assertEqual(controllerSelector.getController(
            sensors, desired_heading), JibeController)

    def test_boatInJibeMode_newSetPointGiven_NonTimeOut_continueInJibeModeToPreviousSetpoint(self):
        pass

    def test_boatInJibeMode_timeOut_boatVelocityFast_switchToSimpleModeOfControlWithNewSetpoint(self):
        pass

    def test_boatInJibeMode_timeOut_boatVelocitySlow_stayInJibeOnlyControlWithNewSetpoint(self):
        pass


if __name__ == "__main__":
    unittest.main()
