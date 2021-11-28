#!/usr/bin/env python
import local_imports
# import rostest
import unittest
from python.sailbot_constants import SWITCH_INTERVAL
import sailbot_constants
from rudder_controller import RudderController
from controller_selector import ControllerSelector
from control_modes import ControlModes
from tack_controller import TackController
from jibe_only_rudder_controller import JibeOnlyRudderController


# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_ControllerSelector(unittest.TestCase):
    
    # TACKABLE tests
    
    # Entry to TACKABLE from UNKNOWN
    def test_enter_tack_initialzation(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0

        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

    # Remain in TACKABLE since exit condition is not met
    def test_remain_tack(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH
        
        # Enter TACKABLE mode from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # No switch
        self.assertFalse(cs.switchControlMode(
            heading_error  = mock_heading_error,
            boat_speed     = mock_speed,
            current_time   = mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_speed *= 2
        mock_time += 0.5 * sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = 1.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWTICH

        # No switch
        self.assertFalse(cs.switchControlMode(
            heading_error  = mock_heading_error,
            boat_speed     = mock_speed,
            current_time   = mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)


    # Exit TACKABLE due to slow speed
    def test_exit_tack_tooSlow(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0

        # Enter TACKABLE from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Make speed low and adjust time to meet the switch interval
        mock_speed -= 0.2
        mock_time += sailbot_constants.SWITCH_INTERVAL

        self.assertTrue(cs.switchControlMode(
            heading_error  = 0.5,
            boat_speed     = mock_speed,
            current_time   = mock_time
        ))

        # Should not be in TACKABLE or UNKNOWN
        self.assertTrue(cs.getControlModeID() != ControlModes.TACKABLE.value)
        self.assertTrue(cs.getControlModeID() != ControlModes.UNKNOWN.value)

    # Exit TACKABLE due to reaching the heading
    def test_exit_tack_reachedHeading(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0

        # Enter TACKABLE from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Adjust time to meet the switch interval
        mock_time += sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = -0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch occurs
        self.assertTrue(cs.switchControlMode(
            heading_error  = mock_heading_error,
            boat_speed     = mock_speed,
            current_time   = mock_time
        ))

        # Should not have changed modes
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

    # Exit TACKABLE due to timeout
    # This test assumes that the timeout interval is larger than the switch interval
    def test_exit_tack_timeout(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0

        # Enter TACKABLE from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Adjust time to meet the switch interval
        mock_time += sailbot_constants.SWITCH_INTERVAL

        # A switch should NOT occur
        self.assertFalse(cs.switchControlMode(
            heading_error  = 0.5,
            boat_speed     = mock_speed,
            current_time   = mock_time
        ))

        mock_time += sailbot_constants.MAX_TIME_FOR_TACKING
        print(mock_time)

        # A switch should occur
        self.assertTrue(cs.switchControlMode(
            heading_error  = 0.5,
            boat_speed     = mock_speed,
            current_time   = mock_time
        ))

        # Should still remain in TACKABLE
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)
        
        

       

if __name__ == "__main__":
    unittest.main()
