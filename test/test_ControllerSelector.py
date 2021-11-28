#!/usr/bin/env python
import local_imports
import rostest
import unittest
import sailbot_constants
from controller_selector import ControllerSelector
from control_modes import ControlModes

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_ControllerSelector(unittest.TestCase):

    # Test for bad ControlMode input
    def test_bad_controlMode(self):
        with self.assertRaises(ValueError):
            ControllerSelector(0.1, 0, -1)

    # Test for switching on the switch interval

    def test_switch_interval(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE mode from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # No switch
        self.assertFalse(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_time = 0.5 * sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = 0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Even if exit condition is met for TACKABLE, no switch until switch interval timeout
        self.assertFalse(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_time = sailbot_constants.SWITCH_INTERVAL

        # Switch occurs
        self.assertTrue(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

        # Adjust time
        mock_time = 1.5 * sailbot_constants.SWITCH_INTERVAL

        # Even if exit condition is met for TACKABLE, no switch until switch interval timeout
        self.assertFalse(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_time = 2 * sailbot_constants.SWITCH_INTERVAL

        # Switch occurs
        self.assertTrue(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

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
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_speed *= 2
        mock_time += 0.5 * sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = 1.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # No switch
        self.assertFalse(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
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
            heading_error=0.5,
            boat_speed=mock_speed,
            current_time=mock_time
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
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
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
            heading_error=0.5,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        mock_time += sailbot_constants.MAX_TIME_FOR_TACKING

        # A switch should occur
        self.assertTrue(cs.switchControlMode(
            heading_error=0.5,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should still remain in TACKABLE
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

    # JIBE_ONLY tests

    # Entry to JIBE_ONLY from UNKNOWN

    def test_enter_jibe_initialzation(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0

        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)
        self.assertTrue(cs.getControlModeID() == ControlModes.JIBE_ONLY.value)

    # Remain in JIBE_ONLY since exit condition is not met

    def test_remain_jibe(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY mode from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # No switch
        self.assertFalse(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.JIBE_ONLY.value)

        # Adjust parameters
        mock_speed /= 2
        mock_time += 0.5 * sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = 1.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # No switch
        self.assertFalse(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertTrue(cs.getControlModeID() == ControlModes.JIBE_ONLY.value)

    # Exit JIBE_ONLY mode due to reaching the heading

    def test_exit_jibe_reachedHeading(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0

        # Enter JIBE_ONLY from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Adjust time to meet the switch interval
        mock_time += sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = -0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch occurs
        self.assertTrue(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should not have changed modes
        self.assertTrue(cs.getControlModeID() == ControlModes.JIBE_ONLY.value)

    # Exit TACKABLE due to timeout
    # This test assumes that the timeout interval is larger than the switch interval

    def test_exit_JIBE_timeout(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0

        # Enter TACKABLE from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Adjust time to meet the switch interval
        mock_time += sailbot_constants.SWITCH_INTERVAL

        # A switch should NOT occur
        self.assertFalse(cs.switchControlMode(
            heading_error=0.5,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        mock_time += sailbot_constants.MAX_TIME_FOR_JIBING

        # A switch should occur
        self.assertTrue(cs.switchControlMode(
            heading_error=0.5,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should still remain in JIBE_ONLY
        self.assertTrue(cs.getControlModeID() == ControlModes.JIBE_ONLY.value)

    # Integration tests

    # Enter TACKABLE mode and then swtich to JIBE_ONLY

    def test_tack_to_jibe(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0

        # Enter TACKABLE from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)

        # Adjust parameters to switch to JIBE_ONLY
        mock_time += sailbot_constants.SWITCH_INTERVAL
        mock_speed = 0.5 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS

        # A switch should occur
        self.assertTrue(cs.switchControlMode(
            heading_error=0.5,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should now be in JIBE_ONLY mode
        self.assertTrue(cs.getControlModeID() == ControlModes.JIBE_ONLY.value)

    # Enter JIBE_ONLY mode and then switch to TACKABLE

    def test_jibe_to_tack(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY from UNKNOWN
        cs = ControllerSelector(mock_speed, mock_time, ControlModes.UNKNOWN.value)
        self.assertTrue(cs.getControlModeID() == ControlModes.JIBE_ONLY.value)

        # Adjust parameters to switch to TACKABLE
        mock_time += sailbot_constants.SWITCH_INTERVAL
        mock_speed = 2 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS
        mock_heading_error = 0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch should occur
        self.assertTrue(cs.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should now be in JIBE_ONLY mode
        self.assertTrue(cs.getControlModeID() == ControlModes.TACKABLE.value)


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_ControllerSelector", Test_ControllerSelector)
