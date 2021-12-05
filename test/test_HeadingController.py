#!/usr/bin/env python
import local_imports
import rostest
import unittest
import sailbot_constants
from heading_controller import HeadingController
from control_modes import ControlModes

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_HeadingController(unittest.TestCase):

    def test_badControlModeInput_ValueError(self):
        with self.assertRaises(ValueError):
            HeadingController(0.1, 0, -1)

    def test_switch_interval(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE mode from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # No switch
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_time = 0.5 * sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = 0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Even if exit condition is met for TACKABLE, no switch until switch interval timeout
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_time = sailbot_constants.SWITCH_INTERVAL

        # Switch occurs
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust time
        mock_time = 1.5 * sailbot_constants.SWITCH_INTERVAL

        # Even if exit condition is met for TACKABLE, no switch until switch interval timeout
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_time = 2 * sailbot_constants.SWITCH_INTERVAL

        # Switch occurs
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    # TACKABLE tests

    def test_initialize_tackableMode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0

        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    def test_noExit_tackableMode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE mode from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # No switch
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters
        mock_speed *= 2
        mock_time += 0.5 * sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = 1.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # No switch
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    def test_slowSpeed_exitTackable(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Make speed low and adjust time to meet the switch interval
        mock_speed -= 0.2
        mock_time += sailbot_constants.SWITCH_INTERVAL

        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should not be in TACKABLE or UNKNOWN
        self.assertNotEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)
        self.assertNotEqual(hc.getControlModeID(), ControlModes.UNKNOWN.value)

    def test_reachedHeading_exitTackable(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Adjust time to meet the switch interval
        mock_time += sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = -0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch occurs
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should not have changed modes
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    # This test assumes that the timeout interval is larger than the switch interval

    def test_timeout_exitTackable(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Adjust time to meet the switch interval
        mock_time += sailbot_constants.SWITCH_INTERVAL

        # A switch should NOT occur
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        mock_time += sailbot_constants.MAX_TIME_FOR_TACKING

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should still remain in TACKABLE
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    # JIBE_ONLY tests

    def test_initialize_jibeMode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0

        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    def test_noExit_jibeMode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY mode from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # No switch
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        # Adjust parameters
        mock_speed /= 2
        mock_time += 0.5 * sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = 1.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # No switch
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    def test_reachedHeading_exitJibe(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Adjust time to meet the switch interval
        mock_time += sailbot_constants.SWITCH_INTERVAL
        mock_heading_error = -0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch occurs
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should not have changed modes
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    # This test assumes that the timeout interval is larger than the switch interval

    def test_timeout_exitJibe(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)

        # Adjust time to meet the switch interval
        mock_time += sailbot_constants.SWITCH_INTERVAL

        # A switch should NOT occur
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        mock_time += sailbot_constants.MAX_TIME_FOR_JIBING

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should still remain in JIBE_ONLY
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    # Integration tests

    def test_tack_to_jibe(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters to switch to JIBE_ONLY
        mock_time += sailbot_constants.SWITCH_INTERVAL
        mock_speed = 0.5 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should now be in JIBE_ONLY mode
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    def test_jibe_to_tack(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_time = 0
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, mock_time, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        # Adjust parameters to switch to TACKABLE
        mock_time += sailbot_constants.SWITCH_INTERVAL
        mock_speed = 2 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS
        mock_heading_error = 0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            current_time=mock_time
        ))

        # Should now be in JIBE_ONLY mode
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_HeadingController", Test_HeadingController)
