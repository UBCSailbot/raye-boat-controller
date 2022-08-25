#!/usr/bin/env python
import local_imports
import rostest
import unittest
import sailbot_constants
import math
from heading_controller import HeadingController
from control_modes import ControlModes
from jibe_only_rudder_controller import JibeOnlyRudderController
from tack_controller import TackController

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_HeadingController(unittest.TestCase):

    def test_badControlModeInput_ValueError(self):
        with self.assertRaises(ValueError):
            HeadingController(0.1, -1)

    # TACKABLE tests

    def test_initialize_tackableMode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1

        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    def test_noExit_tackableMode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE mode from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)

        # Adjust parameters
        mock_speed *= 2
        mock_heading_error = 1.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # No switch
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    def test_slowSpeed_exitTackable(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)

        # Make speed low
        mock_speed -= 0.2

        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should not be in TACKABLE or UNKNOWN
        self.assertNotEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)
        self.assertNotEqual(hc.getControlModeID(), ControlModes.UNKNOWN.value)

    def test_reachedHeading_exitTackable(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        mock_heading_error = -0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch occurs
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should not have changed modes
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    # JIBE_ONLY tests

    def test_initialize_jibeMode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1

        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    def test_noExit_jibeMode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY mode from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)

        # No switch
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        # Adjust parameters
        mock_speed /= 2
        mock_heading_error = 1.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # No switch
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    def test_reachedHeading_exitJibe(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        mock_heading_error = -0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch occurs
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should not have changed modes
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    # Integration tests

    def test_tack_to_jibe(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters to switch to JIBE_ONLY
        mock_speed = 0.5 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should now be in JIBE_ONLY mode
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    def test_jibe_to_tack(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        # Adjust parameters to switch to TACKABLE
        mock_speed = 2 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS
        mock_heading_error = 0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should now be in TACKABLE mode
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    def test_tack_error_function(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1

        # Enter TACKABLE mode from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        mock_current_heading = math.pi
        mock_desired_heading = math.pi / 2
        mock_wind_angle = math.pi / 4

        # Error function in heading controller should be the same as tack controller error functions
        self.assertAlmostEqual(hc.get_heading_error(
            mock_current_heading,
            mock_desired_heading,
            mock_wind_angle
        ), TackController.get_heading_error_tackable(
            mock_desired_heading,
            mock_current_heading
        ))

    def test_jibe_error_function(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1

        # Enter JIBE_ONLY mode from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        mock_current_heading = -math.pi
        mock_desired_heading = math.pi / 2
        mock_wind_angle = math.pi / 4

        jibe_direction = JibeOnlyRudderController.get_jibe_controller_direction(
            mock_current_heading,
            mock_desired_heading,
            mock_wind_angle
        )

        # Error function in heading controller should be the same as jibe controller error functions
        self.assertAlmostEqual(hc.get_heading_error(
            mock_current_heading,
            mock_desired_heading,
            mock_wind_angle
        ), JibeOnlyRudderController.get_jibe_controller_error(
            mock_current_heading,
            mock_desired_heading,
            jibe_direction
        ))

    def test_feedback_function(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1

        # Enter JIBE_ONLY mode from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        heading_error = -1.2

        # Generate List of evenly Spaced floats from -pi to pi
        testFloatList = []
        for i in range(0, (int) (4 * math.pi / 0.01)):
            testFloatList.append(-2 * math.pi + 0.01)

        self.assertEqual(
            hc.get_feed_back_gain(-1.2, 0),
            sailbot_constants.KP / (1 + sailbot_constants.CP * abs(heading_error)),
        )

        self.assertAlmostEqual(
            hc.get_feed_back_gain(-1.2, math.pi),
            sailbot_constants.MAX_ABS_RUDDER_ANGLE_RAD / (abs(heading_error) + 0.01),
        )

        # Test Symmetry of function for all possible cases
        for windAngle in testFloatList:
            for headingError in testFloatList:
                self.assertAlmostEqual(
                    hc.get_feed_back_gain(headingError, windAngle),
                    hc.get_feed_back_gain(-headingError, windAngle),
                )

        # Ensure function is positive and works for all possible cases
        for windAngle in testFloatList:
            for headingError in testFloatList:
                self.assertGreaterEqual(
                    hc.get_feed_back_gain(headingError, windAngle),
                    0,
                )

    def test_tack_to_low_power(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters to switch to LOW POWER
        low_battery = 0
        low_wind = 1

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            low_battery_level=low_battery,
            low_wind=low_wind
        ))

        # Should now be in LOW_POWER mode
        self.assertEqual(hc.getControlModeID(), ControlModes.LOW_POWER.value)

    def test_jibe_to_low_power(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1

        # Can be anything
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        # Adjust parameters to switch to LOW POWER
        # Only one or both of these need to be 1
        low_battery = 1
        low_wind = 0

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            low_battery_level=low_battery,
            low_wind=low_wind
        ))

        # Should now be in LOW_POWER mode
        self.assertEqual(hc.getControlModeID(), ControlModes.LOW_POWER.value)

    def test_switch_from_low_power(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1

        # Can be anything
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        # Adjust parameters to switch to LOW POWER
        # Only one or both of these need to be 1
        low_battery = 1
        low_wind = 1

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            low_battery_level=low_battery,
            low_wind=low_wind
        ))

        # Adjust parameters to switch to TACKABLE
        mock_speed = 2 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS
        mock_heading_error = 0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch should occur
        self.assertTrue(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should now be in TACKABLE mode
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    def test_tack_disabled_low_power(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value, True)
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters to switch to LOW POWER
        low_battery = 0
        low_wind = 1

        # A switch should NOT occur
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            low_battery_level=low_battery,
            low_wind=low_wind
        ))

        # Should now be in TACKABLE mode and not in LOW_POWER mode
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

    def test_jibe_disabled_low_power(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1

        # Can be anything
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value, True)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        # Adjust parameters to switch to LOW POWER
        # Only one or both of these need to be 1
        low_battery = 1
        low_wind = 0

        # A switch NOT should occur
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed,
            low_battery_level=low_battery,
            low_wind=low_wind
        ))

        # Should now be in JIBE_ONLY mode and not LOW_POWER
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

    def test_fixed_tack_mode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS + 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH
        fixedMode = ControlModes.TACKABLE.value

        # Enter TACKABLE from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value, fixedControlMode=fixedMode)
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)

        # Adjust parameters to switch to JIBE_ONLY
        mock_speed = 0.5 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS

        # A switch should NOT occur
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should still be in TACKABLE mode
        self.assertEqual(hc.getControlModeID(), ControlModes.TACKABLE.value)
        self.assertTrue(hc.controlModeIsFixed)

    def test_fixed_jibe_mode(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH
        fixedMode = ControlModes.JIBE_ONLY.value

        # Enter JIBE_ONLY from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value, fixedControlMode=fixedMode)
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)

        # Adjust parameters to switch to TACKABLE
        mock_speed = 2 * sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS
        mock_heading_error = 0.5 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH

        # A switch should NOT occur
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should still be in JIBE_ONLY mode
        self.assertEqual(hc.getControlModeID(), ControlModes.JIBE_ONLY.value)
        self.assertTrue(hc.controlModeIsFixed)

    def test_fixed_low_power(self):
        mock_speed = sailbot_constants.SPEED_THRESHOLD_FOR_JIBING_KNOTS - 0.1
        mock_heading_error = 2 * sailbot_constants.MIN_HEADING_ERROR_FOR_SWITCH
        fixedMode = ControlModes.LOW_POWER.value

        # Enter LOWER_POWER from UNKNOWN
        hc = HeadingController(mock_speed, ControlModes.UNKNOWN.value, fixedControlMode=fixedMode)
        self.assertEqual(hc.getControlModeID(), ControlModes.LOW_POWER.value)

        # A switch should NOT occur
        self.assertFalse(hc.switchControlMode(
            heading_error=mock_heading_error,
            boat_speed=mock_speed
        ))

        # Should now be in LOW_POWER mode
        self.assertEqual(hc.getControlModeID(), ControlModes.LOW_POWER.value)
        self.assertTrue(hc.controlModeIsFixed)


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_HeadingController", Test_HeadingController)
