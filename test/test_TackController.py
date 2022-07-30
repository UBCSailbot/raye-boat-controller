#!/usr/bin/env python
import local_imports
import rostest
import unittest
import sailbot_constants
from tack_controller import TackController
from math import pi

# TODO: Remove RudderController class

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_TackController(unittest.TestCase):
    def test_getFeedBackGain_headingErrorPositive(self):
        self.assertEqual(
            TackController.get_feed_back_gain(1.2),
            sailbot_constants.KP / (1 + sailbot_constants.CP * abs(1.2)),
        )

    def test_getFeedBackGain_headingErrorNegative(self):
        self.assertEqual(
            TackController.get_feed_back_gain(-1.2),
            sailbot_constants.KP / (1 + sailbot_constants.CP * abs(-1.2)),
        )

    def test_getFeedBackGain_symmetric(self):
        self.assertEqual(
            TackController.get_feed_back_gain(0.5),
            -TackController.get_feed_back_gain(-0.5),
        )

    def test_getFeedBackGain_headingErrorGreaterThanPi_valueErrorRaised(self):
        self.assertRaises(
            ValueError, TackController.get_feed_back_gain, 4 * pi)

    def test_getFeedBackGain_headingErrorBelowNegativePi_valueErrorRaised(self):
        self.assertRaises(
            ValueError, TackController.get_feed_back_gain, -4 * pi)

    def test_getHeadingErrorTackable(self):
        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(7 * pi / 8, -7 * pi / 8),
            -pi / 4,
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(-7 * pi / 8, 7 * pi / 8),
            pi / 4,
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(pi / 2, 0),
            pi / 2,
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(0, pi / 2), -pi / 2
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(pi / 4, -pi / 4),
            pi / 2
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(pi / 3, -3 * pi / 4),
            -11 * pi / 12,
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(4 * pi, -pi / 4 + 2 * pi),
            pi / 4,
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(3 * pi, 1.1 * pi),
            -0.1 * pi
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(pi / 4, 7 * pi / 4),
            pi / 2
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(3 * pi / 4, 5 * pi / 4),
            -pi / 2
        )

        self.assertAlmostEqual(
            TackController.get_heading_error_tackable(pi / 4, pi / 4),
            0.
        )


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_TackController",
                   Test_TackController)
