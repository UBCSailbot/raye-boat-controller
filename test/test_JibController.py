#!/usr/bin/env python

# Need this because in Python2 regular division (/)
# is integer division. I.e. 3/2==1
from __future__ import division

import local_imports
import rostest
import unittest
import math
import sailbot_constants
from jib_controller import JibController

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_JibController(unittest.TestCase):
    def test_getSailAngle_apparentWindAngleZero(self):
        self.assertEqual(
            JibController.get_jib_angle(0, X1, X2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAnglePi(self):
        self.assertEqual(JibController.get_jib_angle(math.pi, X1, X2), 0)

    def test_getSailAngle_apparentWindAngleNegativePi(self):
        self.assertEqual(JibController.get_jib_angle(-math.pi, X1, X2), 0)

    def test_getSailAngle_apparentWindAngleBetween0AndPi(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(2, X1, X2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / (-math.pi) * 2
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAngleBetweenNegativePiAnd0(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(-2, X1, X2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / (-math.pi) * 2
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAngle_boundedBetweenPiAnd2Pi(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(3 / 2 * math.pi, X1, X2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE /
            (-math.pi) * (math.pi / 2)
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAngleNegativeNotBounded(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(-2 + 10 * math.pi, X1, X2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / (-math.pi) * 2
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAnglePositiveNotBounded(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(2 + 10 * math.pi, X1, X2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / (-math.pi) * 2
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_X1_varying(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(math.pi / 4 - 0.01, math.pi / 4, X2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE
        )

    def test_getSailAngle_X2_varying(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(3 * math.pi / 4 + 0.01, X1,  3 * math.pi / 4),
            0
        )

    def test_getSailAngle_X1_X2_mid(self):
        X2_TEST = 3 * math.pi / 4 + 0.2
        X1_TEST = math.pi / 4 + 0.2
        self.assertAlmostEqual(
            JibController.get_jib_angle((sailbot_constants.X2_JIB + sailbot_constants.X1_JIB) / 2, X1_TEST, X2_TEST),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / 2
        )
        self.assertAlmostEqual(
            JibController.get_jib_angle((sailbot_constants.X2_JIB * 0.75 + sailbot_constants.X1_JIB * 0.25), X1_TEST, X2_TEST),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / 4
        )
        self.assertAlmostEqual(
            JibController.get_jib_angle((sailbot_constants.X2_JIB * 0.25 + sailbot_constants.X1_JIB * 0.75), X1_TEST, X2_TEST),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE * 0.75
        )
      


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_JibController",
                   Test_JibController)
    X1 = 0
    X2 = math.pi
