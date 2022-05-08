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
            JibController.get_jib_angle(0),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAnglePi(self):
        self.assertEqual(JibController.get_jib_angle(math.pi), 0)

    def test_getSailAngle_apparentWindAngleNegativePi(self):
        self.assertEqual(JibController.get_jib_angle(-math.pi), 0)

    def test_getSailAngle_apparentWindAngleBetween0AndPi(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / (-math.pi) * 2
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAngleBetweenNegativePiAnd0(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(-2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / (-math.pi) * 2
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAngle_boundedBetweenPiAnd2Pi(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(3 / 2 * math.pi),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE /
            (-math.pi) * (math.pi / 2)
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAngleNegativeNotBounded(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(-2 + 10 * math.pi),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / (-math.pi) * 2
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_apparentWindAnglePositiveNotBounded(self):
        self.assertAlmostEqual(
            JibController.get_jib_angle(2 + 10 * math.pi),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / (-math.pi) * 2
            + sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE,
        )

    def test_getSailAngle_X1_varying(self):
        sailbot_constants.X1_JIB = math.pi / 4
        self.assertAlmostEqual(
            JibController.get_jib_angle(math.pi / 4 - 0.01),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE
        )
        sailbot_constants.X1_JIB = 0

    def test_getSailAngle_X2_varying(self):
        sailbot_constants.X2_JIB = 3 * math.pi / 4
        self.assertAlmostEqual(
            JibController.get_jib_angle(3 * math.pi / 4 + 0.01),
            0
        )
        sailbot_constants.X2_JIB = math.pi

    def test_getSailAngle_X1_X2_mid(self):
        sailbot_constants.X2_JIB = 3 * math.pi / 4 + 0.2
        sailbot_constants.X1_JIB = math.pi / 4 + 0.2
        self.assertAlmostEqual(
            JibController.get_jib_angle((sailbot_constants.X2_JIB + sailbot_constants.X1_JIB) / 2),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / 2
        )
        self.assertAlmostEqual(
            JibController.get_jib_angle((sailbot_constants.X2_JIB * 0.75 + sailbot_constants.X1_JIB * 0.25)),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE / 4
        )
        self.assertAlmostEqual(
            JibController.get_jib_angle((sailbot_constants.X2_JIB * 0.25 + sailbot_constants.X1_JIB * 0.75)),
            sailbot_constants.JIB_CONTROLLER_MAX_SAIL_ANGLE * 0.75
        )
        sailbot_constants.X2_JIB = math.pi
        sailbot_constants.X1_JIB = 0


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_JibController",
                   Test_JibController)
    sailbot_constants.X1_JIB = 0
    sailbot_constants.X2_JIB = math.pi
