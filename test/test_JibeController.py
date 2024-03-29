#!/usr/bin/env python

# Need this because in Python2 regular division (/) is integer division.
# I.e. 3/2==1
from __future__ import division
import local_imports
import rostest
import unittest
import math
from jibe_only_rudder_controller import JibeOnlyRudderController

# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_JibeController(unittest.TestCase):

    def test_getJibeDirection_normalCase(self):
        self.assertEqual(
            JibeOnlyRudderController.get_jibe_controller_direction(
                math.pi / 2, - math.pi / 2, - math.pi / 2), 1)

    def test_getJibeError_normalCase(self):
        self.assertAlmostEqual(
            JibeOnlyRudderController.get_jibe_controller_error(
                math.pi / 2, - math.pi / 2, -1), -math.pi)

    def test_getJibeDirection_normalCaseInverse(self):
        self.assertEqual(
            JibeOnlyRudderController.get_jibe_controller_direction(
                math.pi / 2, - math.pi / 2, math.pi / 2), -1)

    def test_getJibeError_normalCaseInverse(self):
        self.assertAlmostEqual(
            JibeOnlyRudderController.get_jibe_controller_error(
                math.pi / 2, - math.pi / 2, 1), math.pi)

    def test_getJibeDirection_smallAngle(self):
        self.assertEqual(
            JibeOnlyRudderController.get_jibe_controller_direction(
                math.pi / 100, - math.pi / 100, math.pi * 99 / 100), -1)

    def test_getJibeError_smallAngle(self):
        self.assertAlmostEqual(JibeOnlyRudderController.get_jibe_controller_error(
            math.pi / 100, - math.pi / 100, 1), (99 / 100) * 2 * math.pi)

    def test_getJibeDirection_wrap(self):
        self.assertEqual(JibeOnlyRudderController.get_jibe_controller_direction(
            math.pi / 2 + 2 * math.pi, - math.pi / 2 + 2 * math.pi, - math.pi / 2 + 2 * math.pi), 1)

    def test_getJibeError_wrap(self):
        self.assertAlmostEqual(JibeOnlyRudderController.get_jibe_controller_error(
            math.pi / 2 + 2 * math.pi, - math.pi / 2 + 2 * math.pi, -1), -math.pi)

    def test_getJibeDirection_Same(self):
        self.assertEqual(
            JibeOnlyRudderController.get_jibe_controller_direction(
                math.pi / 100, - math.pi / 100, math.pi / 4), -1)

    def test_getJibeDirection_Same_Opp(self):
        self.assertEqual(
            JibeOnlyRudderController.get_jibe_controller_direction(
                - math.pi / 100, math.pi / 100, math.pi / 4), 1)


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_JibeController",
                   Test_JibeController)
