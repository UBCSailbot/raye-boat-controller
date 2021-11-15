#!/usr/bin/env python
import local_imports
import rostest
import unittest
from controller_output_refiner import ControllerOutputRefiner
import math
# Do something with local_imports to avoid lint errors
local_imports.printMessage()


class Test_ControllerOutputRefiner(unittest.TestCase):

    # inputSignal is within the bounds
    def test_saturate_inputWithinBound(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(math.pi, 2 * math.pi, -2 * math.pi),
            math.pi
        )

    # inputSignal is greater than upperBound
    def test_saturate_greaterThanUpperBound(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(3.5, 3, -1),
            3
        )

    # inputSignal is smaller than lowerBound
    def test_saturate_smallerThanLowerBound(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(-15, 10, -10),
            -10
        )

    # Corner cases where input signals are close to upper and lower bounds
    def test_saturate_justAboveUpperBound(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(math.pi + (1 / 11) ** 2.1, math.pi, 0),
            math.pi
        )

    def test_saturate_justBelowUpperBound(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(math.pi - (1 / 100000) ** 2.43, math.pi, 0),
            math.pi - (1 / 100000) ** 2.43
        )

    def test_saturate_justAboveLowerBound(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(math.pi ** 3.11, 100, math.pi ** 3.11 - (1 / 149)**16.03),
            math.pi ** 3.11
        )

    def test_saturate_justBelowLowerBound(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(10, 20, 10.0000000001 - (1 / 343443) ** 2.45435),
            10.0000000001 - (1 / 343443) ** 2.45435
        )

    # Assertion error check when lowerBound > upperBound
    def test_saturate_badBoundInputs(self):
        with self.assertRaises(AssertionError):
            ControllerOutputRefiner.saturate(0, 10, 10.0000001)

    # Case where upperBound == lowerBound
    def test_saturate_equalUpperAndLower1(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(math.pi, math.pi, math.pi),
            math.pi
        )

    def test_saturate_equalUpperAndLower2(self):
        self.assertEqual(
            ControllerOutputRefiner.saturate(3.00001, 3, 3),
            3
        )


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_ControllerOutputRefiner", Test_ControllerOutputRefiner)
