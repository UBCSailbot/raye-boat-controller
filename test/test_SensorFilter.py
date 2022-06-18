#! /usr/bin/env python
import local_imports
import rostest
import unittest

from sensor_filter import SensorFilter

# To avoid errors from linting
local_imports.printMessage()

class Test_SensorFilter(unittest.TestCase):
    
    def test_invalid_type(self):
        reading = 4
        lowerbound = 1.0
        upperbound = 5.0
        expected_type = float
        self.assertFalse(SensorFilter.filter(reading, lowerbound, upperbound, expected_type))


    def test_invalid_nonetype(self):
        reading = None
        lowerbound = 1.0
        upperbound = 5.0
        expected_type = float
        self.assertFalse(SensorFilter.filter(reading, lowerbound, upperbound, expected_type))


    def test_below_bound(self):
        reading = 5.0
        lowerbound = 6.0
        upperbound = 8.0
        expected_type = float
        self.assertFalse(SensorFilter.filter(reading, lowerbound, upperbound, expected_type))


    def test_above_bound(self):
        reading = 8.0
        lowerbound = 1.0
        upperbound = 5.0
        expected_type = float
        self.assertFalse(SensorFilter.filter(reading, lowerbound, upperbound, expected_type))

    
    def test_correct_reading(self):
        reading = 3
        lowerbound = 1
        upperbound = 5
        expected_type = int
        self.assertTrue(SensorFilter.filter(reading, lowerbound, upperbound, expected_type))


if __name__ == "__main__":
    rostest.rosrun("boat_controller", "Test_SensorFilter", Test_SensorFilter)
