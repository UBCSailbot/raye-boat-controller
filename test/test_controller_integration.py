#! /usr/bin/env python
import local_imports
import unittest
import rospy
from time import sleep
import rostest
from sailbot_msg.msg import actuation_angle

# Do something with local_imports to avoid lint errors
local_imports.printMessage()

SLEEP_SECONDS = 1


class TestControllerIntegration(unittest.TestCase):
    controllerIntegration_ok = False

    def callback(self, data):
        self.controllerIntegration_ok = True

    def test_rudderAndSailAnglePublished(self):
        rospy.init_node("test_rudderAndSailAnglePublished")
        rospy.Subscriber(
            "/rudder_winch_actuation_angle", actuation_angle, self.callback
        )

        counter = 0
        while (
            not rospy.is_shutdown()
            and counter < 5
            and not (self.controllerIntegration_ok)
        ):
            sleep(SLEEP_SECONDS)
            counter += 1

        self.assertTrue(self.controllerIntegration_ok)


if __name__ == "__main__":
    rostest.rosrun(
        "boat_controller", "test_controller_integration", TestControllerIntegration
    )
