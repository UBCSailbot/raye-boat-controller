#! /usr/bin/env python
import unittest
import rospy
from time import sleep
import rostest
from sailbot_msg.msg import actuation_angle


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
