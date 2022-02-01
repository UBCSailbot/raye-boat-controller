#!/usr/bin/env python
from jib_controller import JibController
from sail_controller import SailController
from rudder_controller import RudderController
from controller_output_refiner import ControllerOutputRefiner
import sailbot_constants
import rospy
import threading
from sailbot_msg.msg import actuation_angle, heading, Sensors
import math

lock = threading.Lock()
# define global variables for the needed topics
headingSetPointRad = None
headingMeasureRad = None
apparentWindAngleRad = None
rudderAngleDegrees = 0
sailAngleDegrees = 0
jibAngleDegrees = 0

rudder_winch_actuation_angle_pub = rospy.Publisher(
    "/rudder_winch_actuation_angle", actuation_angle, queue_size=1
)


def sensorsCallBack(sensors_msg_instance):
    lock.acquire()

    global headingMeasureRad, apparentWindAngleRad
    headingMeasureRad = sensors_msg_instance.gps_can_true_heading_degrees * math.pi / 180
    apparentWindAngleRad = sensors_msg_instance.wind_sensor_1_angle_degrees * math.pi / 180

    publishRudderWinchAngle()
    lock.release()


def desiredHeadingCallBack(heading_msg_instance):
    lock.acquire()

    global headingSetPointRad
    headingSetPointRad = heading_msg_instance.headingDegrees * math.pi / 180

    publishRudderWinchAngle()
    lock.release()


def publishRudderWinchAngle():
    if (
        headingSetPointRad is not None
        and headingMeasureRad is not None
        and apparentWindAngleRad is not None
    ):

        global rudderAngleDegrees
        heading_error_tackable = RudderController.get_heading_error_tackable(
            headingSetPointRad, headingMeasureRad
        )
        rudderAngleDegrees = (
            RudderController.get_feed_back_gain(heading_error_tackable)
            * heading_error_tackable
            * 180
            / math.pi
        )

        global sailAngleDegrees
        sailAngleDegrees = (
            SailController.get_sail_angle(apparentWindAngleRad) * 180 / math.pi
        )

        global jibAngleDegrees
        jibAngleDegrees = (
            JibController.get_jib_angle(apparentWindAngleRad) * 180 / math.pi
        )

        rudder_winch_actuation_angle_pub.publish(
            ControllerOutputRefiner.saturate(
                rudderAngleDegrees,
                sailbot_constants.MAX_ABS_RUDDER_ANGLE_DEG,
                -sailbot_constants.MAX_ABS_RUDDER_ANGLE_DEG),

            ControllerOutputRefiner.saturate(
                sailAngleDegrees,
                sailbot_constants.MAX_ABS_SAIL_ANGLE_DEG,
                -sailbot_constants.MAX_ABS_SAIL_ANGLE_DEG),
            ControllerOutputRefiner.saturate(
                jibAngleDegrees,
                sailbot_constants.MAX_ABS_JIB_ANGLE_DEG,
                -sailbot_constants.MAX_ABS_JIB_ANGLE_DEG)
        )


def main():
    rospy.init_node("rudder_and_sail_angle_publisher", anonymous=True)
    rospy.Subscriber("/desired_heading_degrees",
                     heading, desiredHeadingCallBack)
    rospy.Subscriber("/sensors", Sensors, sensorsCallBack)
    rospy.spin()


if __name__ == "__main__":
    main()
