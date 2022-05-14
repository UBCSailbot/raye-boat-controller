#!/usr/bin/env python
from jib_controller import JibController
from sail_controller import SailController
from heading_controller import HeadingController
from controller_output_refiner import ControllerOutputRefiner
import sailbot_constants
import rospy
import threading
from sailbot_msg.msg import actuation_angle, heading, windSensor, GPS
import math

lock = threading.Lock()
# define global variables for the needed topics
headingSetPointRad = None
headingMeasureRad = None
apparentWindAngleRad = None
groundspeedKnots = None
rudderAngleRad = 0
sailAngle = 0
jibAngle = 0
controller = HeadingController(boat_speed=0)

rudder_winch_actuation_angle_pub = rospy.Publisher(
    "/rudder_winch_actuation_angle", actuation_angle, queue_size=1
)


def windSensorCallBack(windsensor_msg_instance):
    lock.acquire()

    global apparentWindAngleRad

    apparentWindAngleDegrees = windsensor_msg_instance.measuredDirectionDegrees
    apparentWindAngleRad = apparentWindAngleDegrees * sailbot_constants.DEGREES_TO_RADIANS

    publishRudderWinchAngle()
    lock.release()


def gpsCallBack(gps_msg_instance):
    lock.acquire()

    global headingMeasureRad, groundspeedKnots

    headingMeasureDegrees = gps_msg_instance.headingDegrees
    headingMeasureRad = headingMeasureDegrees * sailbot_constants.DEGREES_TO_RADIANS

    groundspeedKMPH = gps_msg_instance.speedKmph
    groundspeedKnots = groundspeedKMPH * sailbot_constants.KMPH_TO_KNOTS

    publishRudderWinchAngle()
    lock.release()


def desiredHeadingCallBack(heading_msg_instance):
    lock.acquire()

    global headingSetPointRad
    headingSetPointRad = heading_msg_instance.headingDegrees * sailbot_constants.DEGREES_TO_RADIANS

    publishRudderWinchAngle()
    lock.release()


def publishRudderWinchAngle():
    if (
        headingSetPointRad is not None
        and headingMeasureRad is not None
        and apparentWindAngleRad is not None
        and groundspeedKnots is not None
    ):

        global rudderAngleRad

        heading_error = controller.get_heading_error(
            current_heading=headingMeasureRad,
            desired_heading=headingSetPointRad,
            apparent_wind_angle=apparentWindAngleRad
        )

        controller.switchControlMode(
            heading_error=heading_error,
            boat_speed=groundspeedKnots
        )

        rudderAngleRad = (
            controller.get_feed_back_gain(heading_error) * heading_error
        )

        global sailAngle
        sailAngle = (
            int(SailController.get_sail_angle(apparentWindAngleRad) * (360 / (math.pi / 2)))
        )

        global jibAngle
        jibAngle = (
            int(JibController.get_jib_angle(apparentWindAngleRad) * (360 / (math.pi / 2)))
        )

        rudder_winch_actuation_angle_pub.publish(
            ControllerOutputRefiner.saturate(
                rudderAngleRad,
                sailbot_constants.MAX_ABS_RUDDER_ANGLE_RAD,
                -sailbot_constants.MAX_ABS_RUDDER_ANGLE_RAD),
            ControllerOutputRefiner.saturate(
                sailAngle,
                sailbot_constants.MAX_WINCH_POSITION,
                sailbot_constants.MIN_WINCH_POSITION),
            ControllerOutputRefiner.saturate(
                jibAngle,
                sailbot_constants.MAX_WINCH_POSITION,
                sailbot_constants.MIN_WINCH_POSITION)
        )


def main():
    rospy.init_node("rudder_and_sail_angle_publisher", anonymous=True)
    rospy.Subscriber("/desired_heading_degrees", heading, desiredHeadingCallBack)
    rospy.Subscriber("/windSensor", windSensor, windSensorCallBack)
    rospy.Subscriber("/GPS", GPS, gpsCallBack)
    rospy.spin()


if __name__ == "__main__":
    main()
