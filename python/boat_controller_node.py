#!/usr/bin/env python
from heading_controller import HeadingController
from controller_output_refiner import ControllerOutputRefiner
import sailbot_constants
import rospy
import threading
from sailbot_msg.msg import actuation_angle, heading, windSensor, GPS, min_voltage
from std_msgs.msg import Bool

lock = threading.Lock()
# define global variables for the needed topics
headingSetPointRad = None
headingMeasureRad = None
apparentWindAngleRad = None
groundspeedKnots = None
rudderAngleRad = 0
sailWinchPosition = 0
jibWinchPosition = 0
lowVoltage = False
lowWind = False

controller = HeadingController(boat_speed=0)

rudder_winch_actuation_angle_pub = rospy.Publisher(
    "/rudder_winch_actuation_angle", actuation_angle, queue_size=1
)


def windSensorCallBack(windsensor_msg_instance):
    lock.acquire()

    global apparentWindAngleRad

    apparentWindAngleDegrees = windsensor_msg_instance.measuredBearingDegrees
    apparentWindAngleRad = apparentWindAngleDegrees * sailbot_constants.DEGREES_TO_RADIANS

    publishRudderWinchAngle()
    lock.release()


def gpsCallBack(gps_msg_instance):
    lock.acquire()

    global headingMeasureRad, groundspeedKnots

    headingMeasureDegrees = gps_msg_instance.bearingDegrees
    headingMeasureRad = headingMeasureDegrees * sailbot_constants.DEGREES_TO_RADIANS

    groundspeedKMPH = gps_msg_instance.speedKmph
    groundspeedKnots = groundspeedKMPH * sailbot_constants.KMPH_TO_KNOTS

    publishRudderWinchAngle()
    lock.release()


def desiredHeadingCallBack(heading_msg_instance):
    lock.acquire()

    global headingSetPointRad

    headingSetPointDeg = heading_msg_instance.headingDegrees
    headingSetPointRad = headingSetPointDeg * sailbot_constants.DEGREES_TO_RADIANS

    publishRudderWinchAngle()
    lock.release()


def minVoltageCallBack(min_voltage_msg_instance):
    lock.acquire()

    global lowVoltage

    min_voltage_level = min_voltage_msg_instance.min_voltage
    lowVoltage = (min_voltage_level < sailbot_constants.MIN_VOLTAGE_THRESHOLD)

    publishRudderWinchAngle()
    lock.release()


def lowWindCallBack(low_wind_msg_instance):
    lock.acquire()

    global lowWind
    lowWind = low_wind_msg_instance

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
            boat_speed=groundspeedKnots,
            low_battery_level=lowVoltage,
            low_wind=lowWind
        )

        rudderAngleRad = (
            controller.get_feed_back_gain(heading_error) * heading_error
        )

        global sailWinchPosition
        sailWinchPosition = controller.get_sail_winch_position(
            apparentWindAngleRad,
            sailbot_constants.X1_SAIL,
            sailbot_constants.X2_SAIL
        )

        global jibWinchPosition
        jibWinchPosition = controller.get_jib_winch_position(
            apparentWindAngleRad,
            sailbot_constants.X1_JIB,
            sailbot_constants.X2_JIB
        )

        rudder_winch_actuation_angle_pub.publish(
            ControllerOutputRefiner.saturate(
                rudderAngleRad,
                sailbot_constants.MAX_ABS_RUDDER_ANGLE_RAD,
                -sailbot_constants.MAX_ABS_RUDDER_ANGLE_RAD),
            ControllerOutputRefiner.saturate(
                sailWinchPosition,
                sailbot_constants.MAX_WINCH_POSITION,
                sailbot_constants.MIN_WINCH_POSITION),
            ControllerOutputRefiner.saturate(
                jibWinchPosition,
                sailbot_constants.MAX_WINCH_POSITION,
                sailbot_constants.MIN_WINCH_POSITION)
        )


def main():
    rospy.init_node("rudder_and_sail_angle_publisher", anonymous=True)
    rospy.Subscriber("/desired_heading_degrees", heading, desiredHeadingCallBack)
    rospy.Subscriber("/windSensor", windSensor, windSensorCallBack)
    rospy.Subscriber("/GPS", GPS, gpsCallBack)
    rospy.Subscriber("/min_voltage", min_voltage, minVoltageCallBack)
    rospy.Subscriber('/lowWindConditions', Bool, lowWindCallBack)
    rospy.spin()


if __name__ == "__main__":
    main()
