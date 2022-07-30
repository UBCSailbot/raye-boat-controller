#!/usr/bin/env python
from heading_controller import HeadingController
from controller_output_refiner import ControllerOutputRefiner
import sailbot_constants
import rospy
import threading
from rospy import Timer, Duration
from datetime import datetime
from sailbot_msg.msg import actuation_angle, heading, windSensor, GPS, min_voltage
from std_msgs.msg import Bool
from control_modes import CONTROL_MODES


# Define global variables for the needed topics
headingSetPointRad = None
headingMeasureRad = None
apparentWindAngleRad = None
groundspeedKnots = None
rudderAngleRad = 0
sailWinchPosition = 0
jibWinchPosition = 0
lowVoltage = False
lowWind = False
lock = threading.Lock()

# Global variables for configuring controller (should be command line arguments ideally)
DISABLE_LOW_POWER = False

"""
No Fixed State: None
Jibe: 1
Tack: 2
Low Power: 3
"""
FIXED_CTRL_STATE = None
WINCH_QUANTIZATION_PARAMETER = 4

# Boat control logic
controller = HeadingController(
    boat_speed=0,
    disableLowPower=DISABLE_LOW_POWER,
    fixedControlMode=FIXED_CTRL_STATE,
    winchQuantParam=WINCH_QUANTIZATION_PARAMETER
)

# Control node publisher
rudder_winch_actuation_angle_pub = rospy.Publisher(
    "/rudder_winch_actuation_angle", actuation_angle, queue_size=1
)


def windSensorCallBack(windsensor_msg_instance):
    global apparentWindAngleRad
    apparentWindAngleDegrees = windsensor_msg_instance.measuredBearingDegrees
    apparentWindAngleRad = apparentWindAngleDegrees * sailbot_constants.DEGREES_TO_RADIANS


def gpsCallBack(gps_msg_instance):
    global headingMeasureRad, groundspeedKnots

    headingMeasureDegrees = gps_msg_instance.bearingDegrees
    headingMeasureRad = headingMeasureDegrees * sailbot_constants.DEGREES_TO_RADIANS

    groundspeedKMPH = gps_msg_instance.speedKmph
    groundspeedKnots = groundspeedKMPH * sailbot_constants.KMPH_TO_KNOTS


def desiredHeadingCallBack(heading_msg_instance):
    global headingSetPointRad
    headingSetPointDeg = heading_msg_instance.headingDegrees
    headingSetPointRad = headingSetPointDeg * sailbot_constants.DEGREES_TO_RADIANS


def minVoltageCallBack(min_voltage_msg_instance):
    global lowVoltage
    min_voltage_level = min_voltage_msg_instance.min_voltage
    lowVoltage = (min_voltage_level < sailbot_constants.MIN_VOLTAGE_THRESHOLD)


def lowWindCallBack(low_wind_msg_instance):
    global lowWind
    lowWind = low_wind_msg_instance


def updateRudders(event):
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

        lock.acquire()
        controller.switchControlMode(
            heading_error=heading_error,
            boat_speed=groundspeedKnots,
            low_battery_level=lowVoltage,
            low_wind=lowWind
        )
        lock.release()

        rudderAngleRad = (
            controller.get_feed_back_gain(heading_error) * heading_error
        )


def updateWinches(event):
    if apparentWindAngleRad is not None:
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


def publishRudderWinchAngle(event):
    if (
        rudderAngleRad is not None
        and sailWinchPosition is not None
        and jibWinchPosition is not None
    ):

        rospy.loginfo(
            "\n" +
            "Published on {}\n".format(datetime.fromtimestamp(int(event.current_real.to_sec()))) +
            "\n" +
            "SENSOR READINGS\n" +
            "\tCurrent Heading: {} radians\n".format(headingMeasureRad) +
            "\tDesired Heading: {} radians\n".format(headingSetPointRad) +
            "\tWind Angle: {} radians\n".format(apparentWindAngleRad) +
            "\tGround Speed: {} knots\n".format(groundspeedKnots) +
            "\tLow Wind: {}\n".format(lowWind) +
            "\tLow Voltage: {}\n".format(lowVoltage) +
            "\n" +
            "CONTROLLER STATE\n" +
            "\tControl Mode: {}\n".format(CONTROL_MODES[controller.getControlModeID()]) +
            "\tFixed Control State: {}\n".format(controller.controlModeIsFixed) +
            "\tLow Power: {}\n".format(lowVoltage or lowWind) +
            "\tLow Power Disabled: {}\n".format(controller.lowPowerDisabled) +
            "\n" +
            "PUBLISHED VALUES\n" +
            "\tRudder Angle: {} radians\n".format(rudderAngleRad) +
            "\tSail Winch Position: {}\n".format(sailWinchPosition) +
            "\tJib Winch Position: {}\n".format(jibWinchPosition) +
            "\n"
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

    # Callbacks to update rudders and winches
    Timer(Duration(sailbot_constants.RUDDER_UPDATE_PERIOD), updateRudders)
    Timer(Duration(sailbot_constants.WINCH_UPDATE_PERIOD), updateWinches)

    # Callback to publish
    Timer(Duration(sailbot_constants.PUBLISHING_PERIOD), publishRudderWinchAngle)

    if (FIXED_CTRL_STATE is not None) and (DISABLE_LOW_POWER):
        rospy.logerr("Cannot disable low power and fix the control state at the same time!")
        rospy.signal_shutdown("Low power mode is disabled and the control state is fixed.")
    else:
        if DISABLE_LOW_POWER:
            rospy.logwarn("Low power mode is DISABLED! If you don't want this, change DISABLE_LOW_POWER to False.\n")
        elif FIXED_CTRL_STATE is not None:
            rospy.logwarn(
                "The controller is fixed to the {} state! ".format(CONTROL_MODES[FIXED_CTRL_STATE]) +
                "If you don't want this, change FIXED_CTRL_STATE to None."
            )
        rospy.loginfo("Boat controller started successfully. Waiting on sensor data...\n")
        rospy.spin()


if __name__ == "__main__":
    main()
