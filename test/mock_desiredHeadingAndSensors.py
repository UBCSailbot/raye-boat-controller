#!/usr/bin/env python

import rospy
import random
from sailbot_msg.msg import heading, windSensor, GPS, min_voltage
ROSPY_RATE = 10  # hertz


def talker():
    heading_degrees_pub = rospy.Publisher(
        "/desired_heading_degrees", heading, queue_size=10
    )
    windSensor_pub = rospy.Publisher("/windSensor", windSensor, queue_size=10)
    gps_pub = rospy.Publisher("/GPS", GPS, queue_size=10)
    min_voltage_pub = rospy.Publisher("/min_voltage", min_voltage, queue_size=10)

    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(ROSPY_RATE)

    while not rospy.is_shutdown():
        current_heading = random.uniform(0.0, 45.0)
        desired_heading = random.uniform(0.0, 45.0)
        groundspeed = random.uniform(0.0, 2.0)
        windangle = int(random.uniform(0.0, 180.0))
        voltage = random.uniform(0.0, 2.0)

        heading_degrees_pub.publish(heading(desired_heading))
        windSensor_pub.publish(windSensor(windangle, 1., False))
        gps_pub.publish(GPS(.0, .0, current_heading, groundspeed))
        min_voltage_pub.publish(min_voltage(voltage))

        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
