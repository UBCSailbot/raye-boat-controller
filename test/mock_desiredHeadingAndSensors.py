#!/usr/bin/env python

import rospy
import random
from sailbot_msg.msg import heading, Sensors
ROSPY_RATE = 10  # hertz


def talker():
    heading_degrees_pub = rospy.Publisher(
        "/desired_heading_degrees", heading, queue_size=10
    )
    sensors_pub = rospy.Publisher("/sensors", Sensors, queue_size=10)

    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(ROSPY_RATE)
    timestamp = "0"
    while not rospy.is_shutdown():
        current_heading = random.uniform(0.0, 45.0)
        desired_heading = random.uniform(0.0, 45.0)
        groundspeed = random.uniform(0.0, 2.0)
        windangle = int(random.uniform(0.0, 180.0))
        timestamp = str(int(timestamp) + 1)
        heading_degrees_pub.publish(heading(desired_heading))
        sensors_pub.publish(Sensors(
            0,
            0.0,
            windangle,
            0.0,
            0,
            0.0,
            0,
            timestamp,
            0.0,
            0.0,
            groundspeed,
            0.0,
            current_heading,
            0.0,
            True,
            "0",
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            True,
            0,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0)
        )
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
