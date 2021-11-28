#!/usr/bin/env python

import rospy
from sailbot_msg.msg import heading, Sensors
ROSPY_RATE = 10  # hertz


def talker():
    heading_degrees_pub = rospy.Publisher(
        "/desired_heading_degrees", heading, queue_size=10
    )
    sensors_pub = rospy.Publisher("/sensors", Sensors, queue_size=10)

    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(ROSPY_RATE)
    while not rospy.is_shutdown():
        heading_degrees_pub.publish(heading(5))
        sensors_pub.publish(Sensors())
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
