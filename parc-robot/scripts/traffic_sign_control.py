#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time
import random


class sign_control():

    def __init__(self):
        # Initialize node
        rospy.init_node('sign_control')

        # Get delay from parameter server
        start_delay = rospy.get_param('~start_delay', 10)

        # Initiate publisher
        self._joint_red_pub = rospy.Publisher('/traffic_sign/joint_red_position_controller/command',
                                              Float64, queue_size=1)

        self._joint_green_pub = rospy.Publisher('/traffic_sign/joint_green_position_controller/command',
                                                Float64, queue_size=1)

        # wait for controllers to come up
        while self._joint_red_pub.get_num_connections() == 0 or \
                self._joint_green_pub.get_num_connections() == 0:
            rospy.sleep(rospy.Duration(0.2))

        rospy.loginfo("Sending Red sign")

        # initial state is stop
        self.send_red()

        # Wait for "start_delay" seconds before turning the sign green
        rospy.sleep(rospy.Duration(start_delay))

        rospy.loginfo("Sending Green sign")

        self.send_green()
        # wait onf green for exactly 15 seconds
        rospy.sleep(rospy.Duration(15))

        rospy.loginfo("Sending Red sign")

        # Return to Red and exit
        self.send_red()

    def send_red(self):
        j1 = Float64(0)
        j2 = Float64(1.57)

        self._joint_red_pub.publish(j1)
        self._joint_green_pub.publish(j2)

    def send_green(self):
        j1 = Float64(1.57)
        j2 = Float64(0)

        self._joint_red_pub.publish(j1)
        self._joint_green_pub.publish(j2)


if __name__ == "__main__":
    try:
        sign_control()
    except rospy.ROSInterruptException:
        pass
