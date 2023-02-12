#!/usr/bin/env python

import sys
import rospy
import std_msgs
from gazebo_msgs.msg import ApplyBodyWrench

def apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, \
        start_time, duration):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        apply_body_wrench(body_name, reference_frame, reference_point, wrench, \
        start_time, duration)
    #except rospy.ServiceException, e:
    except rospy.ServiceException:
        #print ("Service call failed: %s"%e)
        print ("Service call failed")

if __name__ == "__main__":
    body_name = 'panda_link7'
    reference_frame = 'panda_link7'
    reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)