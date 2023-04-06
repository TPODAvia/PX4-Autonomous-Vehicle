#!/usr/bin/env python3

# Enable wind for 10 second manually:
# rosservice call gazebo/apply_body_wrench '{body_name: "base_link" , wrench: { force: { x: 1, y: 0 , z: 10 } }, start_time: 1, duration: 5000000000 }'
# https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_control/uuv_control_utils/scripts/apply_body_wrench.py

from __future__ import print_function
import rospy
import sys
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3
import random
import time

if __name__ == '__main__':
    #print('Apply programmed perturbation to vehicle', rospy.get_namespace())
    rospy.init_node('wind_node', anonymous=False)

    vehicle_name = rospy.get_param("~vehicle_name", "frame450" )

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)
    try:
        rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
    except rospy.ROSException:
        print('Service not available! Closing node...')
        sys.exit(-1)
    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    except rospy.ServiceException as e:
        print('Service call failed, error=', e)
        sys.exit(-1)

    ns = rospy.get_namespace().replace('/', '')


    body_name = '%s{}::base_link'.format(vehicle_name) % ns

    print("The force is appied to the link: ")
    print(body_name)

    starting_time = 0.0
    print('Starting time= {} s'.format(starting_time))
    duration = 1.0
    print('Duration [s]=', ('Inf.' if duration < 0 else duration))
    if starting_time >= 0:
        rate = rospy.Rate(100)
        while rospy.get_time() < starting_time:
            rate.sleep()

    #wind vector scale
    fx=0
    fy=0
    fz=0
    tx=0
    ty=0
    tz=0
    # applying dirturbance scalling
    n = 0.1
    while 1:
        force = [fx+random.randint(-10,10)*n, fy+random.randint(-10,10)*n, fz+random.randint(-10,10)*n]
        torque = [tx+random.randint(-10,10)*n, ty+random.randint(-10,10)*n, tz+random.randint(-10,10)*n]
        print('Force [N]=', force)
        print('Torque [N]=', torque)
        wrench = Wrench()
        wrench.force = Vector3(*force)
        wrench.torque = Vector3(*torque)
        success = apply_wrench(
            body_name,
            'world',
            Point(0, 0, 0),
            wrench,
            rospy.Time().now(),
            rospy.Duration(duration))
        time.sleep(1)

    # Uncomment for debug
    # if success:
    #     print('Body wrench perturbation applied!')
    #     print('\tFrame: ', body_name)
    #     print('\tDuration [s]: ', duration)
    #     print('\tForce [N]: ', force)
    #     print('\tTorque [Nm]: ', torque)
    # else:
    #     print('Failed!')