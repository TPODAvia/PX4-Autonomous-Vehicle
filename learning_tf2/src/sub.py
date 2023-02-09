#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from math import sin, cos, pi

rospy.init_node('odom_pub')
odom_pub = rospy.Publisher("/my_odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
rospy.wait_for_service("/gazebo/get_model_state")
get_model_srv= rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)

odom = Odometry()
header = Header()
header.frame_id = '/odom'

model = GetModelStateRequest()
model.model_name = "r1_rover"

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()
r = rospy.Rate(2)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    result = get_model_srv(model)
    odom.pose.pose = result.pose
    odom.child_frame_id = "base_link"
    odom.twist.twist = result.twist
    
    header.stamp = rospy.Time.now()
    odom.header = header
    odom_pub.publish (odom)
    last_time = current_time
    r.sleep()