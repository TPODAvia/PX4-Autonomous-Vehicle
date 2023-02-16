# Information:
# Rostopics /cmd_vel
# Run teleopt keyboard first:
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# then run this python program:
# python3 keyboard_control.py

import sys
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from clover import srv
from std_srvs.srv import Trigger


def range_callback(msg):
    # Process data from the rangefinder
    print('Rangefinder distance:', msg.range)
    if msg.range <  0.5:
        print('Land')
        land()
        rospy.sleep(5)
        rospy.signal_shutdown("Landed")   

def callback(data):
    # msgs type: Twist
    # data.linear.x data.linear.y data.linear.z
    # data.angular.x data.angular.y data.angular.z
    #Hold Shift to use
    navigate(x=data.linear.x, y=data.linear.y, z= data.linear.z, yaw=math.radians(10*data.angular.z),frame_id='body', auto_arm=True)

rospy.init_node('Keyboard_controller')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
release = rospy.ServiceProxy('simple_offboard/release', Trigger)


print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
# Wait for 5 seconds
rospy.sleep(5)

# Subscribe to keyboard command
rospy.Subscriber('/cmd_vel', Twist, callback)
# Subscribe to laser rangefinder data
rospy.Subscriber('rangefinder/range', Range, range_callback)
rospy.spin()