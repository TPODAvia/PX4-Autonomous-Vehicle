# Information: https://px4_air.coex.tech/aruco
# Enable wind for 10 second:
# rosservice call gazebo/apply_body_wrench '{body_name: "base_link" , wrench: { force: { x: 1, y: 0 , z: 10 } }, start_time: 1, duration: 5000000000 }'

import rospy
from px4_air import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True)
rospy.sleep(5)

print('Fly forward')
navigate(x=10, y=10, z=0, frame_id='body')
rospy.sleep(20)

print('Fly back Home')
navigate(x=0, y=0, z=2, frame_id='map')
rospy.sleep(40)

print('Fly 1 meter above ArUco marker 90')
navigate(x=0, y=0, z=0.4, frame_id='aruco_90')
rospy.sleep(5)

# print('Fly to x=1 y=1 z=1 relative to ArUco markers map')
# navigate(x=1, y=1, z=1, frame_id='aruco_map')
rospy.sleep(5)

print('Perform landing')
land()