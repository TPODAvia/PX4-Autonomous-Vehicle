# Information: https://px4_air.coex.tech/en/simple_offboard.html#gettelemetry

import rospy
from px4_air import srv

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

# Print drone's state
print(get_telemetry())
