#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import PositionTarget

def main():
    rospy.init_node('setpoint_local_publisher', anonymous=True)
    pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Create a PositionTarget message and set the desired position
        target = PositionTarget()
        target.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.position.x = 10.0
        target.position.y = 30.0
        target.position.z = 0.0

        # Publish the message
        pub.publish(target)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass