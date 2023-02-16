#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

g_vel = Twist()

def state_cb(msg):
    global current_state
    current_state = msg

def state_callback(data):
    global g_vel
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.angular.x)
    g_vel = data

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    vel = TwistStamped()

    vel.twist.linear.x = 0
    vel.twist.linear.y = 0
    vel.twist.linear.z = 0
    vel.twist.angular.z = 0
    # pose.pose.position.x = 2
    # pose.pose.position.y = 2
    # pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(vel)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        rospy.Subscriber('cmd_vel', Twist, callback = state_callback)
        vel.twist.linear.x = g_vel.linear.x
        vel.twist.linear.y = g_vel.linear.y
        vel.twist.linear.z = g_vel.linear.z
        vel.twist.angular.z = g_vel.angular.z

        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        # local_pos_pub.publish(pose)
        local_vel_pub.publish(vel)

        rate.sleep()
