/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 * 
 * Body frame seems bugged
 * 
 * COM_RCL_EXCEPT 4
 * rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8"
 * rostopic pub -r 10 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.01
angular:
  x: 0.0
  y: 0.0
  z: 0.1"
 * 
 * srv file:
 * uint8 FRAME_GLOBAL=0
 * uint8 FRAME_LOCAL_NED=1
 * uint8 FRAME_MISSION=2
 * uint8 FRAME_GLOBAL_RELATIVE_ALT=3
 * uint8 FRAME_LOCAL_ENU=4
 * uint8 FRAME_GLOBAL_INT=5
 * uint8 FRAME_GLOBAL_RELATIVE_ALT_INT=6
 * uint8 FRAME_LOCAL_OFFSET_NED=7
 * uint8 FRAME_BODY_NED=8
 * uint8 FRAME_BODY_OFFSET_NED=9
 * uint8 FRAME_GLOBAL_TERRAIN_ALT=10
 * uint8 FRAME_GLOBAL_TERRAIN_ALT_INT=11
 * uint8 FRAME_BODY_FRD=12
 * uint8 FRAME_RESERVED_13=13
 * uint8 FRAME_RESERVED_14=14
 * uint8 FRAME_RESERVED_15=15
 * uint8 FRAME_RESERVED_16=16
 * uint8 FRAME_RESERVED_17=17
 * uint8 FRAME_RESERVED_18=18
 * uint8 FRAME_RESERVED_19=19
 * uint8 FRAME_LOCAL_FRD=20
 * uint8 FRAME_LOCAL_FLU=21
 * uint8 mav_frame

 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//Receive linear and angular speeds from /cmd_vel topic
float linX, angZ;
void VelocityCallback(const geometry_msgs::Twist& msg2){
      linX = msg2.linear.x;
      angZ = msg2.angular.z;
}

mavros_msgs::SetMavFrame set_mav_frm;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 1000, &VelocityCallback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    ros::ServiceClient mav_frame_client = nh.serviceClient<mavros_msgs::SetMavFrame>("/mavros/setpoint_velocity/mav_frame");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = NAN;
    pose.pose.position.y = NAN;
    pose.pose.position.z = NAN;

    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0;
    vel.twist.angular.x = 0;
    vel.twist.angular.y = 0;
    vel.twist.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    // set_mav_frm.request.FRAME_BODY_NED;
    // std::cout << "Set frame id to BODY_NED: " << set_mav_frm.request.FRAME_BODY_NED <<std::endl;
    // mav_frame_client.call(set_mav_frm);

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // local_pos_pub.publish(pose);
        vel.header.stamp.sec = ros::Time::now().toSec();
        vel.header.stamp.nsec = ros::Time::now().toNSec();
        vel.header.frame_id = "target_position";
        vel.twist.linear.x = 1;
        vel.twist.linear.y = 0;
        vel.twist.linear.z = 0.01;
        // vel.twist.angular.x = 0;
        // vel.twist.angular.y = 0;
        vel.twist.angular.z = 0.1;
        std::cout << "linX is: " << linX << std::endl;
        std::cout << "angZ is: " << angZ << std::endl;
        local_vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}