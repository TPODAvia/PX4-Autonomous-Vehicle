#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Altitude.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

float desired_alt = 5.0;
float desired_alt_vel = 0.0;
void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg)
{
    ROS_INFO("Altitude: mon: %f, amon: %f, rel: %f, terr: %f, gnd: %f",
             msg->monotonic, msg->amsl, msg->local, msg->relative, msg->terrain);
    desired_alt_vel = desired_alt - msg->local;

    std::cout << "desired_alt_vel: " << desired_alt_vel  << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber altitude_sub = nh.subscribe("/mavros/altitude", 10, altitudeCallback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    if(current_state.mode != "AUTO.TAKEOFF")
    {
        offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
        set_mode_client.call(offb_set_mode);
        std::cout << "Setting to TAKEOFF Mode..." <<std::endl;

    }


    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget position_target;
    position_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    position_target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                //mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW;
    position_target.position.z = 5.0; // Somehow this didnt even work
    position_target.velocity.x = 0.0;
    position_target.velocity.y = 0.0;
    position_target.velocity.z = desired_alt_vel;
    position_target.yaw_rate = 0.3; // Example yaw rate value
    position_target.header.stamp = ros::Time::now();

    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool trigger = false;
    
    while (ros::ok()) {

        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                trigger = true;
            }
            last_request = ros::Time::now();
        }
        else
        { trigger = true; }

        if( current_state.mode != "OFFBOARD" && trigger &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 

        position_target.velocity.x = 1.0;
        position_target.velocity.z = 0.8*desired_alt_vel;
        position_target.yaw_rate = 0.1;
        setpoint_raw_local_pub.publish(position_target);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}