#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Altitude.h>

#include "yolov8_ros_msgs/DepthPoints.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

float desired_alt = 3.0;
float desired_alt_vel = 0.0;
void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg)
{
    // ROS_INFO("Altitude: mon: %f, amon: %f, rel: %f, terr: %f, gnd: %f",
    //          msg->monotonic, msg->amsl, msg->local, msg->relative, msg->terrain);
    desired_alt_vel = desired_alt - msg->local;

    // std::cout << "desired_alt_vel: " << desired_alt_vel  << std::endl;
}

float x;
float y;
float depth;
void yolo_cb(const yolov8_ros_msgs::DepthPoints& msg)
{   
    // Access the fields of the custom message
    // int num_points = msg.depth_point;
    std::vector<yolov8_ros_msgs::DepthPoint> depth_points = msg.depth_point;

    // Process the depth points as needed
    for (const auto& point : depth_points) {

        if (point.Class == "sports ball")
        {
            x = point.offset_center_x;
            y = point.offset_center_y;
            depth = point.depth;
            if (depth > 17)
            {
                x = 0;
                y = 0;
                depth = 0;
            }

            // std::cout << "x: " << x << ", y: " << y << ", depth: " << depth << std::endl;
            return;
        }
        else
        {
            x = 0;
            y = 0;
            depth = 0;
        }

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber altitude_sub = nh.subscribe("/mavros/altitude", 10, altitudeCallback);
    ros::Subscriber yolo_sub = nh.subscribe("yolov8/DepthPoints", 10, yolo_cb);
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

    // Send a few setpoints before starting
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
    position_target.position.z = 5.0; // Somehow this didn't even work
    position_target.velocity.x = 0.0;
    position_target.velocity.y = 0.0;
    position_target.velocity.z = desired_alt_vel;
    position_target.yaw_rate = 0.3;
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
        { 
            trigger = true; 
        }

        if( current_state.mode != "OFFBOARD" && trigger &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 

        if (depth == 0)
        {
            position_target.velocity.x = 0.0;
        }
        else if (depth > 6) // Fly forward
        {
            position_target.velocity.x = depth/100 -  0.15;
        }
        else // Fly backward with a scalled velocity
        {
            position_target.velocity.x = depth/1000 - 0.015;
        }  
        position_target.velocity.z = desired_alt_vel;
        position_target.yaw_rate = 0 - x/10000;

        // std::cout << position_target << std::endl;
        // std::cout << "x: " << x << ", y: " << y << ", depth: " << depth << std::endl;
        setpoint_raw_local_pub.publish(position_target);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}