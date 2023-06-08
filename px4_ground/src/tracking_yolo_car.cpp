#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include "yolov8_ros_msgs/DepthPoints.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
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
            // std::cout << "x: " << x << ", y: " << y << ", depth: " << depth << std::endl;
        }

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle nh;

    ros::Subscriber yolo_sub = nh.subscribe("yolov8/DepthPoints", 10, yolo_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::OverrideRCIn rc_override_msg;

    int throttle_center = 800;
    int steering_pwm_center = 1500;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";

    ros::Time last_request = ros::Time::now();

    rc_override_pub.publish(rc_override_msg);
    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
    }
    rate.sleep();

    int throttle_pwm;
    while (ros::ok()) {
        if (current_state.mode != "MANUAL" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(manual_set_mode) && manual_set_mode.response.mode_sent) {
                ROS_INFO("Manual mode enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        rc_override_msg.channels[0] = steering_pwm_center + static_cast<int>(x);
        rc_override_msg.channels[1] = throttle_center + static_cast<int>(1*(depth - 15));
        
        // std::cout << "x: " << static_cast<int>(x/4) << ", depth: " << static_cast<int>(15 - depth) << std::endl;
        // std::cout << depth << ", " << rc_override_msg.channels[0] << ", " << rc_override_msg.channels[1] << std::endl;

        rc_override_pub.publish(rc_override_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}