/***************************************************************************************************************************
*
* Author: TPODAvia
* Time: 2023.05.10
* Description: Implement px4 quadrotor YOLO tracking
***************************************************************************************************************************/
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


geometry_msgs::PoseStamped setpoint_pub;
geometry_msgs::TransformStamped transform_stamped;
geometry_msgs::PoseStamped pose_stamped;
geometry_msgs::TwistStamped local_vel_pub;
geometry_msgs::TwistStamped vel;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void tf_callback(const tf2_ros::Buffer &buffer, ros::Publisher &setpoint_pub, ros::Publisher &local_vel_pub)
{

    // pose_stamped.pose.position.z = 5;
    vel.twist.linear.x = 5;
    // vel.twist.linear.y = 0;
    // vel.twist.linear.z = 0;
    // vel.twist.angular.x = 0;
    // vel.twist.angular.y = 0;
    vel.twist.angular.z = 5;
    local_vel_pub.publish(vel);
    // setpoint_pub.publish(pose_stamped);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_yolo_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    if(current_state.mode != "AUTO.TAKEOFF")
    {
        offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
        set_mode_client.call(offb_set_mode);
        std::cout << "Setting to TAKEOFF Mode..." <<std::endl;

    }

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

    offb_set_mode.request.custom_mode = "OFFBOARD";

    bool trigger = false;

    while (nh.ok())
    {

        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                trigger = true;
            }
            last_request = ros::Time::now();
        }


        if( current_state.mode != "OFFBOARD" && trigger &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 


        tf_callback(buffer, setpoint_pub, local_vel_pub);


        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}