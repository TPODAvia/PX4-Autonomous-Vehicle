#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    for(int i = 0; i < 100; i++)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
    }
    double radius = 2.0;
    double angle_increment = 0.1;
    double angle = 0.0;

    while(ros::ok())
    {

        pose.pose.position.x = radius * cos(angle);
        pose.pose.position.y = radius * sin(angle);

        local_pos_pub.publish(pose);

        if(-1.57 < angle < 1.57)
        {
            angle += angle_increment;
            if(angle > 2 * M_PI)
            {
                angle = 0.0;
            }
        }
        else
        {
            if(angle < -2 * M_PI)
            {
                angle = 0.0;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
