/***************************************************************************************************************************
*
* Author: TPODAvia
* Email: thepowerofdarknes2000@gmail.com
* Time: 2023.03.11
* Description: Control mavros system via cmd_vel topic
*  
***************************************************************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <iostream>
#include <sstream>

// topic header file
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>


using namespace std;
mavros_msgs::State current_state;  // The current state of the vehicle [including the locked state mode] (read from the flight control)

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Callback<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

float linx, angZ;

void VelocityCallback(const geometry_msgs::Twist& msg){
   //Using the callback function just for subscribing  
   //Subscribing the message and converting to RC command in 'linx' and 'angZ'
   linx = 700 - (msg.linear.x)*1000;
   angZ = 1500 - (msg.angular.z)*800;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_mavros_node");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);  
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, &VelocityCallback);  
    ros::Publisher cntrl_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1000);
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    ros::Rate rate(10);

    mavros_msgs::SetMode mode_cmd;
    mavros_msgs::CommandBool arm_cmd;

    ros::Publisher control_pub;
    mavros_msgs::OverrideRCIn control;  


    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCU...");
    }

    if(!current_state.armed)
    {
        arm_cmd.request.value = true;
        arming_client.call(arm_cmd);

        cout << "Arming..." <<endl;

    }else
    {
        cout << "Arm Susscess!!!" <<endl;
    }

    while(ros::ok())
    {

        if(current_state.mode != "OFFBOARD")
        {
            mode_cmd.request.custom_mode = "OFFBOARD";
            set_mode_client.call(mode_cmd);
            cout << "Setting to OFFBOARD Mode..." <<endl;

        }else
        {
            cout << "Set to OFFBOARD Mode Susscess!!!" <<endl;
        }      

        control.channels[0] = angZ;     //Turn wheel command
        control.channels[1] = linx;     //Throttle command
        control.channels[2] = 0;         
        control.channels[3] = 0;      
        control.channels[4] = 0;
        control.channels[5] = 0;
        control.channels[6] = 0;
        control.channels[7] = 0;
        control_pub.publish(control);

    }
    return 0;
}