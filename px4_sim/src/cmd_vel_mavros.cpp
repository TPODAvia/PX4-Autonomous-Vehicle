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

float linx = 0, angZ = 0;

void VelocityCallback(const geometry_msgs::Twist& msg2){
   //Using the callback function just for subscribing  
   //Subscribing the message and converting to RC command in 'linx' and 'angZ'
//    float check = msg2.linear.x;

//    if (check < 0.0)
//    {
//         linx = 10;
//    }
//    else if (check == 0.0)
//    {
//         linx = 0;
//    }
//    else if (check > 0.0)
//    {
//         linx = 300 + (msg2.linear.x)*4500;
//    }

//    angZ = 1500 + (msg2.angular.z)*100;
      linx = msg2.linear.x;
      angZ = msg2.angular.z;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_mavros_node");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);  
    ros::Subscriber cmd_sub = n.subscribe("/cmd_vel", 1000, &VelocityCallback); 
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("setpoint_velocity/cmd_vel_unstamped", 1000);
    ros::Publisher cntrl_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1000);
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    ros::Rate rate(8);
    ros::Rate rate2(0.05);

    mavros_msgs::SetMode mode_cmd;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::SetMode offb_set_mode;



    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCU...");
    }

    //send a few setpoints before starting
    geometry_msgs::Twist precontrol;
    for(int i = 100; ros::ok() && i > 0; --i){
        precontrol.linear.x = 0; 
        precontrol.angular.z = 0;
        cmd_pub.publish(precontrol);
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "OFFBOARD";

    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {

        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else 
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        } 
        
        // mavros_msgs::OverrideRCIn control;  
        // control.channels[0] = angZ;     //Turn wheel command
        // control.channels[1] = linx;     //Throttle command
        // control.channels[2] = 0;         
        // control.channels[3] = 0;      
        // control.channels[4] = 0;
        // control.channels[5] = 0;
        // control.channels[6] = 0;
        // control.channels[7] = 0;
        // cntrl_pub.publish(control);
        geometry_msgs::Twist control;
        control.linear.x = linx; 
        control.angular.z = angZ;
        cmd_pub.publish(control);
        rate.sleep();
        cout << "Parameters!!!" <<endl;
        cout << angZ << " and " << linx << endl;
        // angZ = 1500;
        // linx = 0;
        ros::spinOnce();
    }
    return 0;
}