/***************************************************************************************************************************
*
rosservice call /mavros/setpoint_velocity/mav_frame         "mav_frame: 1000"
srv file:
uint8 FRAME_GLOBAL=0
uint8 FRAME_LOCAL_NED=1
uint8 FRAME_MISSION=2
uint8 FRAME_GLOBAL_RELATIVE_ALT=3
uint8 FRAME_LOCAL_ENU=4
uint8 FRAME_GLOBAL_INT=5
uint8 FRAME_GLOBAL_RELATIVE_ALT_INT=6
uint8 FRAME_LOCAL_OFFSET_NED=7
uint8 FRAME_BODY_NED=8
uint8 FRAME_BODY_OFFSET_NED=9
uint8 FRAME_GLOBAL_TERRAIN_ALT=10
uint8 FRAME_GLOBAL_TERRAIN_ALT_INT=11
uint8 FRAME_BODY_FRD=12
uint8 FRAME_RESERVED_13=13
uint8 FRAME_RESERVED_14=14
uint8 FRAME_RESERVED_15=15
uint8 FRAME_RESERVED_16=16
uint8 FRAME_RESERVED_17=17
uint8 FRAME_RESERVED_18=18
uint8 FRAME_RESERVED_19=19
uint8 FRAME_LOCAL_FRD=20
uint8 FRAME_LOCAL_FLU=21
uint8 mav_frame

* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2019.12.31
* Description: Autonomous circular trajectory in offboard mode for amov_car
***************************************************************************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <Eigen/Eigen>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
using namespace std;

mavros_msgs::SetMode mode_cmd;
mavros_msgs::SetMavFrame set_mav_frm;
ros::Publisher local_vel_pub;
ros::ServiceClient set_mode_client;
ros::ServiceClient mav_frame_client;
ros::ServiceClient arming_client;
mavros_msgs::CommandBool arm_cmd;

enum Drone
{
  	WAITING,			//Wait for offboard mode
	CHECKING,			//Check the status of the car
	RUN,			    //Run
};

Drone RunState = WAITING; //Initial state WAITING


//Receive the current status of the car from the flight controller
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
	current_state = *msg;
}

//Receive linear and angular speeds from /cmd_vel topic
float linX, angZ;
void VelocityCallback(const geometry_msgs::Twist& msg2){
      linX = msg2.linear.x;
      angZ = msg2.angular.z;
}



// state machine update
void run_state_update(void)
{

	switch(RunState)
	{
		case WAITING:
			if(current_state.mode != "OFFBOARD" || current_state.armed == false)//Wait for offboard mode
			{
				cout << "Offboard mode and Arming should be true "<<endl;
			}
			else
			{
                cout << "CHECKING enabled "<<endl;
                set_mav_frm.request.FRAME_BODY_NED;
                cout << "Set frame id to BODY_NED: " << set_mav_frm.request.FRAME_BODY_NED <<endl;
                mav_frame_client.call(set_mav_frm);
				RunState = CHECKING;
			}
			cout << "WAITING" <<endl;
			break;

		case CHECKING:
			if(0 == 0) 			//If there is no position information, perform locking
			{
				cout << "Check error, make sure have local location" <<endl;
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
				RunState = WAITING;	
			}
			else
			{
				RunState = RUN;
			}
			cout << "CHECKING" <<endl;
			break;

        case RUN:
        {
            geometry_msgs::TwistStamped cmd_vel;
            cmd_vel.twist.linear.x = linX;
            cmd_vel.twist.angular.z = angZ;
            local_vel_pub.publish(cmd_vel);
            
            cout << "RUN" <<endl;
            cout << "The linear speed is: "<< cmd_vel.twist.linear.x <<endl;
            cout << "The angular speed is: "<< cmd_vel.twist.angular.z <<endl;
            break;
        }
	}

}				


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_mavros_node");
    ros::NodeHandle nh("~");
    //  frequency [20Hz]
    ros::Rate rate(20.0);

    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 1000, &VelocityCallback); 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    // [Service] Modify the system mode
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mav_frame_client = nh.serviceClient<mavros_msgs::SetMavFrame>("/mavros/setpoint_velocity/mav_frame");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    arm_cmd.request.value = true;
    while(ros::ok())
    {
			run_state_update();
	 		ros::spinOnce();
      rate.sleep();
    }

    return 0;

}

