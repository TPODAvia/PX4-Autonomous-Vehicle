/***************************************************************************************************************************
*
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
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
Eigen::Vector3d pos_target;//In offboard mode, the expected value sent to the flight controller
float desire_Radius = 15;// Expected circle trajectory radius
float MoveTimeCnt = 0;
float priod = 2000.0;   //Adjust this value to change the speed of walking in a circle
Eigen::Vector3d temp_pos_drone;
Eigen::Vector3d temp_pos_target;
mavros_msgs::SetMode mode_cmd;
ros::Publisher setpoint_raw_local_pub;
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
mavros_msgs::CommandBool arm_cmd;

enum Drone
{
  	WAITING,			//Wait for offboard mode
	CHECKING,			// check the status of the car
	RUN,			    //Run in a circle
	RUNOVER,			//Finish		
};

Drone RunState = WAITING; //Initial state WAITING

//Receive the current position of the car from the flight controller
Eigen::Vector3d pos_drone;                     
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone = pos_drone_fcu_enu;
}

//Receive the current status of the car from the flight controller
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
	current_state = *msg;
}

//Send the position expectation to the flight controller (input: expected xyz, expected yaw)
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

// state machine update
void run_state_update(void)
{

	switch(RunState)
	{
		case WAITING:
			if(current_state.mode != "OFFBOARD" || current_state.armed == false)//Wait for offboard mode
			{
				pos_target[0] = pos_drone[0];
				pos_target[1] = pos_drone[1];
				pos_target[2] = pos_drone[2];
				temp_pos_drone[0] = pos_drone[0];
				temp_pos_drone[1] = pos_drone[1];
				temp_pos_drone[2] = pos_drone[2];
				send_pos_setpoint(pos_target, 0);
				cout << "current_x: "<< pos_target[0]<<endl;
				cout << "current_y: "<< pos_target[1]<<endl;
			}
			else
			{
				pos_target[0] = temp_pos_drone[0];
				pos_target[1] = temp_pos_drone[1];
				pos_target[2] = temp_pos_drone[2];
				send_pos_setpoint(pos_target, 0);
				RunState = CHECKING;
			}
			cout << "WAITING" <<endl;
			break;
		case CHECKING:
			if(pos_drone[0] == 0 && pos_drone[1] == 0) 			//If there is no position information, perform locking
			{
				cout << "Check error, make sure have local location" <<endl;
        arm_cmd.request.value = false;
        arming_client.call(arm_cmd);
				RunState = WAITING;	
			}
			else
			{
				RunState = RUN;
				MoveTimeCnt = 0;
			}
			cout << "CHECKING" <<endl;
			break;
			case RUN:
			{
				MoveTimeCnt++;
				if(MoveTimeCnt <= 1)
				{
					pos_target[0] = temp_pos_drone[0] + 10;
					pos_target[1] = temp_pos_drone[1];
					cout << "step1" <<endl;
				}
				else if(pos_drone[0] <= (temp_pos_drone[0]+10+0.5) && pos_drone[0] >= (temp_pos_drone[0]+10-0.5) && pos_drone[1] <= (temp_pos_drone[1]+0.5) && pos_drone[1] >= (temp_pos_drone[1]-0.5) )
				{
					pos_target[0] = temp_pos_drone[0] + 10;
					pos_target[1] = temp_pos_drone[1] + 10;
					cout << "step2" <<endl;
				}
				else if(pos_drone[0] <= (temp_pos_drone[0]+10+0.5) && pos_drone[0] >= (temp_pos_drone[0]+10-0.5) && pos_drone[1] <= (temp_pos_drone[1]+10+0.5) && pos_drone[1] >= (temp_pos_drone[1]+10-0.5) )
				{
					pos_target[0] = temp_pos_drone[0];
					pos_target[1] = temp_pos_drone[1] + 10;
					cout << "step3" <<endl;
				}
				else if(pos_drone[0] <= (temp_pos_drone[0]+0.5) && pos_drone[0] >= (temp_pos_drone[0]-0.5) && pos_drone[1] <= (temp_pos_drone[1]+10+0.5) && pos_drone[1] >= (temp_pos_drone[1]+10-0.5) )
				{
					pos_target[0] = temp_pos_drone[0];
					pos_target[1] = temp_pos_drone[1];
					cout << "step4" <<endl;
				}

				pos_target[2] = 0;
				cout << "desire_x"<< pos_target[0]<<endl;
				cout << "desire_y"<< pos_target[1]<<endl;
				send_pos_setpoint(pos_target, 0);
				if(current_state.mode != "OFFBOARD" || current_state.armed == false)			//如果在中途中切换到onboard，则跳到WAITING
				{
					RunState = WAITING;
				}
			}
			cout << "RUN" <<endl;
			break;
		case RUNOVER:
			{
		    	arm_cmd.request.value = false;
		    	arming_client.call(arm_cmd);
				RunState = WAITING;
			}
			cout << "RUNOVER" <<endl;
			break;

		default:
			cout << "error" <<endl;
	}

}				


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_car_offboard");
    ros::NodeHandle nh("~");
    //  frequency [20Hz]
    ros::Rate rate(20.0);

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // [Service] Modify the system mode
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

   	nh.param<float>("desire_Radius", desire_Radius, 1.0);
    arm_cmd.request.value = true;
    while(ros::ok())
    {
			run_state_update();
	 		ros::spinOnce();
      rate.sleep();
    }

    return 0;

}

