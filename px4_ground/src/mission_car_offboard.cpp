/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.02.19
* Description: Realize that the car goes to the predetermined waypoint in the offboard mode. The position control uses the NED coordinate system in the flight control. 
* You can refer to this blog：https://blog.csdn.net/qq_33641919/article/details/101003978
*
***************************************************************************************************************************/
#include "mission_car_offboard.h"
using namespace std;
using namespace Eigen;
MissionCar::MissionCar(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &MissionCar::CmdLoopCallback, this); // Define timer for constant loop rate 0.1s

  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &MissionCar::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());

}

MissionCar::~MissionCar() {
  //Destructor
}
void MissionCar::CmdLoopCallback(const ros::TimerEvent& event)
{
	MissionStateUpdate();
}

/**
* @name      bool MissionCar::CarPosControl(Eigen::Vector3d &currPose,float currYaw,Eigen::Vector3d &expectPose)

* @brief      Realize the position control of the px4 car in offboard mode
*             
* @param[in]  &currPose currYaw The current position and yaw angle of the car (NED coordinate system)
*             
* @param[in]  &expectPose expected position (NED coordinate system)
* @param[out] &Whether to reach the target point
*
* @param[out] 
**/
bool MissionCar::CarPosControl(Eigen::Vector3d &currPose,float currYaw,Eigen::Vector3d &expectPose)
{
    float expectYaw;
    float tanX,tanY;
    Eigen::Vector3d expectAtt;
    tanX = expectPose[0] - currPose[0];
    tanY = expectPose[1] - currPose[1];
    //Compute arc tangent with two parameters
    //return Principal arc tangent of y/x, in the interval [-pi,+pi] radians.
    //One radian is equivalent to 180/PI degrees.
    expectYaw = atan2(tanY,tanX)*(180/pi);
    expectAtt[0] = 0;
    expectAtt[1] = 0;
    expectAtt[2] = expectYaw;
    if(expectAtt[2] < 0)
    {
    	expectAtt[2] = 360+expectAtt[2];
    }
   	// cout << "expectYaw = " << expectAtt[2] << endl;
    currYaw = currYaw * (180/pi);

	Eigen::Vector3d pos_sp;
	pos_sp[0] = expectPose[0];
	pos_sp[1] = expectPose[1];
	pos_sp[2] = expectPose[2];
	OffboardControl_.send_pos_xyz(pos_sp);

	// OffboardControl_.send_attitude_rate_setpoint(expectAtt,40);
    // According to the waypoint is less than 0.5m as the destination
	// std::cout << "currPose[0] = " << currPose[0] << std::endl;
	// std::cout << "currPose[1] = " << currPose[1] << std::endl;
	// std::cout << "expectPose[0] = " << expectPose[0] << std::endl;
	// std::cout << "expectPose[1] = " << expectPose[1] << std::endl;

	if(sqrt((currPose[0]-expectPose[0])*(currPose[0]-expectPose[0]) + (currPose[1]-expectPose[1])*(currPose[1]-expectPose[1])) <= 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}


void MissionCar::MissionStateUpdate()
{
    Eigen::Vector3d desirePose;
	if(mission_finish_ == false)
	{

		if(CarPosControl(car_pose_,curr_yaw_,desire_pose_))
		{
			mission_step_ ++;
			if(mission_step_ == 2)
			{
				desire_pose_[0] = step2_Pose_x;
				desire_pose_[1] = step2_Pose_y;	
			}
			else if(mission_step_ == 3)
			{
				desire_pose_[0] = step3_Pose_x;
				desire_pose_[1] = step3_Pose_y;
			}
			else if(mission_step_ == 4)
			{
				desire_pose_[0] = step4_Pose_x;
				desire_pose_[1] = step4_Pose_y;
			}
			 if(mission_step_ == 5)
			{
				mission_finish_ = true;
				cout << "mission finish" << endl;
				ros::shutdown();
			}
			else
			{
	      		cout << "current step: " << mission_step_ << endl;
				cout << "current desire poseX" << desire_pose_[0] << endl;
				cout << "current desire poseY" << desire_pose_[1] << endl;
			}
		}
	}
	else
	{
		Eigen::Vector3d stopAtt;
		stopAtt[0] = 0;
		stopAtt[1] = 0;
		stopAtt[2] = curr_yaw_*(180/pi);
		OffboardControl_.send_attitude_setpoint(stopAtt,0);
	}
	
//	cout << "missionStep = " << mission_step_ << endl;
}
// Receive the current car position and yaw angle from the flight control, and convert it into the NED coordinate system in the flight control             
void MissionCar::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double temp_roll,temp_pitch,temp_yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(temp_roll,temp_pitch,temp_yaw);
    curr_yaw_ = -temp_yaw + pi/2;
    // Guaranteed that the range of curr_yaw_ is [-pi,+pi]
    if(curr_yaw_ > pi)
	{
		curr_yaw_ = curr_yaw_ - 2*pi;
	}
	//cout << "curr_yaw = " << curr_yaw_ << endl;
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    car_pose_[0] = pos_drone_fcu_enu[0];
    car_pose_[1] = pos_drone_fcu_enu[1];
    car_pose_[2] = -pos_drone_fcu_enu[2];
    //cout << "car_pose_x = " << car_pose_[0] << endl;
    //cout << "car_pose_y = " << car_pose_[1] << endl;
   
}
void MissionCar::Initialize()
{
	mission_step_ = 1;
	mission_finish_ = false;
  nh_private_.param<float>("step1_Pose_x", step1_Pose_x, 0);
  nh_private_.param<float>("step1_Pose_y", step1_Pose_y, 0);
  nh_private_.param<float>("step2_Pose_x", step2_Pose_x, 10);
  nh_private_.param<float>("step2_Pose_y", step2_Pose_y, 10);
  nh_private_.param<float>("step3_Pose_x", step3_Pose_x, -10);
  nh_private_.param<float>("step3_Pose_y", step3_Pose_y, -10);
  nh_private_.param<float>("step4_Pose_x", step4_Pose_x, 0);
  nh_private_.param<float>("step4_Pose_y", step4_Pose_y, 0);
  nh_private_.param<float>("desire_vel_",  desire_vel_, 10);
	desire_pose_[0] = step1_Pose_x;
	desire_pose_[1] = step1_Pose_y;

	cout << "current desire vel: " << desire_vel_ << endl;
	cout << "current step: " << mission_step_ << endl;
	cout << "current desire poseX: " << step1_Pose_x << endl;
	cout << "current desire poseY: " << step1_Pose_y << endl;
}
int main(int argc, char** argv) {
  ros::init(argc,argv,"mission_car_offboard");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  MissionCar MissionCar(nh, nh_private);

  ros::spin();
  return 0;
}
