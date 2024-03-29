/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.02.08
* Description: Realize the precise landing of px4 quadrotor AR code
***************************************************************************************************************************/
#include "landing_quadrotor.h"

using namespace std;
using namespace Eigen;

PX4Landing::PX4Landing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Landing::CmdLoopCallback, this); //周期为0.1s
  //Subscribe to landing pad relative to aircraft position
  ar_pose_sub_ = nh_private_.subscribe("/ar_pose_marker", 1, &PX4Landing::ArPoseCallback, this,ros::TransportHints().tcpNoDelay());

  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4Landing::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());

  rel_alt_sub_ = nh_private_.subscribe("/mavros/global_position/rel_alt", 1, &PX4Landing::Px4RelAltCallback,this,ros::TransportHints().tcpNoDelay());

  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &PX4Landing::Px4StateCallback,this,ros::TransportHints().tcpNoDelay());
  
  waypoints_sub = nh_private_.subscribe("/mavros/mission/waypoints", 1, &PX4Landing::Waypoints_cb,this,ros::TransportHints().tcpNoDelay());
 
  //【Service】Modify system mode
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");



}

PX4Landing::~PX4Landing() {
  //Destructor
}

/**
* @name       S_SETPOINT_VEL PX4Landing::LandingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)

* @brief      pid control program
*             
* @param[in]  &currentPos The position of the current aircraft relative to the landing board, currentYaw The direction of the current aircraft relative to the landing board
*             
* @param[in]  &expectPos Expected position, expectYaw The expected direction of the aircraft relative to the landing pad: default 0
* @param[out] &The expected speed of x, y, z under the machine system, and the expected speed in the yaw direction.
*
* @param[out] 
**/
Eigen::Vector4d PX4Landing::LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw)
{
  Eigen::Vector4d s_PidOut;

	cout << "currentPos" << endl;
	cout << currentPos << endl;

	/*pid control in X direction*/
	s_PidItemX.difference = expectPos[0] - currentPos[0];
	s_PidItemX.intergral += s_PidItemX.difference;
	if(s_PidItemX.intergral >= 100)		
		s_PidItemX.intergral = 100;
	else if(s_PidItemX.intergral <= -100) 
		s_PidItemX.intergral = -100;
	s_PidItemX.differential =  s_PidItemX.difference  - s_PidItemX.tempDiffer;
  	s_PidItemX.tempDiffer = s_PidItemX.difference;
//	cout << "s_PidItemX.tempDiffer: " << s_PidItemX.tempDiffer << endl;
//	cout << "s_PidItemX.differential: " << s_PidItemX.differential << endl;
	s_PidOut[0] = s_PidXY.p*s_PidItemX.difference + s_PidXY.d*s_PidItemX.differential + s_PidXY.i*s_PidItemX.intergral;

	/*pid control in Y direction*/
	s_PidItemY.difference = expectPos[1] - currentPos[1];
	s_PidItemY.intergral += s_PidItemY.difference;
	if(s_PidItemY.intergral >= 100)		
		s_PidItemY.intergral = 100;
	else if(s_PidItemY.intergral <= -100) 
		s_PidItemY.intergral = -100;
	s_PidItemY.differential =  s_PidItemY.difference  - s_PidItemY.tempDiffer;
  	s_PidItemY.tempDiffer = s_PidItemY.difference;
	s_PidOut[1] = s_PidXY.p*s_PidItemY.difference + s_PidXY.d*s_PidItemY.differential + s_PidXY.i*s_PidItemY.intergral;

	/*pid control in Z direction*/
	s_PidItemZ.difference = expectPos[2] - currentPos[2];
	s_PidItemZ.intergral += s_PidItemZ.difference;
	if(s_PidItemZ.intergral >= 100)		
		s_PidItemZ.intergral = 100;
	else if(s_PidItemZ.intergral <= -100) 
		s_PidItemZ.intergral = -100;
	s_PidItemZ.differential =  s_PidItemZ.difference  - s_PidItemZ.tempDiffer;
  	s_PidItemZ.tempDiffer = s_PidItemZ.difference;
	s_PidOut[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;

	/*pid control of Yaw direction*/
	s_PidItemYaw.difference =  expectYaw - currentYaw;
	s_PidItemYaw.intergral += s_PidItemYaw.difference;
	if(s_PidItemYaw.intergral >= 100)		
		s_PidItemYaw.intergral = 100;
	else if(s_PidItemYaw.intergral <= -100) 
		s_PidItemYaw.intergral = -100;
	s_PidItemYaw.differential =  s_PidItemYaw.difference  - s_PidItemYaw.tempDiffer;
  	s_PidItemYaw.tempDiffer = s_PidItemYaw.difference;
	s_PidOut[3] = s_PidYaw.p*s_PidItemYaw.difference + s_PidYaw.d*s_PidItemYaw.differential + s_PidYaw.i*s_PidItemYaw.intergral;

	return s_PidOut;
}
void PX4Landing::CmdLoopCallback(const ros::TimerEvent& event)
{
  LandingStateUpdate();
}


/**
* @name       void PX4Landing::LandingStateUpdate()
* @brief      State machine update function
*             
* @param[in]  none
*             
* @param[in]  none
* @param[out] 
*
* @param[out] 
**/
void PX4Landing::LandingStateUpdate()
{

	// desire_vel_ = LandingPidProcess(ar_pose_,markers_yaw_,desire_pose_,0);
	// cout << "desire_vel_[0]:  "<< desire_vel_[0] <<endl;
	// cout << "desire_vel_[1]:  "<< desire_vel_[1] <<endl;
	// cout << "desire_vel_[2]:  "<< desire_vel_[2] <<endl;
	// cout << "desire_vel_[3]:  "<< desire_vel_[3] <<endl;
	// cout << "markers_yaw_: "  << markers_yaw_ << endl;
	// cout << "ar_pose_[0]:  "<<  ar_pose_[0] << endl;
	// cout << "ar_pose_[1]:  "<<  ar_pose_[1] << endl;
	// cout << "ar_pose_[2]:  "<<  ar_pose_[2] << endl;
	// cout << "desire_pose_[0]:  "<<  desire_pose_[0] << endl;
	// cout << "desire_pose_[1]:  "<<  desire_pose_[1] << endl;
	// cout << "desire_pose_[2]:  "<<  desire_pose_[2] << endl;
	// cout << "detect_state : " << detect_state << endl;
	switch(LandingState)
	{
		case WAITING:

			if (px4_state_.mode == "MISSION" || px4_state_.mode =="AUTO.MISSION") {

				last_waypoint = waypoint_list.waypoints[waypoint_list.waypoints.size() - 1];
				is_current = last_waypoint.is_current;

				if (last_waypoint.command == 21 && is_current == true)
				{
					cout << "Landing waypoint reached " << last_waypoint.command << endl;	
				}
				else 
				{
					break;
				}
			}
			else if (px4_state_.mode == "AUTO.LAND" || px4_state_.mode == "LAND") {
				ROS_INFO("Vehicle is in landing mode");
				aruco_landing = true;
			} else if (px4_state_.mode == "OFFBOARD" && aruco_landing == true) {
				ROS_INFO("Vehicle is in OFFBOARD landing");
			} else {
				aruco_landing = false;
				break;
			}

			if(detect_state == true && px4_state_.mode == "AUTO.RTL")
			{
				detect_count++;
				cout << "detect goal:" << detect_count << endl;
			}
			if(detect_count == Thres_count_detect)
			{
				mode_cmd_.request.custom_mode = "OFFBOARD";
				set_mode_client_.call(mode_cmd_);
			}
			if(px4_state_.mode != "OFFBOARD")//Wait for offboard mode
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = rel_alt_;
				OffboardControl_.send_pos_xyz(temp_pos_drone);
				// OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			if(px4_state_.mode == "OFFBOARD")
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = rel_alt_;
				LandingState = CHECKING;
				cout << "CHECKING" <<endl;
			}
			break;
		case CHECKING:
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0) 			//Execute landing mode without position information
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";
				set_mode_client_.call(mode_cmd_);
				LandingState = WAITING;	
			}
			else
			{
				LandingState = PREPARE;
				cout << "PREPARE" <<endl;
			}
			
			break;
		case PREPARE:											//Take off to the specified altitude
			posxyz_target[0] = temp_pos_drone[0];
			posxyz_target[1] = temp_pos_drone[1];
			posxyz_target[2] = search_alt_;
			OffboardControl_.send_pos_xyz(posxyz_target);
			// OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			if((rel_alt_<=search_alt_+0.5) && (rel_alt_>=search_alt_-0.3) && detect_state == true) //Half a meter above the preset height and 0.3 meters below are considered to be able to enter the next stage of horizontal approach.
			{
				LandingState = SEARCH;
			}					
			if(px4_state_.mode != "OFFBOARD")				//If you switch to onboard in the middle of preparation, skip to WAITING
			{
				LandingState = WAITING;
			}

			break;
		case SEARCH:
			if(detect_big == true)
			{
				LandingState = APPROACH;
			  cout << "approach" <<endl;
			}	
			else//Here the drone is not actively searching for targets
			{
				posxyz_target[0] = temp_pos_drone[0];
				posxyz_target[1] = temp_pos_drone[1];
				posxyz_target[2] = search_alt_;
				OffboardControl_.send_pos_xyz(posxyz_target);
				// OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			}
			if(px4_state_.mode != "OFFBOARD")				//If you switch to onboard during SEARCH, skip to WAITING
			{
				LandingState = WAITING;
			}
      		cout << "SEARCH" <<endl;
			cout << "x:" << posxyz_target[0] << endl;
			cout << "y:" << posxyz_target[1] << endl;
			cout << "z:" << posxyz_target[2] << endl; 
			break;
		
		case APPROACH:
			{
				
				if(detect_big == true)
				{
					desire_vel_ = LandingPidProcess(ar_pose_big_,markers_yaw_,desire_pose_,desire_yaw_);

					cout << "APPROACH" <<endl;
				}
			  else
				{
					desire_vel_[0] = 0;
					desire_vel_[1] = 0;
					desire_vel_[2] = 0;
					desire_vel_[3] = 0;
				}

				desire_xyVel_[0] = desire_vel_[0];
				desire_xyVel_[1] = desire_vel_[1];
				desire_xyVel_[2] = 0;
				desire_yawVel_ = desire_vel_[3];
				OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);

				if((fabs(ar_pose_big_[0]) < 0.02) && (fabs(ar_pose_big_[1]) < 0.02))			//modify error
				{
					approach_count ++;
					cout<< "Distance_land_count: " << approach_count <<endl;
					
				}

				if((detect_state == true) && approach_count == Thres_count_approach)
				{
					LandingState = LANDING;
					cout << "inter LANDING" << endl;
				}
				


				/**
				 * 
				*/
				// if(ar_pose_[0] > -0.1 && ar_pose_[0] < 0.1)
				// 	if(ar_pose_[1] > -0.1 && ar_pose_[1] < 0.1)
				// 		{
				// 			LandingState = LANDING;
				// 			cout << "inter LANDING" << endl;
				// 		}



			}
			break;

		case LANDING:
			{
				// if(detect_state == true && detect_big == true) 
				// {
				// 	// detect_small = false;
				// 	desire_vel_ = LandingPidProcess(ar_pose_big_,markers_yaw_,desire_pose_,desire_yaw_);

				// 	//cout << "search_" <<endl;
				// }
				// if(detect_state == true && detect_small == true)
				// {
				// 	// detect_big = false;
				// 	desire_vel_ = LandingPidProcess(ar_pose_small_,markers_yaw_,desire_pose_,desire_yaw_);
				// }
			  	// else
				// {
				// 	desire_vel_[0] = 0;
				// 	desire_vel_[1] = 0;
				// 	desire_vel_[2] = 0;
				// 	desire_vel_[3] = 0;
				// }
				
				if(!detect_state)
				{
					desire_vel_[0] = 0;
					desire_vel_[1] = 0;
					desire_vel_[2] = 0;
					desire_vel_[3] = 0;
				}
				else if(detect_state == true && detect_big == true)
				{
					desire_vel_ = LandingPidProcess(ar_pose_big_,markers_yaw_,desire_pose_,desire_yaw_);
				}
				// else if(small_count == 30)
				// {
				// 	desire_vel_ = LandingPidProcess(ar_pose_small_,markers_yaw_,desire_pose_,desire_yaw_);
				// }

				if(detect_small == true && small_count == Thres_count_small)
				{
					LandingState = LANDOVER;
					cout << "LANDOVER" <<endl;
				}
				if(px4_state_.mode != "OFFBOARD")			//If you switch to onboard in the middle of LANDING, skip to WAITING
				{
					LandingState = WAITING;
				}

				// if((desire_vel_[2] >= -0.1) && (ar_pose_[2] < 1))
				// {
				// 	desire_vel_[2] = -0.1;
				// }
				if(detect_small == true && small_count < Thres_count_small) 
				{
					small_count ++;
					
					cout << "small count:" << small_count << endl;
				}
				
				desire_xyVel_[0] = desire_vel_[0];
				desire_xyVel_[1] = desire_vel_[1];
				desire_xyVel_[2] = landing_vel;
				desire_yawVel_ = desire_vel_[3];
				OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);
				cout << "ARUCO BIG" << endl;

			}

			break;
		case LANDOVER:
			{
				desire_vel_ = LandingPidProcess(ar_pose_small_,markers_yaw_,desire_pose_,desire_yaw_);
				desire_xyVel_[0] = desire_vel_[0];
				desire_xyVel_[1] = desire_vel_[1];
				desire_xyVel_[2] = 0.8*desire_vel_[2];
				desire_yawVel_ = desire_vel_[3];
				OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);
				cout << "ARUCO SMALL" << endl;
				if(ar_pose_small_[2] <= 0.45)
				{
					LandingState = LAND;
					cout << "Land" << endl;
				}

				
			}

			break;
		
		case LAND:
			{
				mode_cmd_.request.custom_mode = "AUTO.LAND";
        		set_mode_client_.call(mode_cmd_);
				LandingState = WAITING;
				ros::Duration(20.0).sleep();
				aruco_landing = false;
				// ros::shutdown();

			}


		default:
			cout << "error" <<endl;
	}	

}

/*Receive the position of the landing board relative to the aircraft and the yaw angle*/
void PX4Landing::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	detect_state = false;
	detect_small = false;
	detect_big = false;
  double temp_roll,temp_pitch,temp_yaw;
  tf::Quaternion quat;
	for(auto &item : msg->markers)
	{
		if(item.id == markers_id_)
		{
			detect_state = true;
			detect_small = true;
      		ar_pose_small_[0] = -(item.pose.pose.position.x - 0.0);
      		ar_pose_small_[1] = (item.pose.pose.position.y - 0.0);
      		ar_pose_small_[2] = item.pose.pose.position.z;
      		tf::quaternionMsgToTF(item.pose.pose.orientation,quat);
      		tf::Matrix3x3(quat).getRPY(temp_roll,temp_pitch,temp_yaw);
			markers_yaw_ = temp_yaw;
//			cout << "ar_pose_[0]:"  << ar_pose_small_[0] << endl;
//			cout << "ar_pose_[1]:"  << ar_pose_small_[1] << endl;
//			cout << "ar_pose_[2]:"  << ar_pose_small_[2] << endl;
//			cout << "markers_yaw_: "  << markers_yaw_ << endl;
		}
		if(item.id == 4)
		{
			detect_state = true;
			detect_big = true;
      		ar_pose_big_[0] = -(item.pose.pose.position.x - 0.0);
      		ar_pose_big_[1] = (item.pose.pose.position.y - 0.0);
      		ar_pose_big_[2] = item.pose.pose.position.z;
      		tf::quaternionMsgToTF(item.pose.pose.orientation,quat);
      		tf::Matrix3x3(quat).getRPY(temp_roll,temp_pitch,temp_yaw);
			markers_yaw_ = temp_yaw;
//			cout << "ar_pose_[0]:"  << ar_pose_big_[0] << endl;
//			cout << "ar_pose_[1]:"  << ar_pose_big_[1] << endl;
//			cout << "ar_pose_[2]:"  << ar_pose_big_[2] << endl;
//			cout << "markers_yaw_: "  << markers_yaw_ << endl;
		}
	}
//	cout << "detect_state :" << detect_state << endl;
}
/*Receive the current aircraft position from the flight controller*/                  
void PX4Landing::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}
/*Accept the current true altitude data from the flight controller*/
void PX4Landing::Px4RelAltCallback(const std_msgs::Float64::ConstPtr& msg)
{
	double  data;
	data = msg->data;
	rel_alt_ = data;
	
}
/*Receive current aircraft status from flight controller*/
void PX4Landing::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}

void PX4Landing::Waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg) {
    waypoint_list = *msg;
}

/*initialization*/
void PX4Landing::Initialize()
{
  //Read the search altitude of the aircraft in offboard mode
  nh_private_.param<float>("search_alt_", search_alt_, 3);

  //The id number of the small two-dimensional code, the large two-dimensional code is 
  //used to approach the horizontal plane of the height, and its height data is not used.
  nh_private_.param<float>("markers_id_", markers_id_, 4.0);

  nh_private_.param<float>("PidXY_p", s_PidXY.p, 0.4);
  nh_private_.param<float>("PidXY_d", s_PidXY.d, 0.05);
  nh_private_.param<float>("PidXY_i", s_PidXY.i, 0.01);
  nh_private_.param<float>("PidZ_p", s_PidZ.p, 0.1);
  nh_private_.param<float>("PidZ_d", s_PidZ.d, 0);
  nh_private_.param<float>("PidZ_i", s_PidZ.i, 0);
  nh_private_.param<float>("PidYaw_p", s_PidYaw.p, 0);
  nh_private_.param<float>("PidYaw_d", s_PidYaw.d, 0);
  nh_private_.param<float>("PidYaw_i", s_PidYaw.i, 0);
  nh_private_.param<int>("Thres_count_detect",Thres_count_detect, 0);
  nh_private_.param<int>("Thres_count_approach",Thres_count_approach, 0);
  nh_private_.param<int>("Thres_count_small",Thres_count_small, 0);
  nh_private_.param<float>("landing_vel", landing_vel, -0.1);

  //Desired position of the aircraft relative to the landing pad
  float desire_pose_x,desire_pose_y,desire_pose_z;
  nh_private_.param<float>("desire_pose_x", desire_pose_x, 0);
  nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
  nh_private_.param<float>("desire_pose_z", desire_pose_z, 0);
  nh_private_.param<float>("desire_yaw_", desire_yaw_, 0);
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;
  desire_pose_[2] = desire_pose_z;

  detect_state = false;
  detect_big = false;
  detect_small = false;
  desire_vel_[0] = 0;
  desire_vel_[1] = 0;
  desire_vel_[2] = 0;
  desire_vel_[3] = 0;
	desire_xyVel_[0]  = 0;
	desire_xyVel_[1]  = 0;
	desire_xyVel_[2]  = 0;
	desire_yawVel_ = 0;
  s_PidItemX.tempDiffer = 0;
  s_PidItemY.tempDiffer = 0;
  s_PidItemZ.tempDiffer = 0;
  s_PidItemYaw.tempDiffer = 0;
  s_PidItemX.intergral = 0;
  s_PidItemY.intergral = 0;
  s_PidItemZ.intergral = 0;
  s_PidItemYaw.intergral = 0;

	cout << "search_alt_ = " << search_alt_ << endl;
	cout << "markers_id_ = " << markers_id_ << endl;
	cout << "PidXY_p = " << s_PidXY.p << endl;
	cout << "PidXY_d = " << s_PidXY.d << endl;
	cout << "PidXY_i = " << s_PidXY.i << endl;
	cout << "PidZ_p = " << s_PidZ.p << endl;
	cout << "PidZ_d = " << s_PidZ.d << endl;
	cout << "PidZ_i = " << s_PidZ.i << endl;
	cout << "PidYaw_p = " << s_PidYaw.p << endl;
	cout << "PidYaw_d = " << s_PidYaw.d << endl;
	cout << "PidYaw_i = " << s_PidYaw.i << endl;
	cout << "desire_pose_x = " << desire_pose_[0] << endl;
	cout << "desire_pose_y = " << desire_pose_[1] << endl;
	cout << "desire_pose_z = " << desire_pose_[2] << endl;
	cout << "desire_yaw_ = " << desire_yaw_ << endl;
	cout << "Thres_count_detect = " << Thres_count_detect << endl;
	cout << "Thres_count_approach = " << Thres_count_approach << endl;
	cout << "Thres_count_small = " << Thres_count_small << endl;
	cout << "landing_vel = " << landing_vel << endl;

}
int main(int argc, char** argv) {
  ros::init(argc,argv,"landing_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  PX4Landing PX4Landing(nh, nh_private);

  ros::spin();
  return 0;
}
