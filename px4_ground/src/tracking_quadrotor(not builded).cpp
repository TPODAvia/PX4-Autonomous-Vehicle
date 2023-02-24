/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2020.03.10
* Description: 实现px4 quadrotor 二维码跟踪
***************************************************************************************************************************/
#include "tracking_quadrotor.h"
using namespace std;
using namespace Eigen;
PX4Tracking::PX4Tracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Tracking::CmdLoopCallback, this); //周期为0.1s
  //订阅二维码相对飞机正前方位置
  ar_pose_sub_ = nh_private_.subscribe("/ar_pose_marker", 1, &PX4Tracking::ArPoseCallback, this,ros::TransportHints().tcpNoDelay());

  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4Tracking::Px4PosCallback,this,ros::TransportHints().tcpNoDelay());

  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &PX4Tracking::Px4StateCallback,this,ros::TransportHints().tcpNoDelay());
  // 【服务】修改系统模式
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

}

PX4Tracking::~PX4Tracking() {
  //Destructor
}

/**
* @name       S_SETPOINT_VEL PX4Tracking::TRACKINGPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)

* @brief      pid控制程序,机体坐标系下控制无人机
*             
* @param[in]  &currentPos 当前飞机相对二维码的位置
*             
* @param[in]  &expectPos 期望位置 expectPos[0]:相对二维码前后方向距离；expectPos[1]:相对二维码左右方向距离；expectPos[2]:相对二维码上下方向距离
* @param[out] y,z的期望速度,以及yaw方向的期望速度。
*
* @param[out] 
**/
Eigen::Vector3d PX4Tracking::TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos)
{
  Eigen::Vector3d s_PidOut;

	/*前后方向的pid控制，输出机体系下y方向的速度控制*/
	s_PidItemY.difference = currentPos[2] - expectPos[0];
	s_PidItemY.intergral += s_PidItemY.difference;
	if(s_PidItemY.intergral >= 100)		
		s_PidItemY.intergral = 100;
	else if(s_PidItemY.intergral <= -100) 
		s_PidItemY.intergral = -100;
	s_PidItemY.differential =  s_PidItemY.difference  - s_PidItemY.tempDiffer;
  s_PidItemY.tempDiffer = s_PidItemY.difference;
//	cout << "s_PidItemY.tempDiffer: " << s_PidItemY.tempDiffer << endl;
//	cout << "s_PidItemY.differential: " << s_PidItemY.differential << endl;
	s_PidOut[0] = s_PidY.p*s_PidItemY.difference + s_PidY.d*s_PidItemY.differential + s_PidY.i*s_PidItemY.intergral;

	/*左右方向的pid控制，输出yaw方向速度控制*/
	s_PidItemYaw.difference = expectPos[1] - currentPos[0];
	s_PidItemYaw.intergral += s_PidItemYaw.difference;
	if(s_PidItemYaw.intergral >= 100)		
		s_PidItemYaw.intergral = 100;
	else if(s_PidItemYaw.intergral <= -100) 
		s_PidItemYaw.intergral = -100;
	s_PidItemYaw.differential =  s_PidItemYaw.difference  - s_PidItemYaw.tempDiffer;
  s_PidItemYaw.tempDiffer = s_PidItemYaw.difference;
	s_PidOut[1] = s_PidYaw.p*s_PidItemYaw.difference + s_PidYaw.d*s_PidItemYaw.differential + s_PidYaw.i*s_PidItemYaw.intergral;

	/*上下方向的pid控制，输出z方向的速度控制*/
	s_PidItemZ.difference = expectPos[2] - currentPos[1];
	s_PidItemZ.intergral += s_PidItemZ.difference;
	if(s_PidItemZ.intergral >= 100)		
		s_PidItemZ.intergral = 100;
	else if(s_PidItemZ.intergral <= -100) 
		s_PidItemZ.intergral = -100;
	s_PidItemZ.differential =  s_PidItemZ.difference  - s_PidItemZ.tempDiffer;
  s_PidItemZ.tempDiffer = s_PidItemZ.difference;
	s_PidOut[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;

	return s_PidOut;
}
void PX4Tracking::CmdLoopCallback(const ros::TimerEvent& event)
{
  TrackingStateUpdate();
}


/**
* @name       void PX4Tracking::TrackingStateUpdate()
* @brief      状态机更新函数
*             
* @param[in]  无
*             
* @param[in]  无
* @param[out] 
*
* @param[out] 
**/
void PX4Tracking::TrackingStateUpdate()
{


//	desire_vel_ = TrackingPidProcess(ar_pose_,desire_pose_);
//	cout << "desire_vel_[0]:  "<< desire_vel_[0] <<endl;
//	cout << "desire_vel_[1]:  "<< desire_vel_[1] <<endl;
//	cout << "desire_vel_[2]:  "<< desire_vel_[2] <<endl;
//	cout << "desire_vel_[3]:  "<< desire_vel_[3] <<endl;
//	cout << "markers_yaw_: "  << markers_yaw_ << endl;
//	cout << "ar_pose_[0]:  "<<  ar_pose_[0] << endl;
//	cout << "ar_pose_[1]:  "<<  ar_pose_[1] << endl;
//	cout << "ar_pose_[2]:  "<<  ar_pose_[2] << endl;
//	cout << "desire_pose_[0]:  "<<  desire_pose_[0] << endl;
//	cout << "desire_pose_[1]:  "<<  desire_pose_[1] << endl;
//	cout << "desire_pose_[2]:  "<<  desire_pose_[2] << endl;
//	cout << "detect_state : " << detect_state << endl;
	switch(TrackingState)
	{
		case WAITING:
			if(px4_state_.mode != "OFFBOARD")//等待offboard模式
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				OffboardControl_.send_pos_setpoint(temp_pos_drone, 0);
			}
			else
			{
				temp_pos_drone[0] = px4_pose_[0];
				temp_pos_drone[1] = px4_pose_[1];
				temp_pos_drone[2] = px4_pose_[2];
				TrackingState = CHECKING;
				cout << "CHECKING" <<endl;
			}
				//cout << "WAITING" <<endl;
			break;
		case CHECKING:
			if(px4_pose_[0] == 0 && px4_pose_[1] == 0) 			//没有位置信息则执行降落模式
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd_.request.custom_mode = "AUTO.LAND";
				set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;	
			}
			else
			{
				TrackingState = PREPARE;
				cout << "PREPARE" <<endl;
			}
			
			break;
		case PREPARE:											//起飞到指定高度
			posxyz_target[0] = temp_pos_drone[0];
			posxyz_target[1] = temp_pos_drone[1];
			posxyz_target[2] = search_alt_;
			if((px4_pose_[2]<=search_alt_+0.1) && (px4_pose_[2]>=search_alt_-0.1))
			{
				TrackingState = SEARCH;
			}
			else if(detect_state == true)
			{
			//	TrackingState = SEARCH;
			}
			OffboardControl_.send_pos_setpoint(posxyz_target, 0);					
			if(px4_state_.mode != "OFFBOARD")				//如果在准备中途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}

			break;
		case SEARCH:
			if(detect_state == true)
			{
				TrackingState = TRACKING;
			  cout << "TRACKING" <<endl;
			}	
			else//这里无人机没有主动搜寻目标
			{
				OffboardControl_.send_pos_setpoint(posxyz_target, 0);
			}
			if(px4_state_.mode != "OFFBOARD")				//如果在SEARCH途中切换到onboard，则跳到WAITING
			{
				TrackingState = WAITING;
			}
     // cout << "SEARCH" <<endl;
			break;
		case TRACKING:
			{
				if(detect_state == true)
				{
					desire_vel_ = TrackingPidProcess(ar_pose_,desire_pose_);

					//cout << "search_" <<endl;
				}
			  else
				{
					desire_vel_[0] = 0;
					desire_vel_[1] = 0;
					desire_vel_[2] = 0;
				}
				if(ar_pose_[2] <= 0.3)
				{
					TrackingState = TRACKOVER;
					cout << "TRACKOVER" <<endl;
				}
				if(px4_state_.mode != "OFFBOARD")			//如果在TRACKING中途中切换到onboard，则跳到WAITING
				{
					TrackingState = WAITING;
				}
				desire_yzVel_[0] = desire_vel_[0];
				desire_yzVel_[1] = desire_vel_[2];
				desire_yawVel_ = desire_vel_[1];

				OffboardControl_.send_body_velyz_setpoint(desire_yzVel_,desire_yawVel_);
			}

			break;
		case TRACKOVER:
			{
				mode_cmd_.request.custom_mode = "AUTO.LAND";
        set_mode_client_.call(mode_cmd_);
				TrackingState = WAITING;
			}

			break;

		default:
			cout << "error" <<endl;
	}	

}

/*接收降落板相对飞机的位置以及偏航角*/
void PX4Tracking::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	detect_state = false;
	for(auto &item : msg->markers)
	{
		if(item.id == markers_id_)
		{
			detect_state = true;
      ar_pose_[0] = item.pose.pose.position.x;
      ar_pose_[1] = item.pose.pose.position.y;
      ar_pose_[2] = item.pose.pose.position.z;

//			cout << "ar_pose_[0]:"  << ar_pose_[0] << endl;
//			cout << "ar_pose_[1]:"  << ar_pose_[1] << endl;
//			cout << "ar_pose_[2]:"  << ar_pose_[2] << endl;
//			cout << "markers_yaw_: "  << markers_yaw_ << endl;
		}
	}
//	cout << "detect_state :" << detect_state << endl;
}

/*接收来自飞控的当前飞机位置*/                  
void PX4Tracking::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    px4_pose_ = pos_drone_fcu_enu;
}
/*接收来自飞控的当前飞机状态*/
void PX4Tracking::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	px4_state_ = *msg;
}

/*初始化*/
void PX4Tracking::Initialize()
{
  //读取offboard模式下飞机的搜索高度
  nh_private_.param<float>("search_alt_", search_alt_, 2);

  nh_private_.param<float>("markers_id_", markers_id_, 4.0);

  nh_private_.param<float>("PidY_p", s_PidY.p, 0.6);
  nh_private_.param<float>("PidY_d", s_PidY.d, 0.01);
  nh_private_.param<float>("PidY_i", s_PidY.i, 0);
  nh_private_.param<float>("PidZ_p", s_PidZ.p, 0.6);
  nh_private_.param<float>("PidZ_d", s_PidZ.d, 0.01);
  nh_private_.param<float>("PidZ_i", s_PidZ.i, 0);
  nh_private_.param<float>("PidYaw_p", s_PidYaw.p, 0.4);
  nh_private_.param<float>("PidYaw_d", s_PidYaw.d, 0.01);
  nh_private_.param<float>("PidYaw_i", s_PidYaw.i, 0);

  //期望的飞机相对二维码的位置
	float desire_pose_x,desire_pose_y,desire_pose_z;
  nh_private_.param<float>("desire_pose_x", desire_pose_x, 6.5);
  nh_private_.param<float>("desire_pose_y", desire_pose_y, 0);
  nh_private_.param<float>("desire_pose_z", desire_pose_z, 0);
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;
  desire_pose_[2] = desire_pose_z;

  detect_state = false;
  desire_vel_[0] = 0;
  desire_vel_[1] = 0;
  desire_vel_[2] = 0;
	desire_yzVel_[0]  = 0;
	desire_yzVel_[1]  = 0;
  s_PidItemY.tempDiffer = 0;
  s_PidItemYaw.tempDiffer = 0;
  s_PidItemZ.tempDiffer = 0;
  s_PidItemY.intergral = 0;
  s_PidItemYaw.intergral = 0;
  s_PidItemZ.intergral = 0;
	cout << "search_alt_ = " << search_alt_ << endl;
	cout << "markers_id_ = " << markers_id_ << endl;
	cout << "PidY_p = " << s_PidY.p << endl;
	cout << "PidY_d = " << s_PidY.d << endl;
	cout << "PidY_i = " << s_PidY.i << endl;
	cout << "PidZ_p = " << s_PidZ.p << endl;
	cout << "PidZ_d = " << s_PidZ.d << endl;
	cout << "PidZ_i = " << s_PidZ.i << endl;
	cout << "PidYaw_p = " << s_PidYaw.p << endl;
	cout << "PidYaw_d = " << s_PidYaw.d << endl;
	cout << "PidYaw_i = " << s_PidYaw.i << endl;
	cout << "desire_pose_x = " << desire_pose_[0] << endl;
	cout << "desire_pose_y = " << desire_pose_[1] << endl;
	cout << "desire_pose_z = " << desire_pose_[2] << endl;

}
int main(int argc, char** argv) {
  ros::init(argc,argv,"tracking_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  PX4Tracking PX4Tracking(nh, nh_private);

  ros::spin();
  return 0;
}
