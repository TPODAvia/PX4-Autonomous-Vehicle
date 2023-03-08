#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>
#include "offboard_control.h"
#include "px4_control_cfg.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
using namespace std;
using namespace Eigen;

class PX4Landing {
 public:
    /**
     * default constructor
     */
    PX4Landing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    /**
     * destructor
     */
    ~PX4Landing();
    void Initialize();
   OffboardControl OffboardControl_;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer cmdloop_timer_;
  void CmdLoopCallback(const ros::TimerEvent& event);
  void LandingStateUpdate();
  void ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Px4RelAltCallback(const std_msgs::Float64::ConstPtr &msg);
  void Px4StateCallback(const mavros_msgs::State::ConstPtr& msg);
  Eigen::Vector4d LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw);
  Eigen::Vector3d temp_pos_drone;
  Eigen::Vector3d posxyz_target;     //Expect the spatial position of the aircraft
  Eigen::Vector3d velxy_posz_target; //In offboard mode, the expected value sent to the flight controller
  Eigen::Vector3d ar_pose_big_;      //Large AR code relative to aircraft position
  Eigen::Vector3d ar_pose_small_;    //Small AR code relative to aircraft position
  Eigen::Vector3d  px4_pose_;        //Receive the aircraft position from the flight controller
  Eigen::Vector3d desire_pose_;      //Desired position of the aircraft relative to the landing pad
	float desire_yaw_;                 //Desired yaw angle of the aircraft relative to the landing pad
  mavros_msgs::State px4_state_;     //the state of the aircraft
  double rel_alt_;                   //true altitude of the plane
  mavros_msgs::SetMode mode_cmd_;
  int approach_count = 0;
  int small_count = 0;
  int detect_count = 0;
  int Thres_count_approach;
  int Thres_count_small;
  int Thres_count_detect;
  float landing_vel;
  float search_alt_;
  float markers_id_;  //The AR code to be detected, the default is 4
	float markers_yaw_; //The yaw angle of the AR code relative to the aircraft
  bool detect_state;  //Is the landing board flag detected?
  bool detect_small;  //Whether the small AR code is detected
  bool detect_big;    //Whether a large AR code is detected
  Eigen::Vector4d desire_vel_;
	Eigen::Vector3d desire_xyVel_;
	float desire_yawVel_;
  S_PID s_PidXY,s_PidZ,s_PidYaw;
  S_PID_ITEM s_PidItemX;
  S_PID_ITEM s_PidItemY;
  S_PID_ITEM s_PidItemZ;
  S_PID_ITEM s_PidItemYaw;
  enum Drone
 {
  WAITING,		//Wait for offboard mode
  CHECKING,		//Check aircraft status
  PREPARE,		//Take off to the specified altitude
  SEARCH,	  	//Search
  APPROACH,   //Close to level
  LANDING,	  //Landing board detected, start landing
  LANDOVER,		//Finish
  LAND,		
};
Drone LandingState = WAITING;//Initial state WAITING

  ros::Subscriber ar_pose_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber rel_alt_sub_;
  ros::Subscriber state_sub_;
  ros::ServiceClient set_mode_client_;
};
