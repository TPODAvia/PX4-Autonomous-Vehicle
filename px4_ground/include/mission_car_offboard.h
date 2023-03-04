#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Dense>
#include "offboard_control.h"
#include "tf/transform_datatypes.h"
#include <std_msgs/UInt8.h>
using namespace std;
using namespace Eigen;
class MissionCar {
 public:
    /**
     * default constructor
     */
    MissionCar(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    /**
     * destructor
     */
    ~MissionCar();
    void Initialize();


  OffboardControl OffboardControl_;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer cmdloop_timer_;

  Eigen::Vector3d  car_pose_;     //Receive car position from flight controller
  Eigen::Vector3d  desire_pose_;  //Expected car position in mission
  float curr_yaw_;                //Current heading angle
  float mission_step_;

  /*four desired waypoints*/
  float step1_Pose_x;
  float step1_Pose_y;
  float step2_Pose_x;
  float step2_Pose_y;
  float step3_Pose_x;
  float step3_Pose_y;
  float step4_Pose_x;
  float step4_Pose_y;

  float desire_vel_;
  bool mission_finish_;

  void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void CmdLoopCallback(const ros::TimerEvent& event);
  void MissionStateUpdate(void);
  bool CarPosControl(Eigen::Vector3d &currPose,float currYaw,Eigen::Vector3d &expectPose);
  ros::Subscriber position_sub_;
};
