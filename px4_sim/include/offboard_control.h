#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include "mavros_msgs/MountControl.h"

#define pi  3.1415926
using namespace std;
using namespace Eigen;

class OffboardControl {
 public:
    /**
     * default constructor
     */
  OffboardControl(void):
    offboard_nh_("~") {
  mavros_setpoint_pos_pub_ = offboard_nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
  mount_control_pub_ = offboard_nh_.advertise<mavros_msgs::MountControl>("/mavros/mount_control/command", 1);

  mavros_setpoint_local_pos_pub_ = offboard_nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);


  actuator_setpoint_pub_ = offboard_nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);
  setpoint_raw_attitude_pub_ = offboard_nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

}
    void send_velxy_posz_setpoint(const Eigen::Vector3d& vel_sp, float desire_z);
    void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp);
    void send_pos_xyz(const Eigen::Vector3d& pos_sp);
    void send_velxyz_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp);
    void send_body_velxyz_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp);
    void send_body_velyz_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp);
    void send_local_pos_setpoint(const Eigen::Vector3d& pos_sp);
    void send_actuator_setpoint(const Eigen::Vector4d& actuator_sp);
    void send_attitude_setpoint(const Eigen::Vector3d& _AttitudeReference,float thrust_sp);
    void send_attitude_rate_setpoint(const Eigen::Vector3d& attitude_rate_sp, float thrust_sp);
    void send_mount_control_command(const Eigen::Vector3d& mount_sp);
    void send_body_velxy_posz_setpoint(const Eigen::Vector3d& vel_sp, float desire_z);
  private:
    ros::NodeHandle offboard_nh_;
    ros::Publisher mavros_setpoint_pos_pub_;
    ros::Publisher mavros_setpoint_local_pos_pub_;
    ros::Publisher actuator_setpoint_pub_;
    ros::Publisher setpoint_raw_attitude_pub_;
    ros::Publisher mount_control_pub_;

};

//   Send the expected yz velocity and the expected yaw angular velocity to the flight controller in the body coordinate system for AR code tracking
// （refer to：https://docs.px4.io/master/en/flight_modes/offboard.html）,
//  In the ros body coordinate system pos_setpoint.velocity.y+ the aircraft flies forward, pos_setpoint.velocity.x+ the aircraft flies to the right
void OffboardControl::send_body_velyz_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = /*1 +*/ 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
    pos_setpoint.coordinate_frame = 8;

    pos_setpoint.velocity.x = 0;
    pos_setpoint.position.x = 0;
    pos_setpoint.velocity.y = vel_sp[0];
    pos_setpoint.velocity.z = vel_sp[1];
    pos_setpoint.yaw_rate = yaw_sp;
    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}
//  Send xyz velocity expectations and expected yaw angular velocity to the flight controller in the "body" coordinate system
//（refer to：https://docs.px4.io/master/en/flight_modes/offboard.html）
void OffboardControl::send_body_velxyz_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 1 + 2 + 4 +/* 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
    pos_setpoint.coordinate_frame = 8;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.yaw_rate = yaw_sp;
    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}
// Send xyz velocity expectations and expected yaw angular velocity to the flight controller in the "local frame" local coordinate system
void OffboardControl::send_velxyz_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 1 + 2 + 4 +/* 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.yaw_rate = yaw_sp;
    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}
//机体坐标系下发送xy速度期望值以及高度z期望值至飞控（输入：期望xy,期望高度）
void OffboardControl::send_body_velxy_posz_setpoint(const Eigen::Vector3d& vel_sp, float desire_z)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 1 + 2 + /*4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
    pos_setpoint.coordinate_frame = 8;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.position.z = desire_z;

    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}
//Send xy velocity expectation value and altitude z expectation value to the flight controller in the body coordinate system (input: expected xy, expected altitude)
void OffboardControl::send_velxy_posz_setpoint(const Eigen::Vector3d& vel_sp, float desire_z)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 1 + 2 + /*4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.position.z = desire_z;

    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}
//Send position expectation to flight controller (input: expected xyz, expected yaw)
void OffboardControl::send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024*/ + 2048;
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.yaw = yaw_sp;
    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}

//Send position expectation to flight controller (input: expected xyz, expected yaw)
void OffboardControl::send_pos_xyz(const Eigen::Vector3d& pos_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    // pos_setpoint.yaw = yaw_sp;
    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}

//Publish the position control to the flight controller through the topic /mavros/setpoint_position/local
void OffboardControl::send_local_pos_setpoint(const Eigen::Vector3d& pos_sp)
{
    geometry_msgs::PoseStamped pos_target;
    pos_target.pose.position.x = pos_sp[0];
    pos_target.pose.position.y = pos_sp[1];
    pos_target.pose.position.z = pos_sp[2];
    mavros_setpoint_local_pos_pub_.publish(pos_target);
}



//Send bottom layer to flight controller (input: MxMyMz, desired thrust)
void OffboardControl::send_actuator_setpoint(const Eigen::Vector4d& actuator_sp)
{
    mavros_msgs::ActuatorControl actuator_setpoint;

    actuator_setpoint.group_mix = 0;
    actuator_setpoint.controls[0] = actuator_sp(0);
    actuator_setpoint.controls[1] = actuator_sp(1);
    actuator_setpoint.controls[2] = actuator_sp(2);
    actuator_setpoint.controls[3] = actuator_sp(3);
    actuator_setpoint.controls[4] = 0.0;
    actuator_setpoint.controls[5] = 0.0;
    actuator_setpoint.controls[6] = 0.0;
    actuator_setpoint.controls[7] = 0.0;

    actuator_setpoint_pub_.publish(actuator_setpoint);
}

//Send the expected value of the angle to the flight control (input: expected angle - Euler angle, expected thrust) (the expected value is the angle value NED coordinate system, not the radian value)
void OffboardControl::send_attitude_setpoint(const Eigen::Vector3d& _AttitudeReference,float thrust_sp)
{
    mavros_msgs::AttitudeTarget att_setpoint;
    Eigen::Vector3d temp_att;
    tf2::Quaternion quat_obj;

    /*Angle value converted to radian value*/
    temp_att[0] = _AttitudeReference[0]/(180/pi);
    temp_att[1] = _AttitudeReference[1]/(180/pi);
    temp_att[2] =(90- _AttitudeReference[2])/(180/pi);
    if(temp_att[2]<0)
    {
    	temp_att[2] = 2*pi+temp_att[2];
    }
    /*Euler Angle to Quaternion*/
    quat_obj.setRPY( temp_att[0], temp_att[1], temp_att[2]);
    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = 1 + 2 + 4 + 8 + 16 + 32/* + 64 + 128*/;
    att_setpoint.orientation.x = quat_obj.x();
    att_setpoint.orientation.y = quat_obj.y();
    att_setpoint.orientation.z = quat_obj.z();
    att_setpoint.orientation.w = quat_obj.w();

    att_setpoint.thrust = thrust_sp;

    setpoint_raw_attitude_pub_.publish(att_setpoint);

}

//Send the expected value of angular velocity to the flight controller (input: expected angular velocity, expected thrust)
void OffboardControl::send_attitude_rate_setpoint(const Eigen::Vector3d& attitude_rate_sp, float thrust_sp)
{
    mavros_msgs::AttitudeTarget att_setpoint;

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = /*1 + 2 + 4 + */8 + 16 + 32 +/* 64 + */128;
    att_setpoint.body_rate.x = attitude_rate_sp[0];
    att_setpoint.body_rate.y = attitude_rate_sp[1];
    att_setpoint.body_rate.z = attitude_rate_sp[2];

    att_setpoint.thrust = thrust_sp;

    setpoint_raw_attitude_pub_.publish(att_setpoint);

}

void OffboardControl::send_mount_control_command(const Eigen::Vector3d& mount_sp)
{
  mavros_msgs::MountControl mount_setpoint;
  mount_setpoint.mode = 2;
  mount_setpoint.pitch = mount_sp[0]; // Gimbal Pitch
  mount_setpoint.roll = mount_sp[1]; // Gimbal  Yaw
  mount_setpoint.yaw = mount_sp[2]; // Gimbal  Yaw

  mount_control_pub_.publish(mount_setpoint);

}
