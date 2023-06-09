#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <map>
#include <tuple>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <functional>
#include <sstream>
#include <chrono>
#include <mutex>
#include <condition_variable>

mavros_msgs::State current_state_g;
nav_msgs::Odometry current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;
sensor_msgs::NavSatFix my_home_position;
std::vector<ros::Subscriber> drone_subscribers;
mavros_msgs::GlobalPositionTarget leader_home_position;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g;
int my_drone_id;
int leader_drone_id_g = 0;
bool publish_my_home_position = true;
int first_init_leader_id = -1;
int drone_nums;
float shift_x;
float shift_y;
float shift_alt;
double leader_shift_x;
double leader_shift_y;
double leader_alt_z;
std::string ros_namespace;
geometry_msgs::PoseStamped leader_shift;

std_msgs::Bool leader_landing_command;
std_msgs::String swarm_data;

ros::Publisher my_home_position_pub;
ros::Publisher local_pos_pub;
// ros::Publisher global_lla_pos_pub;
// ros::Publisher global_lla_pos_pub_raw;
ros::Publisher my_drone_id_ready_pub;
ros::Publisher reached_state_pub;
ros::Publisher landing_command_pub;

ros::Subscriber my_drone_id_ready_sub;
ros::Subscriber landing_command_sub;
ros::Subscriber local_position_sub;
ros::Subscriber state_sub;
ros::Subscriber global_position_sub;
ros::Subscriber leader_global_position_sub;
ros::Subscriber leader_local_position_sub;

ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;
ros::ServiceClient auto_waypoint_pull_client;
ros::ServiceClient auto_waypoint_push_client;
ros::ServiceClient auto_waypoint_set_current_client;

/**
\ingroup control_functions
This structure is a convenient way to format waypoints
*/
struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};

struct DroneData {
    int drone_id;
    std::string leader_status;
    std::string reached_status;
};