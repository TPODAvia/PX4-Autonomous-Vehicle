/***************************************************************************************************************************
*
* Author: bingo
* Email: 1554459957@qq.com
* Time: 2019.10.14
* Description: lidar collision vfh for car v1.0
*  
***************************************************************************************************************************/

//ROS header files
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cmath>
#include <algorithm>
#include <vector>


//topic header file
#include <iostream>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros_msgs/WaypointList.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/HomePosition.h>
#include <GeographicLib/Geocentric.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
using namespace std;


#define PI 3.1415926

struct Point2D
{
	float x;
	float y;
	float z;
};
float scan_distance_max;
float scan_distance_min;
float angle_resolution;
float heading;
float sector_value;
float sector_scale;
Point2D Uavp;
vector<float> map_cv;
vector<double> ranges;

uint32_t init_mask = 0;
Eigen::Vector3d vel_sp_body;                                           

void SetUavPosition(Point2D& uav) {
	Uavp.x = uav.x;
	Uavp.y = uav.y;
	Uavp.z = uav.z;
}

void SetUavHeading(float hd) {
	heading = hd;
	heading += 270;
	heading = (int)heading % 360;
	heading = 360 - heading;
}

void ComputeMV(vector<float> r) {
	float dist[360] = { 0 };
	ranges.clear();
	map_cv.clear();
	int range_size = r.size(); // how many lasers are there

	for (size_t i = 0; i < range_size; i++)
	{
		//A non-zero value (true) if x is a NaN value; and zero (false) otherwise.

		//isinf A non-zero value (true) if x is an infinity; and zero (false) otherwise.
		if (!std::isnan(r[i]) && !std::isinf(r[i]))// get valid value
		{
			float scan_distance = r[i];
			int sector_index = std::floor((i*angle_resolution) / sector_value);//(i*1)/30 sector_index:[1 12]
			if (scan_distance >= scan_distance_max || scan_distance < scan_distance_min)
				scan_distance = 0;
			else
				scan_distance = scan_distance_max - scan_distance;

			dist[sector_index] += scan_distance;
		}
		ranges.push_back(r[i]);
	}

	for (int j = 0; j < (int)(360 / sector_value); j++)//Divide uav into 12 sectors, the smaller the value of each sector, the safer it is.
	{
		map_cv.push_back(dist[j]);
//		printf("dist[%d]=%f-",j,dist[j]);
	}
//	printf("##############################################################################################\n");
}

/*Check whether it is safe within the range of [340 360],[0 20] ahead*/
bool IsFrontSafety()
{
	float goal_sector = (int)(0 - (sector_value - sector_scale) + 360) % 360;//(0-(30-10)+360)%360 = 340,goal_sector = 340
	int start_index = goal_sector / angle_resolution;//start_index = 340
	float scan_distance = 0;
	for (int i = 0; i < (sector_value - sector_scale) * 2 / angle_resolution; i++)//for(i=0;i<40;i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}
	if (scan_distance < 0.1)
	{
		return true;
	}

	return false;
}

/*Input desired coordinate point, output desired nose direction*/
float CalculDirection(Point2D& goal) {
	float ori;
	//Compute arc tangent with two parameters
	//return Principal arc tangent of y/x, in the interval [-pi,+pi] radians.
	//One radian is equivalent to 180/PI degrees.
	float G_theta = atan2((goal.y - Uavp.y), (goal.x - Uavp.x));
	float goal_ori = G_theta * 180 / PI; //The direction information of the target point relative to uav, that is, the angle with the y-axis, positive values ​​represent the first and third phenomena, and negative values ​​represent the second and fourth phenomena. In the ENU coordinate system
	if (goal_ori < 0)
	{
		goal_ori += 360;
	}
	goal_ori -= heading;
	goal_ori += 360;
	goal_ori = (int)goal_ori % 360; //Convert the direction of the target point into the direction in the body coordinate system

	float goal_sector = (int)(goal_ori - sector_value + 360) % 360;
	int start_index = goal_sector / angle_resolution;
	float scan_distance = 0;
	for (int i = 0; i < sector_value * 2 / angle_resolution; i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}
	if (scan_distance < 0.1) // Determine whether the target direction is safe
	{
		ori = goal_ori;
		ori += heading;
		ori = (int)ori % 360;

		return ori; //If it is safe, point the nose at the target
	}


	vector<int> mesh;
	for (int i = 0; i < map_cv.size(); i++) // Determine the CV value in the grid. There are 12 grids in total. The larger the CV value, the greater the possibility of obstacles
	{
		if (map_cv[i] < 0.1)
			mesh.push_back(0);
		else if (map_cv[i] >= 0.1 && map_cv[i] < 0.3)
			mesh.push_back(2);
		else
			mesh.push_back(4);
	}

	vector<float> cand_dir;
	for (int j = 0; j < mesh.size(); j++)
	{
		if (j == mesh.size() - 1) //if(j == 11)
		{
			if (mesh[0] + mesh[mesh.size() - 1] == 0)
				cand_dir.push_back(0.0);
		}
		else
		{
			if (mesh[j] + mesh[j + 1] == 0)
				cand_dir.push_back((j + 1)*sector_value); //Look for a safe angle, in the body coordinate system; that is, to determine the trough
		}
	}

	if (cand_dir.size() != 0) {
		vector<float> delta;
		for (auto &dir_ite : cand_dir) {
			float delte_theta1 = fabs(dir_ite - goal_ori);
			float delte_theta2 = 360 - delte_theta1;
			float delte_theta = delte_theta1 < delte_theta2 ? delte_theta1 : delte_theta2;
			delta.push_back(delte_theta);
		} // Find the trough close to the target area
		int min_index = min_element(delta.begin(), delta.end()) - delta.begin();
		ori = cand_dir.at(min_index);

		ori += heading;
		ori = (int)ori % 360;

		return ori; // Determine the direction of the head
	}

	return -1;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	init_mask |= 1 << 1;
	current_state = *msg;
}

mavros_msgs::HomePosition home_pos;
void home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg) {
	init_mask |= 1 << 2;
	home_pos = *msg;
}

geometry_msgs::PoseStamped local_pos;
Eigen::Vector3d current_local_pos;
bool local_pos_updated = false;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	init_mask |= 1 << 3;
	current_local_pos = mavros::ftf::to_eigen(msg->pose.position);
	local_pos = *msg;
	Point2D pt2d;
	pt2d.x = current_local_pos.x();
	pt2d.y = current_local_pos.y();
	SetUavPosition(pt2d);
	local_pos_updated = true;
}

mavros_msgs::WaypointList waypoints;
void waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg) {
	init_mask |= 1 << 4;
	waypoints = *msg;
}

bool scan_updated = false;
void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//Lidar data callback function
	init_mask |= 1 << 5;
	ComputeMV(msg->ranges);
	scan_updated = true;
}

bool heading_updated = false;
void heading_cb(const std_msgs::Float64::ConstPtr &msg)
{
	init_mask |= 1 << 6;
	SetUavHeading(msg->data);
	heading_updated = true;
}
Eigen::Vector3d current_gps;
void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	init_mask |= 1;
	current_gps = { msg->latitude, msg->longitude, msg->altitude };
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>main function<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoidance_car_vfh");
    ros::NodeHandle nh;
	scan_distance_max = 6.0;//The valid range of lidar in vfh. Obstacles over 2.1m are not considered
	scan_distance_min = 0.1;//Obstacles smaller than 0.1m are not considered
	angle_resolution = 1.0;//This value and sector_value determine how many vectors there are in vfh (360/angle_resolution/sector_value=12)
    heading = 90;
	sector_value = 30;
	sector_scale = 10;//Judge whether [360-sector_scale, 360] and [0, sector_scale] ahead are safe
	Uavp.x = 0;
	Uavp.y = 0;
	mavros_msgs::SetMode client_set_mode;
	client_set_mode.request.custom_mode = "OFFBOARD";

    ros::Subscriber gps_sub = nh.subscribe("/mavros/global_position/global",100,gps_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, local_pos_cb);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 100, scan_cb);
    ros::Subscriber waypoint_sub = nh.subscribe<mavros_msgs::WaypointList>("/mavros/mission/waypoints", 100, waypoints_cb);
    ros::Subscriber homePos_sub = nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 100, home_pos_cb);
    ros::Subscriber head_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 100, heading_cb);

	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    // Frequency [20Hz]
    ros::Rate rate(20.0);
	while (ros::ok() && !current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}

	std::cout << "Please set the waypoint in QGC before running this program." << endl;
	std::cout << "Set the mode to hold and wait a moment" << endl;
	while (ros::ok())
	{
		if (init_mask == 127)
			break;

		// std::cout << "init_mask: " << init_mask << endl;

		ros::spinOnce();
		rate.sleep();
	}

	std::cout << "init ok!" << endl;


	while (ros::ok() && !current_state.guided)
	{
		ros::spinOnce();
		rate.sleep();
	}
	std::cout << "guild ok" << endl;

	std::vector<geometry_msgs::PoseStamped> pose;
	std::cout << "wp size=" << waypoints.waypoints.size() << endl;

	for (int index = 0; index < waypoints.waypoints.size(); index++)//Convert the waypoint information under GPS to the expected position information under ENU
	{
		geometry_msgs::PoseStamped p;
		GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

		Eigen::Vector3d goal_gps(waypoints.waypoints[index].x_lat, waypoints.waypoints[index].y_long, 0);

		Eigen::Vector3d current_ecef;

		earth.Forward(current_gps.x(), current_gps.y(), current_gps.z(),

			current_ecef.x(), current_ecef.y(), current_ecef.z());

		Eigen::Vector3d goal_ecef;

		earth.Forward(goal_gps.x(), goal_gps.y(), goal_gps.z(),

			goal_ecef.x(), goal_ecef.y(), goal_ecef.z());

		Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;

		Eigen::Vector3d enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, current_gps);

		Eigen::Affine3d sp;

		Eigen::Quaterniond q;

		q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())

			* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())

			* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

		sp.translation() = current_local_pos + enu_offset;

		sp.linear() = q.toRotationMatrix();
		//*******************************Store data in the vector container****************************************
		Eigen::Vector3d testv(sp.translation());
		p.pose.position.x = testv[0];
		p.pose.position.y = testv[1];
		std::cout << "wp" << index << " " << p.pose.position.x << " " << p.pose.position.y << endl;
		pose.push_back(p);
	}

	mavros_msgs::PositionTarget pos_target;
	pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	pos_target.position.x = 0;
	pos_target.position.y = 0;
	pos_target.position.z = 1;
	pos_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX
						| mavros_msgs::PositionTarget::IGNORE_VY
						| mavros_msgs::PositionTarget::IGNORE_VZ
						| mavros_msgs::PositionTarget::IGNORE_AFX
						| mavros_msgs::PositionTarget::IGNORE_AFY
						| mavros_msgs::PositionTarget::IGNORE_AFZ
						| mavros_msgs::PositionTarget::FORCE;

	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pos_target);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - last_request > ros::Duration(5.0))){
			if( set_mode_client.call(client_set_mode) &&
				client_set_mode.response.mode_sent){
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if( !current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0))){
				if( arming_client.call(arm_cmd) &&
					arm_cmd.response.success){
					ROS_INFO("Vehicle armed");
					break;
				}
				last_request = ros::Time::now();
			}
		}
		local_pos_pub.publish(pos_target);
		ros::spinOnce();
		rate.sleep();
	}

	bool continue_outer_loop = true;
	for (int i = 0; i < pose.size(); i++)
	{	

		std::cout << "Task number: " << i << endl;
		while (ros::ok() && continue_outer_loop) {
			local_pos_updated = false;
			scan_updated = false;
			heading_updated = false;

			while (ros::ok())
			{
				ros::spinOnce();//Wait for the subscription update to complete
				if (local_pos_updated && scan_updated && heading_updated)
				{
					break;
				}
				rate.sleep();
			}

			if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
				fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
			{
				std::cout << "Arrived" << endl;
				break;//The distance between the current position and the expected waypoint is less than 1m, then exit the waypoint task
			}

			Point2D goal;
			goal.x = pose[i].pose.position.x;// read desired position
			goal.y = pose[i].pose.position.y;
			// std::cout << "goal: " << goal.x << " " << goal.y << endl;

			float direction = CalculDirection(goal);//According to the distribution of obstacles and the desired position, get the desired heading


			if (direction >= -0.5)
			{
				if (direction > 180)
				{
					direction -= 360;
				}
				float arc = 3.1415 / 180 * direction;
				// pos_target.type_mask = 	mavros_msgs::PositionTarget::IGNORE_VX | 
				// 						mavros_msgs::PositionTarget::IGNORE_VY | 
				// 						mavros_msgs::PositionTarget::IGNORE_VZ;
				// pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
				pos_target.position.x = local_pos.pose.position.x + 5* cos(arc);
				pos_target.position.y = local_pos.pose.position.y + 5* sin(arc);
				std::cout << "pos_target.position.x: " << pos_target.position.x << endl;
				std::cout << "pos_target.position.y: " << pos_target.position.y << endl;
				std::cout << "local_pos.pose.position.x: " << local_pos.pose.position.x << endl;
				std::cout << "local_pos.pose.position.y: " << local_pos.pose.position.y << endl;
				local_pos_pub.publish(pos_target);

				ros::Time last_request = ros::Time::now();
				
				while (ros::ok()) 
				{
					local_pos_updated = false;
					scan_updated = false;
					heading_updated = false;

					while (ros::ok())
					{
						ros::spinOnce();//Wait for the subscription update to complete
						if (local_pos_updated && scan_updated && heading_updated)
						{
							break;
						}
						rate.sleep();
					}

					if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
						fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
					{
						std::cout << "Arrive at the waypoint " << i << endl;
						break;
					}

					if (ros::Time::now() - last_request > ros::Duration(3.0))
					{
						std::cout << "Time out update" << endl;
						break;
					}

					if (IsFrontSafety() == false)
					{
						std::cout << "Obstacle ahead" << endl;
						break;
					}

					local_pos_pub.publish(pos_target);
				}
			}
			else
			{
				std::cout << "Direction Error." << direction << endl;
				// pos_target.type_mask = 	mavros_msgs::PositionTarget::IGNORE_VX | 
				// 						mavros_msgs::PositionTarget::IGNORE_VY | 
				// 						mavros_msgs::PositionTarget::IGNORE_VZ;
				// pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
				pos_target.position.x = 0;
				pos_target.position.y = 0;
				local_pos_pub.publish(pos_target);
			}
		}
	}

	client_set_mode.request.custom_mode = "AUTO.RTL";
	while (ros::ok()) {
	
		if (current_state.mode != "AUTO.RTL") 
		{
			if (set_mode_client.call(client_set_mode) && client_set_mode.response.mode_sent) 
			{
				ROS_INFO("Return mode enabled");
				break;
			}
		}
		local_pos_pub.publish(pos_target);

		ros::spinOnce();
		rate.sleep();
	}

	std::cout << "Task Over" << endl;
	rate.sleep();
	ros::shutdown();


    return 0;
}
