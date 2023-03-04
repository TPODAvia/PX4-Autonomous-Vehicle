/***************************************************************************************************************************
*
* Author: bingo
* Email: 1554459957@qq.com
* Time: 2019.10.14
* Description: lidar collision vfh for quad v1.0
*  
***************************************************************************************************************************/

// ROS header files
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cmath>
#include <algorithm>
#include <vector>


// topic header file
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
float desire_z = 1.5; // high expectations

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
		if (!std::isnan(r[i]) && !std::isinf(r[i]))//Take effective value
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

	for (int j = 0; j < (int)(360 / sector_value); j++)//Divide the uav into 12 sectors, the smaller the value of each sector, the safer it is.
	{
		map_cv.push_back(dist[j]);
//		printf("dist[%d]=%f-",j,dist[j]);
	}
//	printf("##############################################################################################\n");
}

/*Detect whether it is safe within the range of [340 360], [0 20] ahead*/
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

/*Input the desired coordinate point and output the desired nose direction*/
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
	goal_ori = (int)goal_ori % 360; //Transform the direction of the target point into the direction in the body coordinate system

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
	if (scan_distance < 0.1)//Determine whether the target direction is safe
	{
		ori = goal_ori;
		ori += heading;
		ori = (int)ori % 360;

		return ori;//If safe, point the nose at the target
	}


	vector<int> mesh;
	for (int i = 0; i < map_cv.size(); i++) //Determine the CV value in the grid. There are 12 grids in total. The larger the CV value, the greater the possibility of obstacles
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
				cand_dir.push_back((j + 1)*sector_value);//Find a safe angle, in the body coordinate system; that is, determine the trough
		}
	}

	if (cand_dir.size() != 0) {
		vector<float> delta;
		for (auto &dir_ite : cand_dir) {
			float delte_theta1 = fabs(dir_ite - goal_ori);
			float delte_theta2 = 360 - delte_theta1;
			float delte_theta = delte_theta1 < delte_theta2 ? delte_theta1 : delte_theta2;
			delta.push_back(delte_theta);
		}//Look for troughs close to the target area
		int min_index = min_element(delta.begin(), delta.end()) - delta.begin();
		ori = cand_dir.at(min_index);

		ori += heading;
		ori = (int)ori % 360;

		return ori;//Determine the direction of the nose
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
	//LiDAR data callback function
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
    ros::init(argc, argv, "collision_avoidance_vfh");
    ros::NodeHandle nh("~");
	scan_distance_max = 2.1;
	scan_distance_min = 0.1;
	angle_resolution = 1.0;
    heading = 90;
	sector_value = 30;
	sector_scale = 10;
	Uavp.x = 0;
	Uavp.y = 0;

    ros::Subscriber gps_sub = nh.subscribe("/mavros/global_position/global",100,gps_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, local_pos_cb);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/lidar2Dscan", 100, scan_cb);
    ros::Subscriber waypoint_sub = nh.subscribe<mavros_msgs::WaypointList>("/mavros/mission/waypoints", 100, waypoints_cb);
    ros::Subscriber homePos_sub = nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 100, home_pos_cb);
    ros::Subscriber head_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 100, heading_cb);

ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    // frequency [20Hz]
    ros::Rate rate(20.0);
	while (ros::ok() && !current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
    printf("Please set the waypoint in QGC before running this program.\n");
	printf("wait a moment\n");
	while (ros::ok())
	{
		//7F ‭0111 1111‬
		if (init_mask == 0x7f)
			break;

		ros::spinOnce();
		rate.sleep();
	}
	printf("init ok!\n");

while (ros::ok())
	{
		while (ros::ok() && !current_state.guided )
		{
			ros::spinOnce();
			rate.sleep();
		}
		printf("guild ok\n");
		//**********************************************pose vextor container***********************************************
		std::vector<geometry_msgs::PoseStamped> pose;
//		printf("wp size=%d\n", waypoints.waypoints.size());
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
			printf("%f %f\n", testv[0], testv[1]);
			pose.push_back(p);
		}

		for (int i = 0; i < pose.size(); i++)
		{
			while (ros::ok()) {
				local_pos_updated = false;
				scan_updated = false;
				heading_updated = false;

				while (ros::ok())
				{
					ros::spinOnce();//Wait for the subscription update to complete
					if (local_pos_updated && scan_updated && heading_updated)
						break;
					rate.sleep();
				}

				if (!current_state.guided)
					break;

				if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
					fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
				{
					break;//If the distance between the current position and the desired waypoint is less than 1m, the waypoint task will be exited.
				}

				Point2D goal;
				goal.x = pose[i].pose.position.x;//read desired position
				goal.y = pose[i].pose.position.y;
				float direction = CalculDirection(goal);//According to the distribution of obstacles and the desired position, the desired heading is obtained
				if (direction >= -0.5)
				{
					if (direction > 180)
					{
						direction -= 360;
					}
					float arc = 3.1415 / 180 * direction;

					mavros_msgs::PositionTarget pos_target;
					pos_target.coordinate_frame = 1;
					pos_target.type_mask = 1 + 2 + /*4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
					pos_target.velocity.x = 0.5* cos(arc);
					pos_target.velocity.y = 0.5* sin(arc);
					pos_target.position.z = desire_z;
					local_pos_pub.publish(pos_target);

					ros::Time last_request = ros::Time::now();
					while (ros::ok()) {
						local_pos_updated = false;
						scan_updated = false;
						heading_updated = false;

						while (ros::ok())
						{
							ros::spinOnce();
							if (local_pos_updated && scan_updated && heading_updated)
								break;
							rate.sleep();
						}

						if (!current_state.guided)
							break;

						if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
							fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
						{
							break;
						}

						if (ros::Time::now() - last_request > ros::Duration(4.0))
							break;

						if (IsFrontSafety() == false)
							break;

						local_pos_pub.publish(pos_target);
					}
				}
				else
				{
					mavros_msgs::PositionTarget pos_target;
					pos_target.coordinate_frame = 1;
					pos_target.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
					pos_target.velocity.x = 0;
					pos_target.velocity.y = 0;
					pos_target.position.z = desire_z;
 					local_pos_pub.publish(pos_target);
				}
			}
		}

		mavros_msgs::PositionTarget pos_target;
		pos_target.coordinate_frame = 1;
		pos_target.type_mask = 1 + 2 + /*4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
		pos_target.velocity.x = 0;
		pos_target.velocity.y = 0;
		pos_target.position.z = desire_z;
		local_pos_pub.publish(pos_target);

		printf("task over\n");
		while (ros::ok() && current_state.guided)
		{
			ros::spinOnce();
			rate.sleep();
		}
	}

    return 0;
}
