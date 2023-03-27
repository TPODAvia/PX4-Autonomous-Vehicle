#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <std_msgs/String.h>

#include <px4_air/Execute.h>

ros::Duration timeout;

// TODO: handle timeout
bool handle(px4_air::Execute::Request& req, px4_air::Execute::Response& res)
{
	ROS_INFO("Execute: %s", req.cmd.c_str());

	std::array<char, 128> buffer;
	std::string result;

	FILE *fp = popen(req.cmd.c_str(), "r");

	if (fp == NULL) {
		res.code = px4_air::Execute::Request::CODE_FAIL;
		res.output = "popen() failed";
		return true;
	}

	while (fgets(buffer.data(), buffer.size(), fp) != nullptr) {
		res.output += buffer.data();
	}

	res.code = pclose(fp);
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "shell");
	ros::NodeHandle nh, nh_priv("~");

	timeout = ros::Duration(nh_priv.param("timeout", 3.0));

	auto gt_serv = nh.advertiseService("exec", &handle);

	ROS_INFO("shell: ready");
	ros::spin();
}
