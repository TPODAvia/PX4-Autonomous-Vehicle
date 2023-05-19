#include <gnc_functions.hpp>

mavros_msgs::State current_state_g;
nav_msgs::Odometry current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g;
int drone_id_g;
int n = 8; // Number of broadcasters

ros::Publisher local_pos_pub;
ros::Publisher global_lla_pos_pub;
ros::Publisher global_lla_pos_pub_raw;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
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

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_g = *msg;
}

geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu)
{
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  geometry_msgs::Point current_pos_local;
  current_pos_local.x = x*cos((local_offset_g - 90)*deg2rad) - y*sin((local_offset_g - 90)*deg2rad);
  current_pos_local.y = x*sin((local_offset_g - 90)*deg2rad) + y*cos((local_offset_g - 90)*deg2rad);
  current_pos_local.z = z;

  return current_pos_local;

  //ROS_INFO("Local position %f %f %f",X, Y, Z);
}

//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_g = *msg;
  enu_2_local(current_pose_g);
  float q0 = current_pose_g.pose.pose.orientation.w;
  float q1 = current_pose_g.pose.pose.orientation.x;
  float q2 = current_pose_g.pose.pose.orientation.y;
  float q3 = current_pose_g.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  //IS YAWING COUNTERCLOCKWISE POSITIVE?
  current_heading_g = psi*(180/M_PI) - local_offset_g;
  //ROS_INFO("Current Heading %f origin", current_heading_g);
  //ROS_INFO("x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
}

geometry_msgs::Point get_current_location()
{
	geometry_msgs::Point current_pos_local;
	current_pos_local = enu_2_local(current_pose_g);
	return current_pos_local;

}

float get_current_heading()
{
	return current_heading_g;
}


//set orientation of the drone (drone should always be level) 
// Heading input should match the ENU coordinate system
/**
\ingroup control_functions
This function is used to specify the drone’s heading in the local reference frame. Psi is a counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_heading(float heading)
{
  local_desired_heading_g = heading; 
  heading = heading + correction_heading_g + local_offset_g;
  
  ROS_INFO("Desired Heading %f ", local_desired_heading_g);
  float yaw = heading*(M_PI/180);
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  waypoint_g.pose.orientation.w = qw;
  waypoint_g.pose.orientation.x = qx;
  waypoint_g.pose.orientation.y = qy;
  waypoint_g.pose.orientation.z = qz;
}

std::tuple<std::pair<int, int>, size_t> get_loc(int input)
{
    std::map<int, std::pair<int, int>> list = 
    {
        {0, {1, 0}},
        {1, {-1, 0}},
        {2, {0, 1}},
        {3, {0, -1}},
        {4, {-1, 1}},
        {5, {1, 1}},
        {6, {-1, -1}},
        {7, {1, -1}}
        // Add more pairs as needed
    };
    
    std::pair<int, int> result = list[input];
    size_t list_size = list.size();  // Get the size of the list map
    if (list.find(input) != list.end()) {
        // std::cout << "Output: " << result.first << ", " << result.second << std::endl;
    } else {
        std::cout << "Number not found in the list." << std::endl;
    }

    return std::make_tuple(result, list_size);
}

// set position to fly to in the local frame
/**
\ingroup control_functions
This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_destination(float x, float y, float z, float psi)
{


	// get global offset angle
	// calculate rotation
	// wait until done
	// read the input transform
	// calculate the new position for all drones
	// publish the new position
	if (drone_id_g == 0)
	{


		set_heading(psi);
		//transform map to local
		float deg2rad = (M_PI/180);
		float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
		float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
		float Zlocal = z;

		x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x;
		y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y;
		z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z;
		ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

		waypoint_g.pose.position.x = x;
		waypoint_g.pose.position.y = y;
		waypoint_g.pose.position.z = z;

		local_pos_pub.publish(waypoint_g);

	}
	else
	{
		local_pos_pub.publish(waypoint_g);		
	}

}

/**
\ingroup control_functions
Wait for connect is a function that will hold the program until communication with the FCU is established.
@returns 0 - connected to fcu 
@returns -1 - failed to connect to drone
*/
int wait4connect()
{
	ROS_INFO("Waiting for FCU connection");
	// wait for FCU connection
	while (ros::ok() && !current_state_g.connected)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	if(current_state_g.connected)
	{
		ROS_INFO("Connected to FCU");	
		return 0;
	}else{
		ROS_INFO("Error connecting to drone");
		return -1;	
	}
	
	
}

/**
\ingroup control_functions
This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.
@returns 0 - frame initialized
*/
int initialize_local_frame()
{
	//set the orientation of the local reference frame
	ROS_INFO("Initializing local coordinate system");
	local_offset_g = 0;
	for (int i = 1; i <= 30; i++) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();

		

		float q0 = current_pose_g.pose.pose.orientation.w;
		float q1 = current_pose_g.pose.pose.orientation.x;
		float q2 = current_pose_g.pose.pose.orientation.y;
		float q3 = current_pose_g.pose.pose.orientation.z;
		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

		local_offset_g += psi*(180/M_PI);

		local_offset_pose_g.x = local_offset_pose_g.x + current_pose_g.pose.pose.position.x;
		local_offset_pose_g.y = local_offset_pose_g.y + current_pose_g.pose.pose.position.y;
		local_offset_pose_g.z = local_offset_pose_g.z + current_pose_g.pose.pose.position.z;

		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 5;
		local_pos_pub.publish(pose);
		// ROS_INFO("current heading%d: %f", i, local_offset_g/i);
	}
	local_offset_pose_g.x = local_offset_pose_g.x/30;
	local_offset_pose_g.y = local_offset_pose_g.y/30;
	local_offset_pose_g.z = local_offset_pose_g.z/30;
	local_offset_g /= 30;
	ROS_INFO("Coordinate offset set");
	ROS_INFO("the X' axis is facing: %f", local_offset_g);
	return 0;
}

/**
\ingroup control_functions
Wait for strat will hold the program until the user signals the FCU to enther mode OFFBOARD. This is typically done from a switch on the safety pilot’s remote or from the ground control station.
@returns 0 - mission started
@returns -1 - failed to start mission
*/
int wait4start()
{
    ros::Rate rate(20.0);

    mavros_msgs::SetMode offb_set_mode;
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	ros::Time last_request = ros::Time::now();
	offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
	ROS_INFO("Setting to TAKEOFF Mode...");

	while(ros::ok())
	{
		if(current_state_g.mode != "AUTO.TAKEOFF")
		{
			set_mode_client.call(offb_set_mode);
		}

		if (!current_state_g.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
			if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
				ROS_INFO("Vehicle armed");
				break;
			}
			last_request = ros::Time::now();
		}
		rate.sleep();
	}

	//intitialize first waypoint of mission
	set_destination(0,0,5,0);

	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint_g);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}

	// arming
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state_g.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint_g);
		if(arm_request.response.success)
		{
			ROS_INFO("Arming Successful");
		}
		else
		{
			ROS_INFO("Arming failed with %d", arm_request.response.success);
			// ros::shutdown();
		}
	}

	// ROS_INFO("Waiting for user to set mode to OFFBOARD");
	offb_set_mode.request.custom_mode = "OFFBOARD";
	while(ros::ok() && current_state_g.mode != "OFFBOARD")
	{

        if (current_state_g.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD mode enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state_g.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

		set_destination(0,0,5,0);
	    ros::spinOnce();
	    // ros::Duration(0.01).sleep();
		rate.sleep();
  	}

  	if(current_state_g.mode == "OFFBOARD")
	{
		ROS_INFO("Mode set to OFFBOARD. Mission starting");
		return 0;
	}else{
		ROS_INFO("Error starting mission!!");
		return -1;	
	}
}

/**
\ingroup control_functions
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. 
@return 1 - waypoint reached 
@return 0 - waypoint not reached
*/
int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01, tf2_ros::TransformBroadcaster &tf_broadcaster, std::vector<geometry_msgs::TransformStamped> &transforms, int n, ros::Publisher &local_pos_pub, geometry_msgs::PointStamped &waypoint_g)
{
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
	tf2_ros::TransformBroadcaster tf_broadcaster;
	std::vector<geometry_msgs::TransformStamped> transforms(n);

	ros::Time now = ros::Time::now();

	if (drone_id_g == 0)
	{
		ros::Time now = ros::Time::now();
		for (int i = 0; i < n; ++i)
		{
		transforms[i].header.stamp = now;
		tf_broadcaster.sendTransform(transforms[i]);
		}
		local_pos_pub.publish(waypoint_g);

		//check for correct position 
		float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x);
		float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y);
		float deltaZ = 0; //abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z);
		float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
		//check orientation
		float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
		float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
		float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );

		if( dMag < pos_tolerance && headingErr < heading_tolerance)
		{
			return 1;
		}else{
			return 0;
		}

	}
	else
	{
		std::string source_frame = "map";
  		std::string target_frame = "offset_" + std::to_string(drone_id_g);
		geometry_msgs::TransformStamped transform_stamped;
		try
		{
			transform_stamped = tf_buffer.lookupTransform(source_frame, target_frame, ros::Time(0));
		}
		catch (tf2::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		waypoint_g.pose.position.x = transform_stamped.transform.translation.x;
		waypoint_g.pose.position.y = transform_stamped.transform.translation.y;
		waypoint_g.pose.position.z = transform_stamped.transform.translation.z;

		waypoint_g.pose.orientation.w = transform_stamped.transform.rotation.w;
		waypoint_g.pose.orientation.x = transform_stamped.transform.rotation.x;
		waypoint_g.pose.orientation.y = transform_stamped.transform.rotation.y;
		waypoint_g.pose.orientation.z = transform_stamped.transform.rotation.z;

		local_pos_pub.publish(waypoint_g);
		return 0;
	}

}

/**
\ingroup control_functions
this function changes the mode of the drone to land
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
int land()
{
  mavros_msgs::CommandTOL srv_land;
  if(land_client.call(srv_land) && srv_land.response.success)
  {
    ROS_INFO("land sent %d", srv_land.response.success);
    return 0;
  }else{
    ROS_ERROR("Landing failed");
    return -1;
  }
}

void setupTransforms(std::vector<geometry_msgs::TransformStamped> &transforms, int n)
{

	if (drone_id_g == 0)
	{

		for (int i = 0; i < n; ++i)
		{
			std::pair<int, int> loc;
			size_t list_size;
			std::tie(loc, list_size) = get_loc(i);

			transforms[i].header.frame_id = "base_link";
			transforms[i].child_frame_id = "offset_" + std::to_string(i + 1);
			transforms[i].transform.translation.x = loc.first;
			transforms[i].transform.translation.y = loc.second;
			transforms[i].transform.translation.z = 0.0;

			tf2::Quaternion quat;
			quat.setRPY(0, 0, 0);

			transforms[i].transform.rotation.x = quat.x();
			transforms[i].transform.rotation.y = quat.y();
			transforms[i].transform.rotation.z = quat.z();
			transforms[i].transform.rotation.w = quat.w();
		}
	}
	else
	{
		std::string source_frame = "map";
  		std::string target_frame = "offset_" + std::to_string(drone_id_g);
		geometry_msgs::TransformStamped transform_stamped;
		try
		{
			transform_stamped = tf_buffer.lookupTransform(source_frame, target_frame, ros::Time(0));
		}
		catch (tf2::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		waypoint_g.pose.position.x = transform_stamped.transform.translation.x;
		waypoint_g.pose.position.y = transform_stamped.transform.translation.y;
		waypoint_g.pose.position.z = transform_stamped.transform.translation.z;

		waypoint_g.pose.orientation.w = transform_stamped.transform.rotation.w;
		waypoint_g.pose.orientation.x = transform_stamped.transform.rotation.x;
		waypoint_g.pose.orientation.y = transform_stamped.transform.rotation.y;
		waypoint_g.pose.orientation.z = transform_stamped.transform.rotation.z;
	}
}

/**
\ingroup control_functions
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input 
@returns n/a
*/
int init_publisher_subscriber(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	if (!controlnode.hasParam("namespace") || !controlnode.hasParam("drone_id_g"))
	{

		ROS_INFO("Please setup the namespace and drone_id_g parameters");
		ros::shutdown();

	}else{
		controlnode.getParam("namespace", ros_namespace);
		controlnode.getParam("drone_id_g", drone_id_g);
		ROS_INFO("using namespace %s %i", ros_namespace.c_str(), drone_id_g);
	}
	local_pos_pub = 					controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + std::to_string(drone_id_g) + "/mavros/setpoint_position/local").c_str(), 10);
	global_lla_pos_pub = 				controlnode.advertise<geographic_msgs::GeoPoseStamped>((ros_namespace + std::to_string(drone_id_g) + "/mavros/setpoint_position/global").c_str(), 10);
	global_lla_pos_pub_raw = 			controlnode.advertise<mavros_msgs::GlobalPositionTarget>((ros_namespace + std::to_string(drone_id_g) + "/mavros/setpoint_raw/global").c_str(), 10);

	currentPos = 						controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + std::to_string(drone_id_g) + "/mavros/global_position/local").c_str(), 10, pose_cb);
	state_sub = 						controlnode.subscribe<mavros_msgs::State>((ros_namespace + std::to_string(drone_id_g) + "/mavros/state").c_str(), 10, state_cb);

	arming_client = 					controlnode.serviceClient<mavros_msgs::CommandBool>((ros_namespace + std::to_string(drone_id_g) + "/mavros/cmd/arming").c_str());
	land_client = 						controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + std::to_string(drone_id_g) + "/mavros/cmd/land").c_str());
	set_mode_client = 					controlnode.serviceClient<mavros_msgs::SetMode>((ros_namespace + std::to_string(drone_id_g) + "/mavros/set_mode").c_str());
	takeoff_client = 					controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + std::to_string(drone_id_g) + "/mavros/cmd/takeoff").c_str());
	command_client = 					controlnode.serviceClient<mavros_msgs::CommandLong>((ros_namespace + std::to_string(drone_id_g) + "/mavros/cmd/command").c_str());
	auto_waypoint_pull_client = 		controlnode.serviceClient<mavros_msgs::WaypointPull>((ros_namespace + std::to_string(drone_id_g) + "/mavros/mission/pull").c_str());
	auto_waypoint_push_client = 		controlnode.serviceClient<mavros_msgs::WaypointPush>((ros_namespace + std::to_string(drone_id_g) + "/mavros/mission/push").c_str());
	auto_waypoint_set_current_client = 	controlnode.serviceClient<mavros_msgs::WaypointSetCurrent>((ros_namespace + std::to_string(drone_id_g) + "/mavros/mission/set_current").c_str());

	return 0;
}


int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
    int n = 5; // Number of transforms
    std::vector<geometry_msgs::TransformStamped> transforms(n);


	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//create local reference frame 
	// initialize_local_frame();

	//wait for used to switch to mode OFFBORAD
	wait4start();
	
	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);

	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);

	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);

	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);

	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);

	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3, tf_broadcaster, transforms, n, local_pos_pub, waypoint_g) == 1)
		{
			if (counter < waypointList.size())
			{
				// init function here!!!!!
				setupTransforms(transforms, n);
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				//land after all waypoints are reached
				land();
			}	
		}	
		
	}
	return 0;
}