/***************************************************************************************************************************
*
* Author: TPODAvia
* Time: 2023.05.10
* Description: Implement px4 quadrotor Aruco tracking
***************************************************************************************************************************/
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


geometry_msgs::PoseStamped setpoint_pub;
geometry_msgs::TransformStamped transform_stamped;
geometry_msgs::PoseStamped pose_stamped;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped drone_pose;
void local_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("Position: (%f, %f, %f), Orientation: (%f, %f, %f, %f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    drone_pose = *msg;
}

bool tf_callback(const tf2_ros::Buffer &buffer, ros::Publisher &setpoint_pub, std::string aruco_id)
{
    try
    {
        transform_stamped = buffer.lookupTransform("map", aruco_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_ONCE("%s", ex.what());
        pose_stamped.pose.position.x = 0;
        pose_stamped.pose.position.y = 0;
        pose_stamped.pose.position.z = 10;
        setpoint_pub.publish(pose_stamped);
        return false;
    }

    // Perform manipulations on the transform_stamped
    pose_stamped.header = transform_stamped.header;
    pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
    pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
    pose_stamped.pose.position.z = -transform_stamped.transform.translation.z + 4.0;
    pose_stamped.pose.orientation = transform_stamped.transform.rotation;

    // Publish the result to the mavros/setpoint_position topic
    setpoint_pub.publish(pose_stamped);

    float drone_pose_x = drone_pose.pose.position.x - pose_stamped.pose.position.x;
    float drone_pose_y = drone_pose.pose.position.y - pose_stamped.pose.position.y;
    float drone_pose_z = drone_pose.pose.position.z - pose_stamped.pose.position.z;
    // drone_pose_x*drone_pose_x + drone_pose.pose.position.y^2 + drone_pose.pose.position.z^2 < 0.1^2;
    float abs = drone_pose_x*drone_pose_x+drone_pose_y*drone_pose_y+drone_pose_z*drone_pose_z;
    if (abs < 0.2*0.2)
    {
        return true;
    }
    return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_aruco_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);
    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
     ros::Subscriber sub = nh.subscribe("local_position/pose", 10, local_position_callback);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    if(current_state.mode != "AUTO.TAKEOFF")
    {
        offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
        set_mode_client.call(offb_set_mode);
        std::cout << "Setting to TAKEOFF Mode..." <<std::endl;

    }


    pose_stamped.pose.position.x = 0;
    pose_stamped.pose.position.y = 0;
    pose_stamped.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        setpoint_pub.publish(pose_stamped);
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "OFFBOARD";

    bool trigger = false;
    int land_counter = 0;
    while (nh.ok())
    {

        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                trigger = true;
            }
            last_request = ros::Time::now();
        }
        else
        { trigger = true; }

        if( current_state.mode != "OFFBOARD" && trigger &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 

        std::string aruco_id = "aruco_71";
        // std::string aruco_id = "aruco_72";
        // std::string aruco_id = "aruco_73";
        // Some function that uses the transform_stamped to perform some action
        bool destination_state = tf_callback(buffer, setpoint_pub, aruco_id);

        if (destination_state)
        {
            // ROS_INFO("Tracking Aruco Done");
            land_counter++;
            if (land_counter > 200)
            {
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                set_mode_client.call(offb_set_mode);
                std::cout << "Setting to LAND Mode..." <<std::endl;
                if (current_state.mode == "AUTO.LAND")
                {
                    ROS_INFO("Landing...");
                    break;
                }
            }
        }
        else
        {
            // ROS_INFO("Aruco %s is not detected", aruco_id.c_str());
            land_counter = 0;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}