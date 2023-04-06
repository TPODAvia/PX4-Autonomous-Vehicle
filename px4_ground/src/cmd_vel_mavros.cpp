/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 * 
 * Body frame seems bugged. There is no way to control speed in BODY_NED.
 * So my plan is to control dirrectry by RC_override. But people mentioned it's not recomended way.
 * 
 * To reproduce this bug run:
 * 1) cd PX4-Autopilot
 * 2) In PX4-Autopilot/launch/mavros_posix_sitl.launch change vehicle to rover
 * 3) roslaunch px4 mavros_posix_sitl.launch
 * 4) In QGC make sure that parameter is set COM_RCL_EXCEPT 4
 * 5) rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8" # set to BODY_NED
 * 6) rostopic pub -r 10 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear:
        x: 0.5
        y: 0.0
        z: 0.01
        angular:
        x: 0.0
        y: 0.0
        z: 0.1"
 * 7) In QGC arm vehicle first then switch to Offboard mode
 * The multicopter behaves normally however the rover are going forward in the y direction
 *
 *
 * srv file:
 * uint8 FRAME_GLOBAL=0
 * uint8 FRAME_LOCAL_NED=1
 * uint8 FRAME_MISSION=2
 * uint8 FRAME_GLOBAL_RELATIVE_ALT=3
 * uint8 FRAME_LOCAL_ENU=4
 * uint8 FRAME_GLOBAL_INT=5
 * uint8 FRAME_GLOBAL_RELATIVE_ALT_INT=6
 * uint8 FRAME_LOCAL_OFFSET_NED=7
 * uint8 FRAME_BODY_NED=8
 * uint8 FRAME_BODY_OFFSET_NED=9
 * uint8 FRAME_GLOBAL_TERRAIN_ALT=10
 * uint8 FRAME_GLOBAL_TERRAIN_ALT_INT=11
 * uint8 FRAME_BODY_FRD=12
 * uint8 FRAME_RESERVED_13=13
 * uint8 FRAME_RESERVED_14=14
 * uint8 FRAME_RESERVED_15=15
 * uint8 FRAME_RESERVED_16=16
 * uint8 FRAME_RESERVED_17=17
 * uint8 FRAME_RESERVED_18=18
 * uint8 FRAME_RESERVED_19=19
 * uint8 FRAME_LOCAL_FRD=20
 * uint8 FRAME_LOCAL_FLU=21
 * uint8 mav_frame

 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/OverrideRCIn.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//Receive linear and angular speeds from /cmd_vel topic
float linX, angZ;
void VelocityCallback(const geometry_msgs::Twist& msg2){
      linX = msg2.linear.x;
      angZ = msg2.angular.z;
}

mavros_msgs::SetMavFrame set_mav_frm;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_mavros_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel", 1000, &VelocityCallback);

    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::Publisher abababab_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("/mavros/rc/override", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    ros::ServiceClient mav_frame_client = nh.serviceClient<mavros_msgs::SetMavFrame>("/mavros/setpoint_velocity/mav_frame");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    // This is expiremtal for publishing velocity to fmu but it didn't work
    // geometry_msgs::TwistStamped vel;
    // vel.twist.linear.x = 0;
    // vel.twist.linear.y = 0;
    // vel.twist.linear.z = 0;
    // vel.twist.angular.x = 0;
    // vel.twist.angular.y = 0;
    // vel.twist.angular.z = 0;

    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_vel_pub.publish(vel);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "MANUAL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    // set_mav_frm.request.FRAME_BODY_NED;
    // std::cout << "Set frame id to BODY_NED: " << set_mav_frm.request.FRAME_BODY_NED <<std::endl;
    // mav_frame_client.call(set_mav_frm);

    mavros_msgs::OverrideRCIn rc_pub;

    while(ros::ok()){

        // Switch to MANUAL and arming the vehicle
        if( current_state.mode != "MANUAL" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("MANUAL enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //Publishing rc override

        rc_pub.channels[0] = 1500 - angZ*500; // Steeering wheel
        rc_pub.channels[1] = 1000 + linX*2000; // Throttle
        rc_pub.channels[2] = 1500;
        rc_pub.channels[3] = 1500;
        rc_pub.channels[4] = 1500;
        rc_pub.channels[5] = 1500;
        rc_pub.channels[6] = 1500;

        std::cout << "Parameter is:  linX = " << linX << " and angZ = " << angZ << std::endl;


        abababab_pub.publish(rc_pub);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}