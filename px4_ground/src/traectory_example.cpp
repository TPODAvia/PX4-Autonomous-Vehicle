#include <ros/ros.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_setpoint_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher trajectory_pub = nh.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 10);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20);

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::Trajectory trajectory_cmd;
    trajectory_cmd.header.frame_id = "map";
    trajectory_cmd.type = 0; // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        trajectory_cmd.header.stamp = ros::Time::now();
        
        // Set position, velocity, and acceleration components
        trajectory_cmd.point_1.position.x = 0.0;
        trajectory_cmd.point_1.position.y = 0.0;
        trajectory_cmd.point_1.position.z = 5.0;
        trajectory_cmd.point_1.velocity.x = 1.0;
        trajectory_cmd.point_1.velocity.y = 0.0;
        trajectory_cmd.point_1.velocity.z = 0.0;
        trajectory_cmd.point_1.acceleration_or_force.x = 0.0;
        trajectory_cmd.point_1.acceleration_or_force.y = 0.0;
        trajectory_cmd.point_1.acceleration_or_force.z = 0.0;

        trajectory_pub.publish(trajectory_cmd);

        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.base_mode = 0;
            offb_set_mode.request.custom_mode = "OFFBOARD";

            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
