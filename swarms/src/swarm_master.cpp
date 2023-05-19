#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <map>
#include <tuple>

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_master");
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::TransformBroadcaster tf_broadcaster;

  int n = 8; // Number of broadcasters

  std::vector<geometry_msgs::TransformStamped> transforms(n);

  for (int i = 0; i < n; ++i)
  {

    std::pair<int, int> loc;
    size_t list_size;
    std::tie(loc, list_size) = get_loc(i);

    // Set the parent frame for the transform to "camera_link"
    transforms[i].header.frame_id = "swarm_master";

    // Set the child frame for the transform
    transforms[i].child_frame_id = "offset_" + std::to_string(i + 1);

    // Set the translation offsets for the transform
    transforms[i].transform.translation.x = loc.first;
    transforms[i].transform.translation.y = loc.second;
    transforms[i].transform.translation.z = 0.0;

    // Set the rotation to identity (no rotation) for the transform
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);

    transforms[i].transform.rotation.x = quat.x();
    transforms[i].transform.rotation.y = quat.y();
    transforms[i].transform.rotation.z = quat.z();
    transforms[i].transform.rotation.w = quat.w();
  }



  // Set the rate for the transform publishing loop



  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "base_link";
  transform_stamped.child_frame_id = "swarm_master";
  transform_stamped.transform.translation.x = 1.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;

  tf2::Quaternion start_quat, end_quat;
  start_quat.setRPY(0, 0, 0);
  end_quat.setRPY(1.0, 0.5, 0.3);
  float t = 0.0;

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    ros::Time now = ros::Time::now();

    // Interpolate between start_quat and end_quat based on t
    tf2::Quaternion interpolated_quat;
    interpolated_quat = start_quat.slerp(end_quat, t);

    // Update the transform_stamped rotation with the interpolated quaternion
    transform_stamped.header.stamp = now;
    transform_stamped.transform.rotation.x = interpolated_quat.x();
    transform_stamped.transform.rotation.y = interpolated_quat.y();
    transform_stamped.transform.rotation.z = interpolated_quat.z();
    transform_stamped.transform.rotation.w = interpolated_quat.w();
    tf_broadcaster.sendTransform(transform_stamped);


    for (int i = 0; i < n; ++i)
    {
      transforms[i].header.stamp = now;
      tf_broadcaster.sendTransform(transforms[i]);
    }

    // Add any other processing you need to do here

    ros::spinOnce();
    rate.sleep();

    // Update t for the next iteration (you can adjust the increment value for faster or slower interpolation)
    t += 0.01;
    if (t > 1.0) 
    {
      t = 0.0;
      start_quat.setRPY(0, 0, 0);
    }
  }

  return 0;
}