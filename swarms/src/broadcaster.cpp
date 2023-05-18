#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher_subscriber_node");
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "base_link";
  transform_stamped.child_frame_id = "camera_link";
  transform_stamped.transform.translation.x = 1.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  transform_stamped.transform.rotation.x = quat.x();
  transform_stamped.transform.rotation.y = quat.y();
  transform_stamped.transform.rotation.z = quat.z();
  transform_stamped.transform.rotation.w = quat.w();
  ros::Rate rate(10.0);
  while (nh.ok())
  {
    transform_stamped.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(transform_stamped);

    // Add any other processing you need to do here

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}