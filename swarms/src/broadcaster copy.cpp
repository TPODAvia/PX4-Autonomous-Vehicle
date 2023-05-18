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
  
  // Create two TransformStamped objects for the two new transforms
  geometry_msgs::TransformStamped transform_stamped1, transform_stamped2;

  // Set the parent frame for both transforms to "camera_link"
  transform_stamped1.header.frame_id = "camera_link";
  transform_stamped2.header.frame_id = "camera_link";

  // Set the child frames for the two new transforms
  transform_stamped1.child_frame_id = "offset_x";
  transform_stamped2.child_frame_id = "offset_y";

  // Set the translation offsets for both transforms
  transform_stamped1.transform.translation.x = 1.0;
  transform_stamped1.transform.translation.y = 0.0;
  transform_stamped1.transform.translation.z = 0.0;

  transform_stamped2.transform.translation.x = 0.0;
  transform_stamped2.transform.translation.y = 1.0;
  transform_stamped2.transform.translation.z = 0.0;

  // Set the rotation to identity (no rotation) for both transforms
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);

  transform_stamped1.transform.rotation.x = quat.x();
  transform_stamped1.transform.rotation.y = quat.y();
  transform_stamped1.transform.rotation.z = quat.z();
  transform_stamped1.transform.rotation.w = quat.w();

  transform_stamped2.transform.rotation.x = quat.x();
  transform_stamped2.transform.rotation.y = quat.y();
  transform_stamped2.transform.rotation.z = quat.z();
  transform_stamped2.transform.rotation.w = quat.w();

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    // Update the timestamp for both transforms
    ros::Time now = ros::Time::now();
    transform_stamped1.header.stamp = now;
    transform_stamped2.header.stamp = now;

    // Broadcast both transforms
    tf_broadcaster.sendTransform(transform_stamped1);
    tf_broadcaster.sendTransform(transform_stamped2);

    // Add any other processing you need to do here

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}