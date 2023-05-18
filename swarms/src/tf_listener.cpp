#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener_node");
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  std::string source_frame = "base_link";
  std::string target_frame = "camera_link";
  ros::Rate rate(10.0);
  while (nh.ok())
  {
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer.lookupTransform(source_frame, target_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    ROS_INFO("Translation: x = %f, y = %f, z = %f",
             transform_stamped.transform.translation.x,
             transform_stamped.transform.translation.y,
             transform_stamped.transform.translation.z);

    ROS_INFO("Rotation: x = %f, y = %f, z = %f, w = %f",
             transform_stamped.transform.rotation.x,
             transform_stamped.transform.rotation.y,
             transform_stamped.transform.rotation.z,
             transform_stamped.transform.rotation.w);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
