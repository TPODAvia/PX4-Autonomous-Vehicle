#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <pcl/segmentation/sac_segmentation.h>

class DepthImageConverter
{
public:
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg(*cloud, image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
    }
    image_pub_.publish(image_); //publish our cloud image
  }

  DepthImageConverter() : cloud_topic_("input"), image_topic_("output"), image_(0)
  {
    sub_ = nh_.subscribe(cloud_topic_, 30,
                           &DepthImageConverter::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    std::string r_ct = nh_.resolveName(cloud_topic_);
    std::string r_it = nh_.resolveName(image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }

  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_image_converter");
  DepthImageConverter converter; //this loads up the node
  ros::spin(); //where she stops nobody knows
  return 0;
}