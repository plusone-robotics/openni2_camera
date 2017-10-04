// http://www.clearpathrobotics.com/assets/guides/ros/Nodelet%20Everything.html

#include "ros/ros.h"
#include "nodelet/loader.h"
#include "nodelet/nodelet.h"
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

//<pcl::PointCloud<pcl::PointXYZ
void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  ROS_INFO_STREAM("PCL timestamp: " << input->header.stamp);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_pcl_subscriber");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(
      nodelet_name, "openni2_camera/OpenNI2SubscriberNodelet", remap, nargv);
  ros::spin();
  return 0;
}
