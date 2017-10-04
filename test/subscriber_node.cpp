/*
 * Copyright (c) 2017, PlusOne Robotics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the PlusOne Robotics, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <boost/thread/mutex.hpp>                             // For locking

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "opencv2/video/background_segm.hpp"
#include "openni2_camera/openni2_driver.h"
#include <pcl_conversions/pcl_conversions.h>                  // Needed for toROSMsg and fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>                               //
#include <ros/ros.h>                                          // Core ROS functionality
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>                          // ROS point cloud message type
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>                           // Needed for tf::Transform data type
#include <tf/transform_listener.h>                            // Transform listener
#include <visualization_msgs/InteractiveMarker.h>

class OpenNI2SubscriberNode
{
public:
  OpenNI2SubscriberNode()
  {
  }
  ;
  ~OpenNI2SubscriberNode()
  {
  }
  void init(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

private:
  typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::Image> PolicyType;
  typedef message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > PointCloudSubscriberType;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriberType;
  typedef message_filters::Synchronizer<PolicyType> SynchronizerType;
  boost::shared_ptr<PointCloudSubscriberType> sync_point_cloud_sub_;
  boost::shared_ptr<ImageSubscriberType> image_sub_;
  boost::shared_ptr<SynchronizerType> synchronizer_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber color_image_sub_;

  void updateInputData(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const sensor_msgs::ImageConstPtr& img);
};
// class definition ends.

void OpenNI2SubscriberNode::updateInputData(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
                                            const sensor_msgs::ImageConstPtr& img)
{
  ros::Time timestamp_now = ros::Time::now();

  std_msgs::Header ros_cloud_header;
  // Convert PCL type to ROS, because ros::Time is used over the entire
  // PickOne system.
  pcl_conversions::fromPCL(input->header, ros_cloud_header);
  ROS_INFO_STREAM("Diff now - PCL: " << timestamp_now - ros_cloud_header.stamp);
  ROS_INFO_STREAM("Diff now - Img: " << timestamp_now - img->header.stamp);
}

// Not a class  member function.
void updateInputData(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const sensor_msgs::ImageConstPtr& img
//const sensor_msgs::PointCloudConstPtr& input, const sensor_msgs::ImageConstPtr& img
//const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& img2
    )
{
  ros::Time timestamp_now = ros::Time::now();

  std_msgs::Header ros_cloud_header;
  // Convert PCL type to ROS, because ros::Time is used over the entire
  // PickOne system.
  pcl_conversions::fromPCL(input->header, ros_cloud_header);
  ROS_INFO_STREAM("Diff now - PCL: " << timestamp_now - ros_cloud_header.stamp);
  ROS_INFO_STREAM("Diff now - Img: " << timestamp_now - img->header.stamp);
}

//void OpenNI2SubscriberNode::init(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
//{
//  //sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1,
//  //                                               boost::bind(&OpenNI2SubscriberNode::callback, this, _1));
//  //    sub_ = nh_.subscribe<PointCloud>("points2", 1,
//  //                                     boost::bind(&OpenNI2SubscriberNode::cb_pcl, this, _1));
//
//  this->sync_point_cloud_sub_ = boost::shared_ptr<PointCloudSubscriberType>(
//      new PointCloudSubscriberType(nh, "points", 1));
//  this->image_sub_ = boost::shared_ptr<ImageSubscriberType>(new ImageSubscriberType(nh, "image", 1));
//  this->synchronizer_ = boost::shared_ptr<SynchronizerType>(
//      new SynchronizerType(PolicyType(10), this->sync_point_cloud_sub_, this->image_sub_));
//  this->synchronizer_->registerCallback(boost::bind(&OpenNI2SubscriberNode::updateInputData, this, _1, _2));
//
//  //sub_ = nh_.subscribe<PointCloud>(
//  //    "points2", 1, boost::bind(&OpenNI2SubscriberNode::cb_pcl_timesync2, this, _1, _2));
//}
//;

int main(int argc, char **argv)
{
  //OpenNI2SubscriberNode ssbsc;

  typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::Image> PolicyType;
  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> PolicyType;
  //typedef message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > PointCloudSubscriberType;
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudSubscriberType;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriberType;
  typedef message_filters::Synchronizer<PolicyType> SynchronizerType;

  ros::init(argc, argv, "openni2_subscriber_sync_non_nodelet");
  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle priv_nh = ros::NodeHandle();

  //ros::Subscriber point_cloud_sub_;
  //ros::Subscriber color_image_sub_;
//  boost::shared_ptr<PointCloudSubscriberType> sync_point_cloud_sub_;
//  boost::shared_ptr<ImageSubscriberType> image_sub_;
//  boost::shared_ptr<ImageSubscriberType> image_sub2_;
  //boost::shared_ptr<PointCloudSubscriberType> sync_point_cloud_sub_;
  message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > sync_point_cloud_sub_(nh, "/camera/depth/points", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub_(nh, "/camera/rgb/image_raw", 1);
  //message_filters::Subscriber<sensor_msgs::Image> image_sub2_(nh, "image2", 1);

  boost::shared_ptr<SynchronizerType> synchronizer_;

  //ssbsc.init(nh, priv_nh);

  //sync_point_cloud_sub_ = boost::shared_ptr<PointCloudSubscriberType>(new PointCloudSubscriberType(nh, "points", 1));
  //image_sub_ = boost::shared_ptr<ImageSubscriberType>(new ImageSubscriberType(nh, "image", 1));
  //image_sub2_ = boost::shared_ptr<ImageSubscriberType>(new ImageSubscriberType(nh, "image_2", 1));
  synchronizer_ = boost::shared_ptr<SynchronizerType>(new SynchronizerType(PolicyType(10), sync_point_cloud_sub_,
  //image_sub2_,
                                                                           image_sub_));
  synchronizer_->registerCallback(boost::bind(&updateInputData, _1, _2));
  ros::spin();
}
