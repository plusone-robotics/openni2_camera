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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "openni2_camera/openni2_driver.h"

namespace openni2_camera
{

class OpenNI2SubscriberNodelet : public nodelet::Nodelet
{
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  OpenNI2SubscriberNodelet()
  {
  }
  ;
  ~OpenNI2SubscriberNodelet()
  {
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  boost::shared_ptr<openni2_wrapper::OpenNI2Driver> lp;

  typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::Image> PolicyType;
  typedef message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > PointCloudSubscriberType;
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriberType;
  typedef message_filters::Synchronizer<PolicyType> SynchronizerType;
  boost::shared_ptr<PointCloudSubscriberType> sync_point_cloud_sub_;
  boost::shared_ptr<ImageSubscriberType> image_sub_;
  boost::shared_ptr<SynchronizerType> synchronizer_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber color_image_sub_;

  void onInit()
  {
    nh_ = getNodeHandle();
    //lp.reset(new openni2_wrapper::OpenNI2Driver(nh_, getPrivateNodeHandle()));

    //sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1,
    //                                               boost::bind(&OpenNI2SubscriberNodelet::callback, this, _1));
    //    sub_ = nh_.subscribe<PointCloud>("points2", 1,
    //                                     boost::bind(&OpenNI2SubscriberNodelet::cb_pcl, this, _1));

    sync_point_cloud_sub_ = boost::shared_ptr<PointCloudSubscriberType>(new PointCloudSubscriberType(nh_, "points", 1));
    image_sub_ = boost::shared_ptr<ImageSubscriberType>(new ImageSubscriberType(nh_, "image", 1));
    synchronizer_ = boost::shared_ptr<SynchronizerType>(new SynchronizerType(PolicyType(10),
      sync_point_cloud_sub_, image_sub_));
    synchronizer_->registerCallback(boost::bind(&OpenNI2SubscriberNodelet::cb_pcl_timesync2, this, _1, _2));

    //sub_ = nh_.subscribe<PointCloud>(
    //    "points2", 1, boost::bind(&OpenNI2SubscriberNodelet::cb_pcl_timesync2, this, _1, _2));
  }
  ;

  void callback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    ros::Time timestamp_now = ros::Time::now();
    ROS_INFO_STREAM("Diff now - sensor_msgs.PCL.timestamp: " << timestamp_now - input->header.stamp);
  }

  void cb_pcl_nosync(const PointCloud::ConstPtr& input)
  {
    ros::Time timestamp_now = ros::Time::now();
    std_msgs::Header ros_cloud_header;
    pcl_conversions::fromPCL(input->header, ros_cloud_header);

    ROS_INFO_STREAM("Diff now - PCL.timestamp: " << timestamp_now - ros_cloud_header.stamp);
  }

  /**
   * Callback for nodelet.
   */
//  void cb_pcl_timesync(const PointCloud::ConstPtr& input, const sensor_msgs::ImageConstPtr& img)
//  {
//    sensor_msgs::Image image;
//
//    message_filters::Subscriber<PointCloud> pcl_sub(nh_, "point2", 1);
//    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh_, "/kinect/rgb/image_raw", 1);
//
//    message_filters::TimeSynchronizer<PointCloud, sensor_msgs::Image> sync(pcl_sub, img_sub, 10);
//    sync.registerCallback(boost::bind(&OpenNI2SubscriberNodelet::cb_pcl_img, _1, _2));
//  }

  /**
   * Callback for nodelet.
   */
  void cb_pcl_timesync2(const PointCloud::ConstPtr& input, const sensor_msgs::ImageConstPtr& img)
  {

  }

  /**
   * Callback for time synchronizer.
   */
  void cb_pcl_img(const PointCloud::ConstPtr& input, const sensor_msgs::ImageConstPtr& img)
  {
    ros::Time timestamp_now = ros::Time::now();
    std_msgs::Header ros_cloud_header;
    pcl_conversions::fromPCL(input->header, ros_cloud_header);

    ROS_INFO_STREAM("Diff now - PCL.timestamp: " << timestamp_now - ros_cloud_header.stamp);
    ROS_INFO_STREAM("Diff now - Img.timestamp: " << timestamp_now - img->header.stamp);
  }

}; // class ends.
}// namespace ends.

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(openni2_camera, OpenNI2SubscriberNodelet, openni2_camera::OpenNI2SubscriberNodelet,
                        nodelet::Nodelet);
