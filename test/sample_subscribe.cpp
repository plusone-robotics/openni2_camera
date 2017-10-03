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
 *      Author: Julius Kammerl (jkammerl@willowgarage.com)
 */

#include <sstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>

class SampleSubscribe
{
public:

  void colorImage(const sensor_msgs::ImageConstPtr& color_image)
  {
    ROS_INFO_STREAM("color_image timestamp: " << color_image->header.stamp);
  }
  void updateInputData(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                              const sensor_msgs::ImageConstPtr& image)
  {
    ros::Time timestamp_before = ros::Time::now();
    //ROS_INFO_STREAM("Timestamp before PCL-ROS conversion: " << )timestamp_stamp;

    // Transform if we need to, convert to PCL type regardless of if we're transforming
    std_msgs::Header ros_cloud_header;
    pcl_conversions::fromPCL(cloud->header, ros_cloud_header);

    // Turned out timestamp in ROS is the same with the one in PCL since it's just converted.
    //ROS_INFO_STREAM("Timestamp PCL: " << cloud->header.stamp);
    ROS_INFO_STREAM("Timestamp diff = " << timestamp_before - ros_cloud_header.stamp);
  }

//  void subscribe_nodehandle(int argc, char **argv)
//  {
//    ros::init(argc, argv, "openni2_subscriber");
//    ros::NodeHandle n;
//    ros::Subscriber point_cloud_sub_;
//    point_cloud_sub_ = n.subscribe(
//        "/camera/depth/points", 1, &updateInputData);
//
//    ros::spin();
//  }

  void subscribe_synchronizing(int argc, char **argv)
  {
    ros::init(argc, argv, "openni2_subscriber_sync");
    ros::NodeHandle nh;
    typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::Image> PolicyType;
    typedef message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > PointCloudSubscriberType;
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriberType;
    typedef message_filters::Synchronizer<PolicyType> SynchronizerType;
    boost::shared_ptr<PointCloudSubscriberType> sync_point_cloud_sub_;
    boost::shared_ptr<ImageSubscriberType> image_sub_;
    boost::shared_ptr<SynchronizerType> synchronizer_;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber color_image_sub_;

    sync_point_cloud_sub_ = boost::shared_ptr<PointCloudSubscriberType>(
        new PointCloudSubscriberType(nh, "/camera/depth/points", 1));
    image_sub_ = boost::shared_ptr<ImageSubscriberType>(
        new ImageSubscriberType(nh, "/camera/rgb/image_raw", 1));
    synchronizer_ = boost::shared_ptr<SynchronizerType>(
        new SynchronizerType(PolicyType(10), sync_point_cloud_sub_, image_sub_));
    synchronizer_->registerCallback(boost::bind(
        &SampleSubscribe::updateInputData, this, _1, _2));
    color_image_sub_ = nh.subscribe(
        "/camera/rgb/image_raw/compressed", 1, &SampleSubscribe::colorImage, this);
  }

  /*
   * Only with nodelet. No syncronizer.
   */
  void subscribe_nodelet(int argc, char **argv)
  {
    ros::init(argc, argv, "openni2_subscriber_nosync_nodelet");
    ros::NodeHandle nh;
    typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::Image> PolicyType;
    typedef message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > PointCloudSubscriberType;
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriberType;
    typedef message_filters::Synchronizer<PolicyType> SynchronizerType;
    boost::shared_ptr<PointCloudSubscriberType> sync_point_cloud_sub_;
    boost::shared_ptr<ImageSubscriberType> image_sub_;
    boost::shared_ptr<SynchronizerType> synchronizer_;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber color_image_sub_;

    sync_point_cloud_sub_ = boost::shared_ptr<PointCloudSubscriberType>(
        new PointCloudSubscriberType(nh, "/camera/depth/points", 1));
    image_sub_ = boost::shared_ptr<ImageSubscriberType>(
        new ImageSubscriberType(nh, "/camera/rgb/image_raw", 1));
    synchronizer_ = boost::shared_ptr<SynchronizerType>(
        new SynchronizerType(PolicyType(10), sync_point_cloud_sub_, image_sub_));
    synchronizer_->registerCallback(boost::bind(
        &SampleSubscribe::updateInputData, this, _1, _2));
    color_image_sub_ = nh.subscribe(
        "/camera/rgb/image_raw/compressed", 1, &SampleSubscribe::colorImage, this);
  }
};

int main(int argc, char **argv)
{
  SampleSubscribe ssbsc = SampleSubscribe();
  //ssbsc.subscribe_nodehandle(argc, argv);
  ssbsc.subscribe_synchronizing(argc, argv);
}
