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

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "openni2_camera/openni2_driver.h"

namespace openni2_camera
{

class OpenNI2SubscriberNodelet : public nodelet::Nodelet
{
public:
  OpenNI2SubscriberNodelet()
  {};
  ~OpenNI2SubscriberNodelet()
  {}

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  boost::shared_ptr<openni2_wrapper::OpenNI2Driver> lp;

  void onInit()
  {
    nh_ = getNodeHandle();
    //lp.reset(new openni2_wrapper::OpenNI2Driver(nh_, getPrivateNodeHandle()));
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/camera/depth/points",
        1,
        boost::bind(
            &OpenNI2SubscriberNodelet::callback,
            this, _1)
    );
  };

  void callback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    ROS_INFO_STREAM("PCL timestamp: " << input->header.stamp);
  }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(openni2_camera, OpenNI2SubscriberNodelet, openni2_camera::OpenNI2SubscriberNodelet, nodelet::Nodelet);
