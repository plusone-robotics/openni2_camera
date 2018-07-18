/*
 * Copyright (c) 2018, PlusOne Robotics
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
 *     * Neither the name of the Plus One Robotics, Inc. nor the names of its
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

#include <openni2_camera/GetSerial.h>
#include <ros/ros.h>

class CameraSerialFaker
{
public:
  CameraSerialFaker(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh)
  {
    // Get serial number from parameter server
    if (pnh_.getParam("serial", serial_) && !serial_.empty())
    {
      get_serial_service_ = nh_.advertiseService("get_serial", &CameraSerialFaker::getSerial, this);
    }
  }

  ~CameraSerialFaker() { }

  bool getSerial(openni2_camera::GetSerial::Request &request,
    openni2_camera::GetSerial::Response &response)
  {
    response.serial = serial_;
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceServer get_serial_service_;
  std::string serial_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_serial_faker");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  CameraSerialFaker serial_faker(nh, pnh);
  ros::spin();

  return 0;
}