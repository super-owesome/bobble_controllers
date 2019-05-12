///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, S.O. Engineering, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of S.O. Engineering, LLC nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Mike Moore

#include <cmath>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <bobble_controllers/ControlCommands.h>
#include <bobble_controllers/BobbleBotStatus.h>

// Floating-point value comparison threshold
const double VELOCITY_TOLERANCE = 0.05; // 5 cm-s precision

class BalanceSimControllerTest : public ::testing::Test
{
public:

  BalanceSimControllerTest()
  : mode_cmd_pub(nh.advertise<bobble_controllers::ControlCommands>("/bobble/bobble_balance_controller/bb_cmd", 1)),
    cmd_vel_pub(nh.advertise<geometry_msgs::Twist>("/bobble/bobble_balance_controller/cmd_vel", 1)),
    bobble_status_sub(nh.subscribe("/bobble/bobble_balance_controller/bb_controller_status", 1, &BalanceSimControllerTest::statusCallback, this))
  {
  }

  ~BalanceSimControllerTest()
  {
    bobble_status_sub.shutdown();
  }

  void SetUp(){
    waitForController();
    ros::Duration(0.25).sleep();
    ROS_INFO("Activating balance controller.");
    ros::Duration(0.25).sleep();
    activate();
  }

  void publish_vel_cmd(geometry_msgs::Twist cmd_vel){ cmd_vel_pub.publish(cmd_vel); }

  bobble_controllers::BobbleBotStatus getLastStatus(){ return last_status; }

  void activate()
  {
    bobble_controllers::ControlCommands mode_cmd;
    mode_cmd.StartupCmd = true;
    mode_cmd.IdleCmd = false;
    mode_cmd.DiagnosticCmd = false;
    mode_cmd_pub.publish(mode_cmd);
  }

  void reset()
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    publish_vel_cmd(cmd_vel);
    ros::Duration(0.25).sleep();
    bobble_controllers::ControlCommands mode_cmd;
    mode_cmd.IdleCmd = true;
    mode_cmd.StartupCmd = false;
    mode_cmd.DiagnosticCmd = false;
    mode_cmd_pub.publish(mode_cmd);
    ros::Duration(0.25).sleep();
  }

  bool isControllerAlive()const{ return (bobble_status_sub.getNumPublishers() > 0) && (cmd_vel_pub.getNumSubscribers() > 0); }

  void waitForController() const
  {
    while(!isControllerAlive() && ros::ok())
    {
      ROS_DEBUG_STREAM_THROTTLE(0.5, "Waiting for controller.");
      ros::Duration(0.1).sleep();
    }
    if (!ros::ok())
      FAIL() << "Something went wrong while executing test.";
  }

  void statusCallback(const bobble_controllers::BobbleBotStatus& status)
  {
    ROS_INFO_STREAM("Callback received: status.ForwardVelocity: " << status.ForwardVelocity
                     << ", status.Tilt: " << status.Tilt);
    last_status = status;
    received_first_status = true;
  }

  bool hasReceivedFirstStatus()const{ return received_first_status; }

  void waitForStatusMsgs() const
  {
    while(!hasReceivedFirstStatus() && ros::ok())
    {
      ROS_DEBUG_STREAM_THROTTLE(0.5, "Waiting for status messages to be published.");
      ros::Duration(0.01).sleep();
    }
    if (!ros::ok())
      FAIL() << "Something went wrong while executing test.";
  }

private:
  bool received_first_status;
  ros::NodeHandle nh;
  ros::Publisher mode_cmd_pub;
  ros::Publisher cmd_vel_pub;
  ros::Subscriber bobble_status_sub;
  bobble_controllers::BobbleBotStatus last_status;
};

