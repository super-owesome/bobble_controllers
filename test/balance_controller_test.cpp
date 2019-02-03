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

#include "test_common.h"

// TEST CASES
TEST_F(BobbleBalanceControllerTest, testBalance)
{
  // wait for 3s. Is it holding balance?
  ros::Duration(3.0).sleep();
  bobble_controllers::BobbleBotStatus status = getLastStatus();
  EXPECT_LT(fabs(status.Tilt), 5.0); // we should be standing
}

TEST_F(BobbleBalanceControllerTest, testForward)
{
  // send a forward velocity command of 0.1 m/s
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.1;
  publish_vel_cmd(cmd_vel);
  // wait for 8s
  ros::Duration(8.0).sleep();
  bobble_controllers::BobbleBotStatus status = getLastStatus();
  const double expected_velocity = 0.1;
  EXPECT_NEAR(status.ForwardVelocity, expected_velocity, VELOCITY_TOLERANCE);
}

TEST_F(BobbleBalanceControllerTest, testBackward)
{
  // send a forward velocity command of -0.1 m/s
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = -0.1;
  publish_vel_cmd(cmd_vel);
  // wait for 8s
  ros::Duration(8.0).sleep();
  bobble_controllers::BobbleBotStatus status = getLastStatus();
  const double expected_velocity = -0.1;
  EXPECT_NEAR(status.ForwardVelocity, expected_velocity, VELOCITY_TOLERANCE);
}

TEST_F(BobbleBalanceControllerTest, testTurnLeft)
{
  // send a turn rate command of 0.1 rad/s
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = 0.1;
  publish_vel_cmd(cmd_vel);
  // wait for 3s
  ros::Duration(3.0).sleep();
  bobble_controllers::BobbleBotStatus status = getLastStatus();
  const double expected_turn_rate = 25.0;
  EXPECT_GT(status.TurnRate, expected_turn_rate); // should be turning at least 25 deg/s
}

TEST_F(BobbleBalanceControllerTest, testTurnRight)
{
  // send a turn rate command of -0.1 rad/s
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = -0.1;
  publish_vel_cmd(cmd_vel);
  // wait for 3s
  ros::Duration(3.0).sleep();
  bobble_controllers::BobbleBotStatus status = getLastStatus();
  const double expected_turn_rate = -25.0;
  EXPECT_LT(status.TurnRate, expected_turn_rate); // should be turning at least -25 deg/s
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "balance_controller_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
