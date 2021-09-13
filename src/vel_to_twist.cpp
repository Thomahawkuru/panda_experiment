/*******************************************************************************
 *      Title     : Vel_to_twist.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, Thomas de Boer
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "control_msgs/JointJog.h"
#include "ros/ros.h"

namespace moveit_servo
{
static const int NUM_SPINNERS = 1;
static const int QUEUE_LENGTH = 1;

  class VelToTwist{
    
  public:
    VelToTwist() : spinner_(NUM_SPINNERS)
    {
      Vel_sub_ = n_.subscribe("/Unity/HandVelCommands", QUEUE_LENGTH, &VelToTwist::VelCallback, this);
      twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("servo_server/delta_twist_cmds", QUEUE_LENGTH);
      joint_delta_pub_ = n_.advertise<control_msgs::JointJog>("servo_server/delta_joint_cmds", QUEUE_LENGTH);

      spinner_.start();
      ros::waitForShutdown();
    };

  private:
    // Convert incoming Vel commands to TwistStamped commands for servoing.
    void VelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
      // Cartesian servoing with the axes
      geometry_msgs::TwistStamped twiststamped;
      twiststamped.header.stamp = ros::Time::now();
      twiststamped.twist.linear.x = msg->twist.linear.z;
      twiststamped.twist.linear.y = msg->twist.linear.x;
      twiststamped.twist.linear.z = msg->twist.linear.y;

      twiststamped.twist.angular.x = msg->twist.angular.z;
      twiststamped.twist.angular.y = msg->twist.angular.x;
      twiststamped.twist.angular.z = msg->twist.angular.y;

      twist_pub_.publish(twiststamped);
    }

    ros::NodeHandle n_;
    ros::Subscriber Vel_sub_;
    ros::Publisher twist_pub_, joint_delta_pub_;
    ros::AsyncSpinner spinner_;
  };
}  // namespace moveit_servo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Vel_to_twist");
  moveit_servo::VelToTwist to_twist;

  return 0;
}