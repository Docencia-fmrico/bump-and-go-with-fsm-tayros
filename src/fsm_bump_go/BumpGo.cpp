// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fsm_bump_go/BumpGo.h"

#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_bump_go
{

BumpGo::BumpGo()
: state_(GOING_FORWARD),
  pressed_(false)
{
  // sub_bumber_ = n_.subscribe(...);
  // pub_vel_ = n_.advertise<...>(...)
}

void
BumpGo::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  // pressed_ = (...);
  //  ...
}

void
BumpGo::step()
{
  geometry_msgs::Twist cmd;

  switch (state_)
  {
    case GOING_FORWARD:
      // cmd.linear.x = ...;
      // cmd.angular.z = ...;

      if (pressed_)ç
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }

      break;
    case GOING_BACK:
      // cmd.linear.x = ...;
      // cmd.angular.z = ...;

      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME )
      {
        turn_ts_ = ros::Time::now();
        state_ = TURNING;
        ROS_INFO("GOING_BACK -> TURNING");
      }

      break;
    case TURNING:
      // cmd.linear.x = ...;
      // cmd.angular.z = ...;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }

    // pub_vel_.publish(...);
}

}  // namespace fsm_bump_go
