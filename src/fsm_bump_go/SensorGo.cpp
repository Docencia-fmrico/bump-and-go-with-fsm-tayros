// Copyright 2022 TayRos
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

#include "fsm_bump_go/SensorGo.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace fsm_bump_go
{

SensorGo::SensorGo()
{
  state_ = GOING_FORWARD;
  pressed_ = false;
  turn_direction_ = TURN_LEFT;
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  pub_led_ = n_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
  pub_sound_ = n_.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
}

void
SensorGo::step()
{
  geometry_msgs::Twist cmd;
  kobuki_msgs::Led led_control;
  kobuki_msgs::Sound sound_control;

  switch (state_)
  {
    case GOING_FORWARD:
      cmd.linear.x = LINEAR_VELOCITY_X;
      cmd.angular.z = 0.0;
      if (TOGGLE_LED)
      {
        led_control.value = GREEN;
      }

      if (pressed_)
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }

      break;

    case GOING_BACK:
      cmd.linear.x = -LINEAR_VELOCITY_X;
      cmd.angular.z = 0.0;

      if (TOGGLE_LED)
      {
        led_control.value  = ORANGE;
      }

      if (TOGGLE_SOUND)
      {
        sound_control.value = SOUND_ERROR;
        pub_sound_.publish(sound_control);
      }

      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME)
      {
        turn_ts_ = ros::Time::now();

        if (turn_direction_ == TURN_LEFT)
        {
          state_ = TURNING_LEFT;
          ROS_INFO("GOING_BACK -> TURNING_LEFT");
        }
        else
        {
          state_ = TURNING_RIGHT;
          ROS_INFO("GOING_BACK -> TURNING_RIGHT");
        }
      }

      break;

    case TURNING_LEFT:
      cmd.linear.x = 0.0;
      cmd.angular.z = ANGULAR_VELOCITY_Z;
      if (TOGGLE_LED)
      {
        led_control.value  = RED;
      }

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_LEFT -> GOING_FORWARD");
      }
      break;

    case TURNING_RIGHT:
      cmd.linear.x = 0.0;
      cmd.angular.z = -ANGULAR_VELOCITY_Z;
      if (TOGGLE_LED)
      {
        led_control.value  = RED;
      }
      
      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_RIGHT -> GOING_FORWARD");
      }
      break;

    break;
  }
    pub_vel_.publish(cmd);
    pub_led_.publish(led_control);
}

}  // namespace fsm_bump_go
