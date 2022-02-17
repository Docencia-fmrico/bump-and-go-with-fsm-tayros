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
:n_("~")
{
  // Roslaunch params 
  std::string pub_vel_path =  n_.param("pub_vel_path", std::string("/mobile_base/commands/velocity"));
  std::string pub_led_path =  n_.param("pub_led_path", std::string("/mobile_base/commands/led1"));

  pub_vel_ = n_.advertise<geometry_msgs::Twist>(pub_vel_path, 1);
  pub_led_ = n_.advertise<kobuki_msgs::Led>(pub_led_path, 1);
  pub_sound_ = n_.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);

  state_ = n_.param("init_state", 0);
  pressed_ = n_.param("init_pressed", false);
  turn_direction_ = n_.param("init_direction", false);
  linear_velocity_x = n_.param("linear_vel", 0.2);
  angular_velocity_z = n_.param("angular_vel", 0.4);
}

int
SensorGo::get_state()
{
  return state_;
}

bool
SensorGo::get_turn_direction()
{
  return turn_direction_;
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
      
      cmd.linear.x = linear_velocity_x;
      cmd.angular.z = 0;
      led_control.value = GREEN;

      if (pressed_)
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }

      break;

    case GOING_BACK:
      cmd.linear.x = -linear_velocity_x;
      cmd.angular.z = 0;
      led_control.value  = ORANGE;
      sound_control.value = SOUND_ERROR;
      pub_sound_.publish(sound_control);

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
      cmd.linear.x = 0;
      cmd.angular.z = angular_velocity_z;
      led_control.value  = RED;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_LEFT -> GOING_FORWARD");
      }
      break;

    case TURNING_RIGHT:
      cmd.linear.x = 0;
      cmd.angular.z = -angular_velocity_z;
      led_control.value  = RED;

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
