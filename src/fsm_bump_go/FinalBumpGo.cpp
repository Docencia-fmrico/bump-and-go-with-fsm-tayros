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

#include "fsm_bump_go/FinalBumpGo.h"
#include "kobuki_msgs/BumperEvent.h"
#include "ros/ros.h"

namespace fsm_bump_go
{

FinalBumpGo::FinalBumpGo()
: SensorGo()
{
  std::string sub_sensor_path =  n_.param("sub_sensor_path", std::string("/mobile_base/events/bumper"));
  sub_sensor_ = n_.subscribe(sub_sensor_path, 1, &FinalBumpGo::sensorCallback, this);
}

void
FinalBumpGo::sensorCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  pressed_ = msg->state; 
  bumper_ = msg->bumper; 

  if(bumper_ == LEFT)
  {
    turn_direction_ = TURN_RIGHT; 
  }

  // If bumper detects left or center, the robot turn left 
  else
  {
    turn_direction_ = TURN_LEFT; 
  }

}

}  // namespace fsm_bump_go
  
