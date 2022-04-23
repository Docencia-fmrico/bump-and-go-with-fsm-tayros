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

#include "fsm_bump_go/lidarBumpGo.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include <string>

enum
{
  LEFT = 0,
  RIGHT = 1,
};

namespace fsm_bump_go
{

lidarBumpGo::lidarBumpGo()
{
  std::string sub_sensor_topic =  n_.param("sub_sensor_path", std::string("/scan_filtered"));

  sub_sensor_ = n_.subscribe(sub_sensor_topic, 1, &lidarBumpGo::sensorCallback, this);
  MIN_DISTANCE_ = n_.param("DETECTION_DISTANCE", 0.5);
  range_ = n_.param("RANGE", 60);
}

float
lidarBumpGo::obtainDistance(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int size = msg->ranges.size();
  center_ = 0;

  /* Left range */
  float nearest_distance = 100;  /* The value its aleatory */
  for (int i = 0; i < range_; i++)
  {
    if (nearest_distance > msg->ranges[i])
    {
      nearest_distance = msg->ranges[i];
      nearest_position_ = LEFT;
    }
  }

  /* Right range */
  for (int i = size-1; i > (size - range_); i--)
  {
    if (nearest_distance > msg->ranges[i])
    {
        nearest_distance = msg->ranges[i];
        nearest_position_ = RIGHT; 
    }
  }

  return nearest_distance;
}

void
lidarBumpGo::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float nearest_distance = obtainDistance(msg);

  if (nearest_distance <= MIN_DISTANCE_)
  {
    pressed_ = true;
  }
  else
  {
    pressed_ = false;
  }

  if (nearest_position_ == LEFT)
  {
    turn_direction_ = TURN_RIGHT;
  }

  // If bumper detects left or center, the robot turn left
  if (nearest_position_ == RIGHT)
  {
    turn_direction_ = TURN_LEFT;
  }
}

}  // namespace fsm_bump_go
