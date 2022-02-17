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

namespace fsm_bump_go
{

lidarBumpGo::lidarBumpGo()
{
  sub_sensor_ = n_.subscribe("/scan_filtered", 1, &lidarBumpGo::sensorCallback, this);
  ros::param::get("/lidarBuumpgp/DETECTION_DISTANCE", MIN_DISTANCE_);
  ros::param::get("/lidarBuumpgp/RANGE", range_);
  
}

void
lidarBumpGo::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int size = msg->ranges.size();
  center_ = size / 2;
  
  float nearest_distance = 100;

  for (int i = center_ - range_ ; i <= center_ + range_ ; i++){ // Array index
    if (nearest_distance > msg->ranges[i]){
        nearest_distance = msg->ranges[i];
        nearest_position_ = i; 

               
    }
  }
  
  if (nearest_distance <= MIN_DISTANCE_) { pressed_ = true; }
  else { pressed_ = false ; }
  

  if(nearest_position_ <= center_)
  {
    turn_direction_ = TURN_RIGHT; 
  }

  // If bumper detects left or center, the robot turn left 
  if (nearest_position_ > center_)
  {
    turn_direction_ = TURN_LEFT; 
  }

}

}  // namespace fsm_bump_go