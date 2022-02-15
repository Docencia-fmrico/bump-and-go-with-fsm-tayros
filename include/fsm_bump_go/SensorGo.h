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

#ifndef FSM_BUMP_GO_SENSORGO_H
#define FSM_BUMP_GO_SENSORGO_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

namespace fsm_bump_go
{

class SensorGo
{
public:

	// Constructor
  SensorGo();

	// Functions 
  //virtual void sensorCallback() = 0;	// Need to implement center detection(LEFT || RIGHT)
  void step();

	// Instance variables(used in the virtual Callback)
	bool turn_direction_; // RIGHT OR LEFT
	bool pressed_;

	static const bool TURN_LEFT = true;  
	static const bool TURN_RIGHT = false; 

  ros::NodeHandle n_;

private:

  static const int GOING_FORWARD   = 0;
  static const int GOING_BACK = 1;
	static const int TURNING_LEFT = 2;
  static const int TURNING_RIGHT = 3;
	
  static constexpr double TURNING_TIME = 5.0;
  static constexpr double BACKING_TIME = 3.0;

	static constexpr double LINEAR_VELOCITY_X = 0.2; 
	static constexpr double ANGULAR_VELOCITY_Z = 0.4; 

  ros::Publisher pub_vel_;

  int state_;

  ros::Time press_ts_;
  ros::Time turn_ts_;

};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_SENSORGO_H
