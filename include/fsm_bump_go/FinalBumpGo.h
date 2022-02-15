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

#ifndef FSM_BUMP_GO_FINALBUMPGO_H
#define FSM_BUMP_GO_FINALBUMPGO_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"
#include "fsm_bump_go/SensorGo.h"

namespace fsm_bump_go
{

class FinalBumpGo : public SensorGo
{
public:

	// Constructor
  FinalBumpGo();

	// Functions 
	void sensorCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);	// Need to implement center detection(LEFT || RIGHT)

private: 

	static const int LEFT = 0;
	static const int CENTER = 1;
	static const int RIGHT = 2;

	int bumper_; 

	ros::Subscriber sub_sensor_;
};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_FINALBUMPGO_H
