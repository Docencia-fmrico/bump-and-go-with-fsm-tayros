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

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fsm_bump_go");

  fsm_bump_go::BumpGo fsm_bump_go;

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    fsm_bump_go.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
