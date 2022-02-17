[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6870050&assignment_repo_type=AssignmentRepo)
# fsm_bump_go

<div align="center">
<img width=400px src="" alt="explode"></a>
</div>

<h3 align="center">Bump And Go </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Sensor_Go(base class)](#sensorgo)
- [Final Bump Go](#finalbumpgo)
- [Team](#team)
- [Licencia](#licencia)



## SensorGo 

Sensor Go, is the **base class** from which the different implementations of a "bump and go" will **inherit**.  

This will be in charge of **containing the state machine** which will be formed by 4 states: 

- GOING_FORWARD
- GOING_BACK
- TURNING_LEFT
- TURNING_RIGHT

Therefore in the ".cpp" of this one is implemented the function SensorGo::step() that controls the movements of the robot in function to two variables, that in this case will be part of the public instance variables and they are: 

  **bool pressed_** -> Indicates if the sensor detects that the robot must turn. 
  **bool turn_direction_** -> Indicates the direction the turn should be made. 
  
With this we obtain that the classes that inherit of this one only have to implement a function "Callback" that takes the values of the sensor and translates them to introduce them to these two variables.

-----------------------------------------------------------------------
Snippet(use of pressed_):
``` cpp
  if (pressed_)
  {
     press_ts_ = ros::Time::now();
     state_ = GOING_BACK;
     ROS_INFO("GOING_FORWARD -> GOING_BACK");
  }
```
-----------------------------------------------------------------------

-----------------------------------------------------------------------
Snippet(use of turn_direction_):
``` cpp
  if(turn_direction_ == TURN_LEFT)
  {
    state_ = TURNING_LEFT;
    ROS_INFO("GOING_BACK -> TURNING_LEFT");
   }
   else 
   {
     state_ = TURNING_RIGHT;
     ROS_INFO("GOING_BACK -> TURNING_RIGHT");
   }
```
-----------------------------------------------------------------------

## FinalBumpGo

FinalBumpGo inherits from the SensoGo base class and implements a sensorCallback function that allows it to communicate with the kobuki bumpers to take the state and indicate it to the state machine. 

If it detects that the left bumper has been activated, it will indicate a right turn and if the right or central bumper is activated, it will turn left. 

-----------------------------------------------------------------------
Snippet(sensorCallback):
``` cpp
void
FinalBumpGo::sensorCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  pressed_ = msg->state; 
  bumper_ = msg->bumper; 

  if(bumper_ == LEFT){turn_direction_ = TURN_RIGHT;}
  
  else{turn_direction_ = TURN_LEFT;}
}
```
-----------------------------------------------------------------------

## Team

<img width=600px src="" alt="explode"></a>
<h5 align="center">TayRos 2022</h5
  
- [Saul Navajas](https://github.com/SaulN99)
- [Guillermo Alcocer](https://github.com/GuilleAQ)
- [Adrian Madinabeitia](https://github.com/madport)
- [Ivan Porras](https://github.com/porrasp8)

## Licencia 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(TayRos) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
