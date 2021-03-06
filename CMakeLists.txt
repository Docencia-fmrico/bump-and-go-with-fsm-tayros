cmake_minimum_required(VERSION 2.8.3)
project(fsm_bump_go)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/fsm_bump_go/BumpGo.cpp
  src/fsm_bump_go/FinalBumpGo.cpp
  src/fsm_bump_go/lidarBumpGo.cpp
  src/fsm_bump_go/SensorGo.cpp
)

add_executable(bumpgo_node src/bumpgo_node.cpp)
add_executable(finalbumpgo_node src/finalbumpgo_node.cpp)
add_executable(lidarBumpgo_node src/lidarBumpgo_node.cpp)

target_link_libraries(bumpgo_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(finalbumpgo_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(lidarBumpgo_node ${catkin_LIBRARIES} ${PROJECT_NAME})

roslint_cpp(
 src/bumpgo_node.cpp
 src/finalbumpgo_node.cpp
 src/lidarBumpgo_node.cpp

 src/fsm_bump_go/BumpGo.cpp

 src/fsm_bump_go/FinalBumpGo.cpp
 src/fsm_bump_go/lidarBumpGo.cpp
 src/fsm_bump_go/SensorGo.cpp

 include/fsm_bump_go/BumpGo.h
 include/fsm_bump_go/SensorGo.h
 include/fsm_bump_go/FinalBumpGo.h
 include/fsm_bump_go/lidarBumpGo.h
)

catkin_add_gtest(BumpGoTest tests/BumpGoTest.cpp)
add_dependencies(BumpGoTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(TARGET BumpGoTest)
   target_link_libraries(BumpGoTest ${catkin_LIBRARIES} ${PROJECT_NAME})
 endif()
