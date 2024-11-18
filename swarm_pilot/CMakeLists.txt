cmake_minimum_required(VERSION 2.8.3)
project(swarm_pilot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  swarm_msgs
  swarmcomm_msgs
  swarmtal_msgs
  bspline
)

find_package(Eigen3)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES swarm_pilot
#  CATKIN_DEPENDS dji_sdk roscpp rospy std_msgs swarm_msgs swarmtal_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

 add_executable(${PROJECT_NAME}_node src/swarm_pilot_node.cpp src/swarm_pilot.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

