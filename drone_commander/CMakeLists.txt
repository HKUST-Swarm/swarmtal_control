cmake_minimum_required(VERSION 2.8.3)
project(drone_commander)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -DFCHardware=DJI_SDK)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  swarmtal_msgs
  # dji_sdk
)

find_package (Eigen3 3.3 NO_MODULE)


catkin_package(
 LIBRARIES swarm_commander
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs swarm_msgs drone_position_control
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/drone_commander_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(gpstoodom src/gpstoodom.cpp)
# add_dependencies(gpstoodom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gpstoodom
  # ${catkin_LIBRARIES}
# )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
