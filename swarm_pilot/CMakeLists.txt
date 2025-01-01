cmake_minimum_required(VERSION 3.5)
project(swarm_pilot)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(swarmcomm_msgs REQUIRED)
find_package(swarmtal_msgs REQUIRED)
find_package(bspline REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 添加可执行文件
add_executable(${PROJECT_NAME}_node
  src/swarm_pilot_node.cpp
  src/swarm_pilot.cpp
  src/swarm_formation_control.cpp
)

# 为可执行文件添加依赖
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  swarm_msgs
  swarmcomm_msgs
  swarmtal_msgs
  geometry_msgs
  sensor_msgs
  bspline
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

# 安装可执行文件到 lib/<package_name> 目录
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件到 include/<package_name> 目录
install(DIRECTORY include/
  DESTINATION include/
)

# 如果有 Python 脚本，添加以下安装规则（可选）
# install(PROGRAMS scripts/your_python_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# 最后，声明这个包是一个ment_package
ament_package()
