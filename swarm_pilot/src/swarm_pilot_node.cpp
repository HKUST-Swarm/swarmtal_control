//
// Created by xuhao on 5/21/19.
//
#include <rclcpp/rclcpp.hpp>
#include "swarm_pilot.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("swarm_pilot"), "swarm pilot Initing");

    auto node = std::make_shared<SwarmPilot>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
