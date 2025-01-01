#pragma once
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Eigen>
#include <swarmcomm_msgs/msg/incoming_broadcast_data.hpp>
#include <swarmtal_msgs/msg/drone_onboard_command.hpp>
#include <swarmtal_msgs/msg/drone_pos_ctrl_cmd.hpp>
#include <swarmtal_msgs/msg/drone_commander_state.hpp>
#include <swarmcomm_msgs/msg/remote_uwb_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <swarm_msgs/msg/swarm_drone_basecoor.hpp>
#include <swarm_msgs/msg/swarm_fused.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <swarm_msgs/Pose.h>
#include <bspline/msg/bspline.hpp>
#include <swarmcomm_msgs/msg/swarm_network_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "swarm_formation_control.h"

using namespace swarmcomm_msgs;
using namespace swarmtal_msgs;
using DroneCommanderState = swarmtal_msgs::msg::DroneCommanderState;
    
class SwarmPilot;
struct NetworkStatus {
    rclcpp::Time last_heartbeat = rclcpp::Time(0);
    int drone_id;
    bool active = 0;
    double quality = -1;//0-100
    double bandwidth = -1; //0-100
    int hops = -1; //Hops to the target.
};

// 新增参数结构体
struct SwarmPilotParam {
    bool planning_debug_mode{false};
    int drone_id{-1};
    int accept_cmd_node_id{-1};
    double send_drone_status_freq{5.0};
    double Ts{0.1};
    double heartbeat_timeout{0.5};
    bool enable_planner{false};
    // 可根据需要继续加入其他参数
};

class SwarmPilot : public rclcpp::Node {
    // 修改：使用 ROS2 的节点指针
    rclcpp::Node::SharedPtr node_;

    // 旧的 ros::Publisher / ros::Subscriber 替换为 rclcpp::Publisher / rclcpp::Subscription
    rclcpp::Subscription<swarmcomm_msgs::msg::RemoteUwbInfo>::SharedPtr uwb_remote_sub;
    rclcpp::Subscription<swarm_msgs::msg::SwarmFused>::SharedPtr swarm_local_sub;
    rclcpp::Subscription<swarm_msgs::msg::SwarmDroneBasecoor>::SharedPtr swarm_basecoor_sub;
    rclcpp::Subscription<swarmcomm_msgs::msg::DroneNetworkStatus>::SharedPtr drone_network_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<swarmtal_msgs::msg::DronePosCtrlCmd>::SharedPtr local_cmd_sub;
    rclcpp::Publisher<swarmtal_msgs::msg::DroneOnboardCommand>::SharedPtr onboardcmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planning_tgt_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr exprolaration_pub;
    rclcpp::Publisher<swarmcomm_msgs::msg::SwarmNetworkStatus>::SharedPtr swarm_network_status_pub;

    // 使用 ROS2 的定时器
    rclcpp::TimerBase::SharedPtr eight_trajectory_timer, net_timer;

    // 存储参数
    SwarmPilotParam param_;

    // 其他成员变量
    double mission_trajectory_timer_t = 0.0, eight_trajectory_timer_period = 30.0;
    bool eight_trajectory_enable = false, eight_trajectory_yaw_enable = false;
    int eight_traj_mode = 0; //0: more on x 1: more on y
    bool mission_trajs_enable = false, mission_trajs_yaw_enable = false;
    
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 4>> mission_trajs; //[X, Y, Z, YAW] XYZ IN FLU. YAW in NED.
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> mission_trajs_t;

    int cur_mission_index = 0;
    int cur_mission_id = 0;
    
    Vector3d eight_trajectory_center;

    int accept_cmd_node_id = -1; //-1 Accept all, >=0 accept corresponding
    double heartbeat_timeout = 0.5;
    double send_odom_freq = 50.0;

    rclcpp::Time last_send_odom;

    DroneCommanderState cmd_state;
    rclcpp::Time last_send_drone_status;

    int self_id = -1;
    uint8_t buf[1000] = {0};
    sensor_msgs::msg::TimeReference uwb_time_ref;

    std::map<int, NetworkStatus> swarm_network_status;

    SwarmFormationControl * formation_control = nullptr;

    bool planning_debug_mode = false;
    bool enable_planner;


public:
    SwarmPilot();

    // 其他函数接口不变，内部实现改为 rclcpp
    bool is_planning_control_available();

    rclcpp::Time LPS2ROSTIME(const int32_t &lps_time);
    int32_t ROSTIME2LPS(rclcpp::Time ros_time);

    void send_position_command(Eigen::Vector3d pos, double yaw, Eigen::Vector3d vel = Eigen::Vector3d::Zero(), bool enable_planning = false);
    void send_velocity_command(Eigen::Vector3d vel, double yaw);

    void on_uwb_remote_node(const swarmcomm_msgs::msg::RemoteUwbInfo & info);

    void on_remote_cmd(const DroneOnboardCommand& onboardCommand);
    
    void send_planning_command(DroneOnboardCommand cmd);

    void send_start_exploration(const DroneOnboardCommand & cmd);

    void eight_trajectory_timer_callback();
    void timer_callback();
    
    void drone_network_callback(const swarmcomm_msgs::msg::DroneNetworkStatus & status);
    void network_monitior_timer_callback();
    void mission_trajs_timer_callback();

    void start_mission_trajs(const DroneOnboardCommand & cmd);
    void start_spec_trajs(const DroneOnboardCommand & cmd);
    void end_mission();
    void load_missions();
};

