#pragma once

#include <swarmtal_msgs/msg/drone_onboard_command.hpp>
#include <swarmtal_msgs/msg/drone_pos_ctrl_cmd.hpp>
#include <swarm_msgs/msg/swarm_fused.hpp>
#include <swarm_msgs/Pose.h>
#include <swarm_msgs/msg/swarm_drone_basecoor.hpp>
#include <bspline/msg/bspline.hpp>
#include <map>
#include <Eigen/Dense>

class SwarmPilot;

using DroneOnboardCommand = swarmtal_msgs::msg::DroneOnboardCommand;
using DronePosCtrlCmd = swarmtal_msgs::msg::DronePosCtrlCmd;

class SwarmFormationControl {
    int self_id;
    int formation_mode = DroneOnboardCommand::CTRL_FORMATION_IDLE;
    int master_id = -1;

    std::map<int, Eigen::Vector3d> swarm_pos;
    std::map<int, Eigen::Vector3d> swarm_vel;
    std::map<int, double> swarm_yaw;

    std::map<int, Swarm::Pose> swarm_transformation;

    Eigen::Vector3d dpos;
    double dyaw;
    SwarmPilot * pilot = nullptr;
    
    double Ts;
public:
    SwarmFormationControl(int _self_id, SwarmPilot * _pilot, double filter_Ts);

    void on_swarm_localization(const swarm_msgs::msg::SwarmFused & swarm_fused);
    void on_swarm_basecoor(const swarm_msgs::msg::SwarmDroneBasecoor & swarm_fused);

    void on_swarm_traj(const bspline::msg::Bspline & bspl);

    void on_position_command(DroneOnboardCommand cmd, int _id);
    void on_drone_position_command(DronePosCtrlCmd pos_cmd);
    void set_swarm_formation_mode(uint8_t _formation_mode, int master_id, int sub_mode, Eigen::Vector3d dpos = Eigen::Vector3d::Zero(), double dyaw = 0);
    void end_formation();
};

