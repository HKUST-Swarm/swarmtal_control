#include <swarm_pilot.h>
#include <fstream>
#include <swarmcomm_msgs/msg/swarm_network_status.hpp>
#include "swarm_pilot_utils.h"

// 移除 using namespace，改为使用命名空间 msg::


rclcpp::Time SwarmPilot::LPS2ROSTIME(const int32_t &lps_time) {
    rclcpp::Time base(
        uwb_time_ref.header.stamp.sec,
        uwb_time_ref.header.stamp.nanosec,
        RCL_ROS_TIME
    );
    double ref_sec = rclcpp::Time(uwb_time_ref.time_ref).seconds(); 
    auto adjusted = rclcpp::Time(base) - rclcpp::Duration::from_seconds(ref_sec);
    auto result = adjusted + rclcpp::Duration::from_seconds(lps_time / 1000.0);
    return result;
}

int32_t SwarmPilot::ROSTIME2LPS(rclcpp::Time ros_time) {
    rclcpp::Time base(
        uwb_time_ref.header.stamp.sec,
        uwb_time_ref.header.stamp.nanosec,
        RCL_ROS_TIME
    );
    double ref_sec = rclcpp::Time(uwb_time_ref.time_ref).seconds();
    double lps_t_s = (ros_time - base).seconds() + ref_sec;
    return static_cast<int32_t>(lps_t_s * 1000);
}
 
void SwarmPilot::send_position_command(Eigen::Vector3d pos, double yaw, Eigen::Vector3d vel, bool enable_planning) {
    if (enable_planning) {
        geometry_msgs::msg::PoseStamped pose_tgt;
        pose_tgt.header.stamp = node_->now();
        pose_tgt.header.frame_id = "world";
        
        pose_tgt.pose.position.x = pos.x();
        pose_tgt.pose.position.y = pos.y();
        pose_tgt.pose.position.z = pos.z();

        Eigen::Quaterniond _quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        // pose_tgt.
        pose_tgt.pose.orientation.w = _quat.w();
        pose_tgt.pose.orientation.x = _quat.x();
        pose_tgt.pose.orientation.y = _quat.y();
        pose_tgt.pose.orientation.z = _quat.z();

        RCLCPP_INFO(node_->get_logger(), "send_position_command with planning");
        planning_tgt_pub->publish(pose_tgt);
    } else {
        swarmtal_msgs::msg::DroneOnboardCommand cmd;
        cmd.command_type = swarmtal_msgs::msg::DroneOnboardCommand::CTRL_POS_COMMAND;
        cmd.param1 = pos.x()*10000;
        cmd.param2 = pos.y()*10000;
        cmd.param3 = pos.z()*10000;
        cmd.param4 = yaw * 10000;
        cmd.param5 = vel.x()*10000;
        cmd.param6 = vel.y()*10000;
        cmd.param7 = vel.z()*10000;
        cmd.param8 = 0;
        cmd.param9 = 0;
        cmd.param10 = 0;

        onboardcmd_pub->publish(cmd);
    }
}


void SwarmPilot::send_velocity_command(Eigen::Vector3d vel, double yaw) {
    swarmtal_msgs::msg::DroneOnboardCommand cmd;
    cmd.command_type = swarmtal_msgs::msg::DroneOnboardCommand::CTRL_VEL_COMMAND;
    cmd.param1 = vel.x()*10000;
    cmd.param2 = vel.y()*10000;
    cmd.param3 = vel.z()*10000;
    cmd.param4 = yaw * 10000;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;
    cmd.param8 = 0;
    cmd.param9 = 0;
    cmd.param10 = 0;

    onboardcmd_pub->publish(cmd);
}

// 参数获取改用 declare_parameter / get_parameter，去除 ros::NodeHandle
void SwarmPilot::load_missions() {
    int mission_num = 0;
    node_->declare_parameter<int>("mission_num", 0);
    node_->get_parameter("mission_num", mission_num);

    std::string mission_path;
    node_->declare_parameter<std::string>("mission_path", "/home/dji/SwarmConfig/missions/");
    node_->get_parameter("mission_path", mission_path);

    RCLCPP_INFO(node_->get_logger(), "[SWARM_PILOT] Trying to load %d missions from %s", mission_num, mission_path.c_str());
    char mission_i_path[100] = {0};
    for (int i = 0; i < mission_num; i ++) {
        sprintf(mission_i_path, "%s/mission_%d_drone%d.csv", mission_path.c_str(), i, self_id);
        auto mission = readMatrix(mission_i_path);
        Eigen::Matrix<double, Eigen::Dynamic, 1> mission_traj_t = mission.block(0, 0, mission.rows(), 1);
        Eigen::Matrix<double, Eigen::Dynamic, 4> mission_traj = mission.block(0, 1, mission.rows(), 4);
        if (mission_traj_t.rows() > 0) {
            mission_trajs_t.push_back(mission_traj_t);
            mission_trajs.push_back(mission_traj);
            RCLCPP_INFO(node_->get_logger(), "[SWARM_PILOT] Loaded mission %d from %s pts: %d duration: %3.1fs",
                i, mission_i_path, static_cast<int>(mission_traj_t.rows()), mission_traj_t(mission_traj_t.rows() - 1));
        }
    }
}

// 构造函数去除 ros::NodeHandle，使用 rclcpp::Node::SharedPtr
SwarmPilot::SwarmPilot()
    : Node("swarm_pilot"){

    // 声明并获取参数
    node_->declare_parameter<bool>("planning_debug_mode", false);
    node_->declare_parameter<int>("drone_id", -1);
    node_->declare_parameter<int>("acpt_cmd_node", -1);
    node_->declare_parameter<double>("send_drone_status_freq", 5.0);
    node_->declare_parameter<double>("Ts", 0.1);
    node_->declare_parameter<double>("heartbeat_timeout", 0.5);
    node_->declare_parameter<bool>("enable_planner", false);

    node_->get_parameter("planning_debug_mode", planning_debug_mode);
    node_->get_parameter("drone_id", self_id);
    node_->get_parameter("acpt_cmd_node", accept_cmd_node_id);
    node_->get_parameter("send_drone_status_freq", send_drone_status_freq);
    double Ts;
    node_->get_parameter("Ts", Ts);
    node_->get_parameter("heartbeat_timeout", heartbeat_timeout);
    node_->get_parameter("enable_planner", enable_planner);

    assert(self_id > 0 && "Self ID must be bigger than 0!!!");

    load_missions();

    formation_control = new SwarmFormationControl(self_id, this, Ts);

    // 替换原来的 advertises
    onboardcmd_pub = node_->create_publisher<swarmtal_msgs::msg::DroneOnboardCommand>("/drone_commander/onboard_command", 1);
    planning_tgt_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/goal", 10);
    exprolaration_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
    swarm_network_status_pub = node_->create_publisher<swarmcomm_msgs::msg::SwarmNetworkStatus>("/swarm_drones/swarm_network_status", 10);

    swarm_local_sub = node_->create_subscription<swarm_msgs::msg::SwarmFused>(
        "/swarm_drones/swarm_drone_fused",
        1,
        [this] (const swarm_msgs::msg::SwarmFused::SharedPtr msg) {
            this->formation_control->on_swarm_localization(*msg);
        }
    );
    swarm_basecoor_sub = node_->create_subscription<swarm_msgs::msg::SwarmDroneBasecoor>(
        "/swarm_drones/SwarmDroneBasecoor",
        1,
        [this] (const swarm_msgs::msg::SwarmDroneBasecoor::SharedPtr msg) {
            this->formation_control->on_swarm_basecoor(*msg);
        }
    );
    local_cmd_sub = node_->create_subscription<swarmtal_msgs::msg::DronePosCtrlCmd>(
        "/drone_position_control/drone_pos_cmd",
        1,
        [this] (const swarmtal_msgs::msg::DronePosCtrlCmd::SharedPtr msg) {
            this->formation_control->on_drone_position_command(*msg);
        }
    );
    drone_network_sub = node_->create_subscription<swarmcomm_msgs::msg::DroneNetworkStatus>(
        "/swarm_loop/drone_network_status",
        1,
        [this] (const swarmcomm_msgs::msg::DroneNetworkStatus::SharedPtr msg) {
            this->drone_network_callback(*msg);
        }
    );

    eight_trajectory_timer = node_->create_wall_timer(
        std::chrono::duration<double>(0.02),
        [this] () {
            this->timer_callback();
        }
    );
    net_timer = node_->create_wall_timer(
        std::chrono::duration<double>(0.02),
        std::bind(&SwarmPilot::network_monitior_timer_callback, this)
    );

    last_send_drone_status = node_->now();
    last_send_odom = node_->now();

    RCLCPP_INFO(node_->get_logger(), "[SWARM_PILOT] Node %d ready", self_id);
}

bool SwarmPilot::is_planning_control_available() {
    if (planning_debug_mode) {
        return true;
    } else {
        return cmd_state.control_auth == swarmtal_msgs::msg::DroneCommanderState::CTRL_AUTH_THIS
            && cmd_state.flight_status == swarmtal_msgs::msg::DroneCommanderState::FLIGHT_STATUS_IN_AIR;
    }
}

// 改写timer回调函数：移除rclcpp::TimerEvent e
void SwarmPilot::timer_callback() {
    // 使用 node_->now() 作为时间戳
    if (is_planning_control_available()) {
        if (mission_trajs_enable) {
            mission_trajs_timer_callback();
        } else if(eight_trajectory_enable) {
            eight_trajectory_timer_callback();
        } else {
            mission_trajectory_timer_t = 0;
            mission_trajs_enable = false;
            eight_trajectory_enable = false;
        }
    } else {
        mission_trajectory_timer_t = 0;
        mission_trajs_enable = false;
        eight_trajectory_enable = false;
    }
}

void SwarmPilot::mission_trajs_timer_callback() {
    if (mission_trajs_enable && cur_mission_id < (int)mission_trajs_t.size()) {
        auto & mission_traj_t = mission_trajs_t[cur_mission_id];
        auto & mission_traj = mission_trajs[cur_mission_id];
        bool need_send_planning_cmd = false;

        // 此处只示例性展示时间逻辑
        double dt = 0.02;
        while ((cur_mission_index < 0 || cur_mission_index < (int)mission_traj_t.rows() - 1)
            && (mission_traj_t[cur_mission_index + 1] <  mission_trajectory_timer_t)) {
            cur_mission_index++;
            need_send_planning_cmd = true;
        }

        mission_trajectory_timer_t += dt;

        if (!need_send_planning_cmd) {
            return;
        }

        geometry_msgs::msg::PoseStamped pose_tgt;
        pose_tgt.header.stamp = node_->now();
        pose_tgt.header.frame_id = "world";

        pose_tgt.pose.position.x = mission_traj(cur_mission_index, 0);
        pose_tgt.pose.position.y = mission_traj(cur_mission_index, 1);
        pose_tgt.pose.position.z = mission_traj(cur_mission_index, 2);

        double yaw_sp = 0;
        if (mission_trajs_yaw_enable) {
            yaw_sp = mission_traj(cur_mission_index, 3);
        }

        Eigen::Quaterniond _quat(Eigen::AngleAxisd(-yaw_sp, Eigen::Vector3d::UnitZ()));
        pose_tgt.pose.orientation.w = _quat.w();
        pose_tgt.pose.orientation.x = _quat.x();
        pose_tgt.pose.orientation.y = _quat.y();
        pose_tgt.pose.orientation.z = _quat.z();
        
        RCLCPP_INFO(node_->get_logger(),
            "[SWARM_PILOT] Mission index %d traj navigate to [%.2f, %.2f, %.2f]",
            cur_mission_index,
            pose_tgt.pose.position.x,
            pose_tgt.pose.position.y,
            pose_tgt.pose.position.z
        );
        planning_tgt_pub->publish(pose_tgt);

        if (cur_mission_index >= mission_traj_t.rows() - 1) {
            RCLCPP_INFO(node_->get_logger(), "[SWARM_PILOT] Mission_trajs finish index %d t %.1f.",
                cur_mission_index, mission_trajectory_timer_t);
            mission_trajs_enable = false;
            cur_mission_index = -1;
            mission_trajectory_timer_t = 0;
            return;
        }
    }
}

void SwarmPilot::eight_trajectory_timer_callback() {
    if (!eight_trajectory_enable || !is_planning_control_available()) {
        eight_trajectory_enable = false;
        mission_trajectory_timer_t = 0.0;
        return;
    }

    double T = eight_trajectory_timer_period;
    double dt = 0.02;
    double _t = mission_trajectory_timer_t * 2.0 * M_PI / T; 
    double ox = eight_trajectory_center(0);
    double oy = eight_trajectory_center(1);
    double oz = eight_trajectory_center(2);

    double x, y, vx, vy, ax, ay, yaw;
    if (eight_traj_mode == 0) {
        x = ox + 2 * sin(_t);
        y = oy + 2 * sin(_t) * cos(_t);
        vx = 2 * cos(_t) * 2.0*M_PI/T;
        vy = 2 * cos(2*_t) * 2.0*M_PI/T;
        ax = -2 * sin(_t) * std::pow(2.0*M_PI/T,2);
        ay = -2 * sin(2*_t) * std::pow(2.0*M_PI/T,2)*2.0;
        yaw = atan2(-cos(2*_t), cos(_t));
    } else {
        x = ox + 2 * sin(_t) * cos(_t);
        y = oy + 2 * sin(_t);
        vy = 2 * cos(_t) * 2.0*M_PI/T;
        vx = 2 * cos(2*_t) * 2.0*M_PI/T;
        ay = -2 * sin(_t) * std::pow(2.0*M_PI/T,2);
        ax = -2 * sin(2*_t) * std::pow(2.0*M_PI/T,2)*2.0;
        yaw = atan2(cos(_t), -cos(2*_t));
    }

    swarmtal_msgs::msg::DroneOnboardCommand onboardCommand;
    onboardCommand.command_type = swarmtal_msgs::msg::DroneOnboardCommand::CTRL_POS_COMMAND;
    onboardCommand.param1 = x * 10000;
    onboardCommand.param2 = y * 10000;
    onboardCommand.param3 = oz * 10000;

    if (eight_trajectory_yaw_enable) {
        onboardCommand.param4 = yaw * 10000;
    } else {
        onboardCommand.param4 = 666666;
    }

    onboardCommand.param5 = vx * 10000;
    onboardCommand.param6 = vy * 10000;
    onboardCommand.param7 = 0;
    onboardCommand.param8 = ax * 10000;
    onboardCommand.param9 = ay * 10000;
    onboardCommand.param10 = 0;

    onboardcmd_pub->publish(onboardCommand);
    mission_trajectory_timer_t += dt;
}

void SwarmPilot::start_mission_trajs(const swarmtal_msgs::msg::DroneOnboardCommand & cmd) {
    if (!mission_trajs_enable) {
        cur_mission_id = cmd.param1;
        mission_trajs_yaw_enable = cmd.param2>0;
        if (cur_mission_id >= (int)mission_trajs_t.size()) {
            RCLCPP_WARN(node_->get_logger(),
                "[SWAMR_PILOT] start_mission_trajs rejected. Mission %d not loaded.", cur_mission_id);
            return;
        }

        mission_trajs_enable = true;
        mission_trajectory_timer_t = 0;
        cur_mission_index = -1;
        RCLCPP_INFO(node_->get_logger(),
            "[SWAMR_PILOT] start_mission_trajs mission %d enable_yaw %d",
            cur_mission_id, mission_trajs_yaw_enable);
    }
}

void SwarmPilot::start_spec_trajs(const swarmtal_msgs::msg::DroneOnboardCommand & cmd) {
    if (cmd.param1 == 1) {
        eight_trajectory_enable = true;
        eight_trajectory_yaw_enable = cmd.param2;
        eight_trajectory_timer_period = cmd.param3/10000.0;
        eight_traj_mode = cmd.param7;
        mission_trajectory_timer_t = 0;
        eight_trajectory_center = Eigen::Vector3d(
            cmd.param4/10000.0,
            cmd.param5/10000.0,
            cmd.param6/10000.0
        );
        RCLCPP_INFO(node_->get_logger(),
            "[SWAMR_PILOT] Start 8 trajectort: enable Yaw: %d mode %d, T %3.1f center [%3.2f, %3.2f, %3.2f]",
            eight_trajectory_yaw_enable, eight_traj_mode, 
            eight_trajectory_timer_period,
            eight_trajectory_center.x(),
            eight_trajectory_center.y(),
            eight_trajectory_center.z()
        );
    }
}

void SwarmPilot::end_mission() {
    if (mission_trajs_enable || eight_trajectory_enable) {
        mission_trajs_enable = false;
        eight_trajectory_enable = false;
        mission_trajectory_timer_t = 0;
        cur_mission_index = -1;
        formation_control->end_formation();
        RCLCPP_INFO(node_->get_logger(), "[SWARM_PILOT] Terminate current mission.");
    }
}

void SwarmPilot::send_start_exploration(const swarmtal_msgs::msg::DroneOnboardCommand & cmd) {
    geometry_msgs::msg::PoseStamped pose_tgt;
    pose_tgt.header.stamp = node_->now();
    pose_tgt.header.frame_id = "world";
    exprolaration_pub->publish(pose_tgt);
    RCLCPP_INFO(node_->get_logger(), "[SWAMR_PILOT] Sending exploration command.");
}

void SwarmPilot::send_planning_command(swarmtal_msgs::msg::DroneOnboardCommand cmd) {
    if (cmd.command_type == swarmtal_msgs::msg::DroneOnboardCommand::CTRL_PLANING_TGT_COMMAND &&
        is_planning_control_available()) {
        if (enable_planner) {
            geometry_msgs::msg::PoseStamped pose_tgt;
            pose_tgt.header.stamp = node_->now();
            pose_tgt.header.frame_id = "world";
            
            pose_tgt.pose.position.x = cmd.param1 / 10000.0;
            pose_tgt.pose.position.y = cmd.param2 / 10000.0;
            pose_tgt.pose.position.z = cmd.param3 / 10000.0;

            Eigen::Quaterniond _quat(Eigen::AngleAxisd(-cmd.param4/10000.0, Eigen::Vector3d::UnitZ()));
            pose_tgt.pose.orientation.w = _quat.w();
            pose_tgt.pose.orientation.x = _quat.x();
            pose_tgt.pose.orientation.y = _quat.y();
            pose_tgt.pose.orientation.z = _quat.z();
            RCLCPP_INFO(node_->get_logger(),
                "[SWAMR_PILOT] Sending traj fly to [%.2f, %.2f, %.2f]",
                pose_tgt.pose.position.x, pose_tgt.pose.position.y, pose_tgt.pose.position.z
            );
            planning_tgt_pub->publish(pose_tgt);
        } else {
            cmd.command_type = swarmtal_msgs::msg::DroneOnboardCommand::CTRL_POS_COMMAND;
            onboardcmd_pub->publish(cmd);
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "[SWAMR_PILOT] Reject fly to. Planning not ready.");
    }
}

void SwarmPilot::on_remote_cmd(const swarmtal_msgs::msg::DroneOnboardCommand& cmd) {
    // 这里只是示例回调
    RCLCPP_INFO(node_->get_logger(),
        "[SWAMR_PILOT] Recv onboard cmd from ??? type %d is planning ok %d Params: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
        cmd.command_type,
        is_planning_control_available(),
        cmd.param1, cmd.param2, cmd.param3,
        cmd.param4, cmd.param5, cmd.param6,
        cmd.param7, cmd.param8, cmd.param9,
        cmd.param10
    );

    // ...existing logic...
}

void SwarmPilot::drone_network_callback(const swarmcomm_msgs::msg::DroneNetworkStatus & status) {
    if (status.quality > 0) {
        swarm_network_status[status.drone_id].quality = status.quality;
    }
    if (status.bandwidth > 0) {
        swarm_network_status[status.drone_id].bandwidth = status.bandwidth;
    }
    if (status.hops > 0) {
        swarm_network_status[status.drone_id].hops = status.hops;
    }
}

// 移除 rclcpp::TimerEvent，使用 node_->now()
void SwarmPilot::network_monitior_timer_callback() {
    rclcpp::Time stamp = node_->now();
    swarmcomm_msgs::msg::SwarmNetworkStatus s_status;
    s_status.header.stamp = stamp;

    for (auto & it : swarm_network_status) {
        swarmcomm_msgs::msg::DroneNetworkStatus d_status;
        d_status.header.stamp = stamp;
        auto & drone_status = it.second;
        auto _id = it.first;
        float dtlast = (stamp.seconds() - drone_status.last_heartbeat.seconds());
        if (drone_status.active && dtlast > heartbeat_timeout) {
            RCLCPP_INFO(node_->get_logger(),
                "[SWARM_PILOT] Drone %d lost at %.1f timeout %.1f/%.1f.",
                _id, stamp.seconds(), dtlast, heartbeat_timeout
            );
            drone_status.active = false;
        }
        d_status.drone_id = _id;
        d_status.quality = drone_status.quality;
        d_status.bandwidth = drone_status.bandwidth;
        d_status.hops = drone_status.hops;
        d_status.active = drone_status.active;
        s_status.node_ids.emplace_back(_id);
        s_status.network_status.push_back(d_status);
    }
    swarm_network_status_pub->publish(s_status);
}
