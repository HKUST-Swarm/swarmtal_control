#include <swarm_pilot.h>
#include <fstream>
#include <swarmcomm_msgs/swarm_network_status.h>
#include <mavlink/swarm/mavlink_msg_drone_odom_gt.h>

using namespace swarmcomm_msgs;
using namespace swarmtal_msgs;

#define MAXBUFSIZE  ((int) 1e6)

Eigen::MatrixXd readMatrix(const char * filename) {
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE] = {0};

    // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(filename);
    while (! infile.eof())
    {
        std::string line;
        std::getline(infile, line);

        int temp_cols = 0;
        std::stringstream stream(line);
        while(! stream.eof()){
            stream >> buff[cols*rows+temp_cols++];
        }

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;
        if (temp_cols>=5) {
            rows++;
        }
    }

    infile.close();

    // Populate matrix with numbers.
    Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    return result;
};


template <typename T>
inline T lowpass_filter(T input, double Ts, T outputLast, double dt) {
    double alpha = dt / (Ts + dt);
    return outputLast + (alpha * (input - outputLast));
}

SwarmFormationControl::SwarmFormationControl(int _self_id, SwarmPilot * _pilot, double filter_Ts):
    self_id(_self_id), pilot(_pilot), Ts(filter_Ts) {
}


void SwarmFormationControl::on_swarm_traj(const bspline::Bspline & bspl) {
    bspline::Bspline bspl_local = bspl;
    if (bspl.drone_id == self_id) {
        pilot->send_swarm_traj(bspl_local);
        return;
    }

    int _id = bspl.drone_id;
    
    if (swarm_transformation.find(_id) == swarm_transformation.end()) {
        // ROS_WARN("[SWARM_PILOT] Swarm TRAJ for %d could not be transfer, swarm localization not ready", _id);
        return;
    }
    
    auto _cvt = swarm_transformation[_id];
    for (auto & pos : bspl_local.pos_pts) {
        Eigen::Vector3d _pos(pos.x, pos.y, pos.z);
        _pos = _cvt * _pos;
        pos.x = _pos.x();
        pos.y = _pos.y();
        pos.z = _pos.z();
    }

    for (auto & yaw : bspl_local.yaw_pts) {
        yaw = yaw + _cvt.yaw();
    }

    pilot->send_swarm_traj(bspl_local);
}

void SwarmFormationControl::on_swarm_localization(const swarm_msgs::swarm_fused & swarm_fused) {
    for (size_t i = 0; i < swarm_fused.ids.size(); i++) {
        auto _id = swarm_fused.ids[i];
        auto pos = swarm_fused.local_drone_position[i];
        auto vel = swarm_fused.local_drone_velocity[i];
        auto yaw = swarm_fused.local_drone_yaw[i];

        if (swarm_pos.find(_id) != swarm_pos.end()) {
            swarm_pos[_id] = lowpass_filter(Eigen::Vector3d(pos.x, pos.y, pos.z), Ts, swarm_pos[_id], 0.01);
            //swarm_vel[_id] = lowpass_filter(Eigen::Vector3d(vel.x, vel.y, vel.z), Ts, swarm_vel[_id], 0.01);
            swarm_yaw[_id] = lowpass_filter(yaw, Ts, swarm_yaw[_id], 0.01);
        } else {
            swarm_pos[_id] = Eigen::Vector3d(pos.x, pos.y, pos.z);
            swarm_vel[_id] = Eigen::Vector3d(vel.x, vel.y, vel.z);
            swarm_yaw[_id] = yaw;
        }
    }
}

void SwarmFormationControl::on_swarm_basecoor(const swarm_msgs::swarm_drone_basecoor & swarm_fused) {
    for (size_t i = 0; i < swarm_fused.ids.size(); i++) {
        auto _id = swarm_fused.ids[i];
        auto pos = swarm_fused.drone_basecoor[i];
        auto yaw = swarm_fused.drone_baseyaw[i];
        swarm_transformation[_id] = Swarm::Pose(pos, yaw);
    }
}

void SwarmFormationControl::on_position_command(drone_onboard_command cmd, int _id) {
    if (formation_mode <= drone_onboard_command::CTRL_FORMATION_IDLE || _id != master_id || _id == self_id ||
    !pilot->is_planning_control_available()) {
        return;
    }

    Eigen::Vector3d pos_sp, vel_sp, acc_sp;
    double yaw_sp;
    
    if (formation_mode == drone_onboard_command::CTRL_FORMATION_HOLD_0 
        && swarm_transformation.find(master_id) != swarm_transformation.end()) {
        if (master_id != self_id) {
            if (cmd.command_type - 100 == drone_onboard_command::CTRL_POS_COMMAND) {
                pos_sp.x() = cmd.param1/10000.0;
                pos_sp.y() = cmd.param2/10000.0;
                pos_sp.z() = cmd.param3/10000.0;

                yaw_sp = cmd.param4/10000.0;

                vel_sp.x() = cmd.param5/10000.0;
                vel_sp.y() = cmd.param6/10000.0;
                vel_sp.z() = cmd.param7/10000.0;

                acc_sp.x() = cmd.param8/10000.0;
                acc_sp.y() = cmd.param9/10000.0;
                acc_sp.z() = cmd.param10/10000.0;

                auto _cvt = swarm_transformation[master_id];
                Eigen::Vector3d self_desired_pos = _cvt * pos_sp + dpos;
                Eigen::Vector3d self_desired_vel = _cvt.att() * vel_sp;
                Eigen::Vector3d self_desired_acc = _cvt.att() * acc_sp;
                double self_desired_yaw = yaw_sp + _cvt.yaw();

                ROS_INFO("[SWARM_PILOT] CTRL_FORMATION_HOLD_0 POS TGT %3.2f %3.2f %3.2f MASTER SP %3.2f %3.2f %3.2f POS %3.2f %3.2f %3.2f DPOS %3.2f %3.2f %3.2f\n", 
                    self_desired_pos.x(), self_desired_pos.y(), self_desired_pos.z(),
                    pos_sp.x(), pos_sp.y(), pos_sp.z(),
                    swarm_pos[master_id].x(), swarm_pos[master_id].y(), swarm_pos[master_id].z(),
                    dpos.x(), dpos.y(), dpos.z());
                
                pilot->send_position_command(self_desired_pos, self_desired_yaw, self_desired_vel);
            } else if (cmd.command_type - 100 == drone_onboard_command::CTRL_VEL_COMMAND) {
                yaw_sp = cmd.param4/10000.0;

                vel_sp.x() = cmd.param1/10000.0;
                vel_sp.y() = cmd.param2/10000.0;
                vel_sp.z() = cmd.param3/10000.0;

                acc_sp.x() = cmd.param5/10000.0;
                acc_sp.y() = cmd.param6/10000.0;
                acc_sp.z() = cmd.param7/10000.0;

                auto _cvt = swarm_transformation[master_id];
                Eigen::Vector3d self_desired_pos = _cvt * pos_sp + dpos;
                Eigen::Vector3d self_desired_vel = _cvt.att() * vel_sp;
                Eigen::Vector3d self_desired_acc = _cvt.att() * acc_sp;
                double self_desired_yaw = yaw_sp + _cvt.yaw();

                ROS_INFO("[SWARM_PILOT] CTRL_FORMATION_HOLD_0 VEL TGT %3.2f %3.2f %3.2f MASTER POS %3.2f %3.2f %3.2f DPOS %3.2f %3.2f %3.2f\n", 
                    self_desired_vel.x(), self_desired_vel.y(), self_desired_vel.z(),
                    swarm_pos[master_id].x(), swarm_pos[master_id].y(), swarm_pos[master_id].z(),
                    dpos.x(), dpos.y(), dpos.z());
                
                pilot->send_velocity_command(self_desired_vel, self_desired_yaw);
            }
        }
    }


    // if (formation_mode == drone_onboard_command::CTRL_FORMATION_HOLD_1 
    //     && swarm_pos.find(master_id) != swarm_pos.end()) {
    //     if (master_id != self_id) {

    //         Eigen::AngleAxisd R(swarm_yaw[master_id], Eigen::Vector3d::UnitZ());
    //         Eigen::Vector3d self_desired_pos = swarm_pos[master_id] + R*dpos;
    //         Eigen::Vector3d self_desired_vel = R*swarm_vel[master_id];
    //         double self_desired_yaw = -swarm_yaw[master_id] + dyaw;
    //         printf("CTRL_FORMATION_HOLD_1 TGT %3.2f %3.2f %3.2f MASTER POS %3.2f %3.2f %3.2f DPOS %3.2f %3.2f %3.2f\n", 
    //             self_desired_pos.x(), self_desired_pos.y(), self_desired_pos.z(),
    //             swarm_pos[master_id].x(), swarm_pos[master_id].y(), swarm_pos[master_id].z(),
    //             dpos.x(), dpos.y(), dpos.z());
    //         pilot->send_position_command(self_desired_pos, self_desired_yaw, self_desired_vel);
    //     }
    // }

    // if (formation_mode == drone_onboard_command::CTRL_FORMATION_FLY_0 && 
    //     swarm_pos.find(master_id) != swarm_pos.end()) {

    //     Eigen::Vector3d self_desired_pos = swarm_pos[master_id] + dpos;
    //     Eigen::Vector3d self_desired_vel = swarm_vel[master_id];
    //     double self_desired_yaw = -swarm_yaw[master_id] + dyaw;
    //     printf("CTRL_FORMATION_FLY_0 TGT %3.2f %3.2f %3.2f MASTER POS %3.2f %3.2f %3.2f DPOS %3.2f %3.2f %3.2f\n", 
    //         self_desired_pos.x(), self_desired_pos.y(), self_desired_pos.z(),
    //         swarm_pos[master_id].x(), swarm_pos[master_id].y(), swarm_pos[master_id].z(),
    //         dpos.x(), dpos.y(), dpos.z());
    //     pilot->send_position_command(self_desired_pos, self_desired_yaw, self_desired_vel, true);
    // }

}

void SwarmFormationControl::end_formation() {
    formation_mode = drone_onboard_command::CTRL_FORMATION_IDLE ;
    ROS_INFO("[SWARM_PILOT] Formation fly terminated");
    return;
}

void SwarmFormationControl::set_swarm_formation_mode(uint8_t _formation_mode, int master_id, int sub_mode, Eigen::Vector3d dpos, double dyaw) {
    ROS_INFO("[SWARM_PILOT] set_swarm_formation_mode _formation_mode %d master_id %d sub_mode %d self_id %d",
        _formation_mode, master_id, sub_mode, self_id);
    if (!pilot->is_planning_control_available()) {
        return;
    }

    if (_formation_mode == drone_onboard_command::CTRL_FORMATION_IDLE && (master_id == -1 || master_id == self_id)) {
        end_formation();
    }

    if (swarm_pos.find(master_id) != swarm_pos.end() 
        && swarm_pos.find(self_id) != swarm_pos.end()) {
        formation_mode = _formation_mode;
    } else {
        ROS_WARN("[SWARM_PILOT] Swarm Relative Localization not ready... reject formation");
        return;
    }

    if (formation_mode == drone_onboard_command::CTRL_FORMATION_HOLD_0) {
        this->master_id = master_id;
        if (sub_mode == 0) {
            this->dpos = swarm_pos[self_id] - swarm_pos[master_id];
            this->dyaw = -swarm_yaw[self_id] - (-swarm_yaw[master_id]);
        } else if (sub_mode == 1)  {
            this->dpos = dpos;
            this->dyaw = dyaw;
        }
    }

    if (formation_mode == drone_onboard_command::CTRL_FORMATION_HOLD_1) {
        this->master_id = master_id;
        if (sub_mode == 0) {
            Eigen::AngleAxisd R(swarm_yaw[master_id], Eigen::Vector3d::UnitZ());
            this->dpos =  R.inverse()*(swarm_pos[self_id] - swarm_pos[master_id]);
            this->dyaw = -swarm_yaw[self_id] - (-swarm_yaw[master_id]);
        } else if (sub_mode == 1)  {
            this->dpos = dpos;
            this->dyaw = dyaw;
        }
    }
}

void SwarmFormationControl::on_drone_position_command(drone_pos_ctrl_cmd pos_cmd) {
    if (!pilot->is_planning_control_available()) {
        return;
    }
    if (self_id == master_id && formation_mode > drone_onboard_command::CTRL_FORMATION_IDLE) {
        //Then broadcast this message to all
        auto ts = pilot->ROSTIME2LPS(ros::Time::now());
        mavlink_message_t msg;
        if (pos_cmd.ctrl_mode == drone_pos_ctrl_cmd::CTRL_CMD_POS_MODE) {
            mavlink_msg_swarm_remote_command_pack(self_id, 0, &msg, ts, self_id,
                drone_onboard_command::CTRL_POS_COMMAND + 100,
                pos_cmd.pos_sp.x * 10000,
                pos_cmd.pos_sp.y * 10000,
                pos_cmd.pos_sp.z * 10000,
                pos_cmd.yaw_sp*10000,
                pos_cmd.vel_sp.x * 10000,
                pos_cmd.vel_sp.y * 10000,
                pos_cmd.vel_sp.z * 10000,
                pos_cmd.acc_sp.x * 10000,
                pos_cmd.acc_sp.y * 10000,
                pos_cmd.acc_sp.z * 10000
            );
        
        } else if (pos_cmd.ctrl_mode == drone_pos_ctrl_cmd::CTRL_CMD_VEL_MODE) {
            mavlink_msg_swarm_remote_command_pack(self_id, 0, &msg, ts, self_id,
                drone_onboard_command::CTRL_VEL_COMMAND + 100,
                pos_cmd.vel_sp.x * 10000,
                pos_cmd.vel_sp.y * 10000,
                pos_cmd.vel_sp.z * 10000,
                pos_cmd.yaw_sp*10000,
                pos_cmd.acc_sp.x * 10000,
                pos_cmd.acc_sp.y * 10000,
                pos_cmd.acc_sp.z * 10000,
                0,
                0,
                0
            );
        }
        //Send only by WiFi
        pilot->send_mavlink_message(msg, 1);
    }
}


void SwarmPilot::on_uwb_timeref(const sensor_msgs::TimeReference &ref) {
    uwb_time_ref = ref;
}

ros::Time SwarmPilot::LPS2ROSTIME(const int32_t &lps_time) {
    ros::Time base = uwb_time_ref.header.stamp - ros::Duration(uwb_time_ref.time_ref.toSec());
    return base + ros::Duration(lps_time / 1000.0);
}

int32_t SwarmPilot::ROSTIME2LPS(ros::Time ros_time) {
    double lps_t_s = (ros_time - uwb_time_ref.header.stamp).toSec() + uwb_time_ref.time_ref.toSec();
    return (int32_t)(lps_t_s * 1000);
}
 
void SwarmPilot::send_position_command(Eigen::Vector3d pos, double yaw, Eigen::Vector3d vel, bool enable_planning) {
    if (enable_planning) {
        geometry_msgs::PoseStamped pose_tgt;
        pose_tgt.header.stamp = ros::Time::now();
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

        planning_tgt_pub.publish(pose_tgt);
    } else {
        drone_onboard_command cmd;
        cmd.command_type = drone_onboard_command::CTRL_POS_COMMAND;
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

        onboardcmd_pub.publish(cmd);
    }
}


void SwarmPilot::send_velocity_command(Eigen::Vector3d vel, double yaw) {
    drone_onboard_command cmd;
    cmd.command_type = drone_onboard_command::CTRL_VEL_COMMAND;
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

    onboardcmd_pub.publish(cmd);
}

void SwarmPilot::load_missions(ros::NodeHandle & nh) {
    //Now only load mission id 0 from default path
    int mission_num = 0;
    nh.param<int>("mission_num", mission_num, 0);
    std::string mission_path;
    nh.param<std::string>("mission_path", mission_path,  "/home/dji/SwarmConfig/missions/");
    ROS_INFO("[SWARM_PILOT] Trying to load %d missions from %s", mission_num, mission_path.c_str());
    char mission_i_path[100] = {0};
    for (int i = 0; i < mission_num; i ++) {
        sprintf(mission_i_path, "%s/mission_%d_drone%d.csv", mission_path.c_str(), i, self_id);
        auto mission = readMatrix(mission_i_path);
        Eigen::Matrix<double, Eigen::Dynamic, 1> mission_traj_t = mission.block(0, 0, mission.rows(), 1);
        Eigen::Matrix<double, Eigen::Dynamic, 4> mission_traj = mission.block(0, 1, mission.rows(), 4);
        if (mission_traj_t.rows() > 0) {
            mission_trajs_t.push_back(mission_traj_t);
            mission_trajs.push_back(mission_traj);
            ROS_INFO("[SWARM_PILOT] Loaded mission %d from %s pts: %d duration: %3.1fs", i, mission_i_path, mission_traj_t.rows(), mission_traj_t(mission_traj_t.rows() - 1));
        }
    }
}

SwarmPilot::SwarmPilot(ros::NodeHandle & _nh):
    nh(_nh) {
    double Ts;
    nh.param<bool>("planning_debug_mode", planning_debug_mode, false);
    nh.param<int>("drone_id", self_id, -1);
    nh.param<int>("acpt_cmd_node", accept_cmd_node_id, -1);
    nh.param<double>("send_drone_status_freq", send_drone_status_freq, 5);
    nh.param<double>("Ts", Ts, 0.1);
    nh.param<double>("heartbeat_timeout", heartbeat_timeout, 0.5);
    nh.param<bool>("enable_planner", enable_planner, false);

    assert(self_id > 0 && "Self ID must bigger than 0!!!");
    load_missions(nh);

    formation_control = new SwarmFormationControl(self_id, this, Ts);

    uwb_timeref_sub = nh.subscribe("/uwb_node/time_ref", 1, &SwarmPilot::on_uwb_timeref, this, ros::TransportHints().tcpNoDelay());

    onboardcmd_pub = nh.advertise<drone_onboard_command>("/drone_commander/onboard_command", 1);
    uwb_send_pub = nh.advertise<data_buffer>("/uwb_node/send_broadcast_data", 10);
    planning_tgt_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/goal", 10);
    exprolaration_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    swarm_traj_pub = nh.advertise<bspline::Bspline>("/planning/swarm_traj", 10);
    swarm_network_status_pub = nh.advertise<swarmcomm_msgs::swarm_network_status>("/swarm_drones/swarm_network_status", 10);
    
    traj_pub = nh.advertise<std_msgs::Int8>("/swarm_traj_start_trigger", 1);
    incoming_data_sub = nh.subscribe("/uwb_node/incoming_broadcast_data", 10, &SwarmPilot::incoming_broadcast_data_sub, this, ros::TransportHints().tcpNoDelay());
    drone_cmd_state_sub = nh.subscribe("/drone_commander/swarm_commander_state", 1, &SwarmPilot::on_drone_commander_state, this, ros::TransportHints().tcpNoDelay());
    swarm_local_sub = nh.subscribe("/swarm_drones/swarm_drone_fused", 1, &SwarmFormationControl::on_swarm_localization, formation_control, ros::TransportHints().tcpNoDelay());
    swarm_basecoor_sub = nh.subscribe("/swarm_drones/swarm_drone_basecoor", 1, &SwarmFormationControl::on_swarm_basecoor, formation_control, ros::TransportHints().tcpNoDelay());
    swarm_traj_sub = nh.subscribe("/planning/swarm_traj_recv", 10, &SwarmFormationControl::on_swarm_traj, formation_control, ros::TransportHints().tcpNoDelay());
    uwb_remote_sub = nh.subscribe("/uwb_node/remote_nodes", 1, &SwarmPilot::on_uwb_remote_node, this, ros::TransportHints().tcpNoDelay());
    local_cmd_sub = nh.subscribe("/drone_position_control/drone_pos_cmd", 1, &SwarmFormationControl::on_drone_position_command, formation_control, ros::TransportHints().tcpNoDelay());
    drone_network_sub = nh.subscribe("/swarm_loop/drone_network_status", 1, &SwarmPilot::drone_network_callback, this, ros::TransportHints().tcpNoDelay());
    odom_sub = nh.subscribe("/d2vins/imu_propagation", 1, &SwarmPilot::on_odometry, this, ros::TransportHints().tcpNoDelay());
    eight_trajectory_timer = nh.createTimer(ros::Duration(0.02), &SwarmPilot::timer_callback, this);
    net_timer = nh.createTimer(ros::Duration(0.02), &SwarmPilot::network_monitior_timer_callback, this);

    last_send_drone_status = ros::Time(0);
    last_send_odom = ros::Time::now();
    
    ROS_INFO("[SWARM_PILOT] Node %d ready", self_id);
}

bool SwarmPilot::is_planning_control_available() {
    if (planning_debug_mode) {
        return true;
    } else {
        return cmd_state.control_auth == drone_commander_state::CTRL_AUTH_THIS
            && cmd_state.flight_status == drone_commander_state::FLIGHT_STATUS_IN_AIR;
    }
}

void SwarmPilot::timer_callback(const ros::TimerEvent &e) {
    if (is_planning_control_available()) {
        if (mission_trajs_enable) {
            mission_trajs_timer_callback(e);
        } else if(eight_trajectory_enable) {
            eight_trajectory_timer_callback(e);
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

void SwarmPilot::mission_trajs_timer_callback(const ros::TimerEvent &e) {
    if (mission_trajs_enable && cur_mission_id < mission_trajs_t.size()) {
        auto & mission_traj_t = mission_trajs_t[cur_mission_id];
        auto & mission_traj = mission_trajs[cur_mission_id];
        bool need_send_planning_cmd = false;

        while (cur_mission_index < 0 || cur_mission_index < mission_traj_t.rows() - 1 && mission_traj_t[cur_mission_index + 1] <  mission_trajectory_timer_t) {
            cur_mission_index++;
            need_send_planning_cmd = true;
        }
        
        mission_trajectory_timer_t+= 0.02;

        if (!need_send_planning_cmd) {
            return;
        }

        //Execute current target position by fast planner.
        geometry_msgs::PoseStamped pose_tgt;
        pose_tgt.header.stamp = ros::Time::now();
        pose_tgt.header.frame_id = "world";
        
        pose_tgt.pose.position.x = mission_traj(cur_mission_index, 0);
        pose_tgt.pose.position.y = mission_traj(cur_mission_index, 1);
        pose_tgt.pose.position.z = mission_traj(cur_mission_index, 2);

        double yaw_sp = 0;
        if (mission_trajs_yaw_enable) {
            mission_traj(cur_mission_index, 3);
        }

        Eigen::Quaterniond _quat(Eigen::AngleAxisd(-yaw_sp, Eigen::Vector3d::UnitZ()));
        
        // pose_tgt.
        pose_tgt.pose.orientation.w = _quat.w();
        pose_tgt.pose.orientation.x = _quat.x();
        pose_tgt.pose.orientation.y = _quat.y();
        pose_tgt.pose.orientation.z = _quat.z();
        ROS_INFO("[SWARM_PILOT] Mission index %d traj navigate to [%3.2f, %3.2f, %3.2f]", cur_mission_index, pose_tgt.pose.position.x, pose_tgt.pose.position.y, pose_tgt.pose.position.z);
        planning_tgt_pub.publish(pose_tgt);

        if (cur_mission_index >= mission_traj_t.rows() - 1) {
            ROS_INFO("[SWARM_PILOT] Mission_trajs finish index %d t %3.1fs.", cur_mission_index, mission_trajectory_timer_t);
            mission_trajs_enable = false;
            cur_mission_index = -1;
            mission_trajectory_timer_t = 0;
            return;
        }

    }
}

void SwarmPilot::eight_trajectory_timer_callback(const ros::TimerEvent &e) {
    if (!eight_trajectory_enable || !is_planning_control_available()) {
        eight_trajectory_enable = false;
        mission_trajectory_timer_t = 0.0;
        return;
    }

    double T = eight_trajectory_timer_period;
    double ox = 0.0, oy = 0.0, oz = 1.0;
    
    double _t = mission_trajectory_timer_t* 2*M_PI/T;
    double x, y, vx, vy, ax, ay, yaw;
    ox = eight_trajectory_center(0);
    oy = eight_trajectory_center(1);
    oz = eight_trajectory_center(2);
    if (eight_traj_mode == 0) {
        x = ox + 2 * sin(_t);
        y = oy + 2 * sin(_t) * cos(_t);
        vx = 2 * cos(_t) * M_PI * 2/T;
        vy = 2 * cos(2*_t) * M_PI * 2/T;
        ax = -2 * sin(_t) * M_PI * 2/T * M_PI * 2/T;
        ay = -2 * sin(2*_t) * M_PI * 2/T * 2 * M_PI * 2/T;
        yaw = atan2(-cos(2*_t),cos(_t));
    } else {
        x = ox + 2 * sin(_t) * cos(_t);
        y = oy + 2 * sin(_t);
        vy = 2 * cos(_t) * M_PI * 2/T;
        vx = 2 * cos(2*_t) * M_PI * 2/T;
        ay = -2 * sin(_t) * M_PI * 2/T * M_PI * 2/T;
        ax = -2 * sin(2*_t) * M_PI * 2/T * 2 * M_PI * 2/T;
        yaw = atan2(cos(_t),-cos(2*_t));
    }

    drone_onboard_command onboardCommand;
    // onboardCommand.command_type = cmd.command_type;
    // xyz
    onboardCommand.param1 = x * 10000;
    onboardCommand.param2 = y * 10000;
    onboardCommand.param3 = oz * 10000;
    // yaw
    if (eight_trajectory_yaw_enable) {
        if (_t < 0.5) {
            yaw = _t * 2 * yaw;
            onboardCommand.param4 = yaw*10000;
        } else {
            onboardCommand.param4 = yaw*10000;
        }
    }
    else {
        onboardCommand.param4 = 666666;  
    }

    // vel feedforward
    onboardCommand.param5 = vx * 10000;
    onboardCommand.param6 = vy * 10000;
    onboardCommand.param7 = 0;
    // acc feedforward
    onboardCommand.param8 = ax * 10000;
    onboardCommand.param9 = ay * 10000;
    onboardCommand.param10 = 0;

    onboardcmd_pub.publish(onboardCommand);

    mission_trajectory_timer_t += 0.02;

    // ROS_INFO("Time: %f, Eight trajectory x: %f, y: %f, vx: %f, vy:%f, ax: %f, ay: %f", eight_trajectory_timer_t, x, y, vx, vy, ax, ay);

    return;
}

void SwarmPilot::start_mission_trajs(const drone_onboard_command & cmd) {
    if (!mission_trajs_enable) {
        cur_mission_id = cmd.param1;
        mission_trajs_yaw_enable = cmd.param2>0;
        if (cur_mission_id >= mission_trajs_t.size()) {
            ROS_WARN("[SWAMR_PILOT] start_mission_trajs rejected. Mission %d not loaded.", cur_mission_id);
            return;
        }

        mission_trajs_enable = true;
        mission_trajectory_timer_t = 0;
        cur_mission_index = -1;
                
        ROS_INFO("[SWAMR_PILOT] start_mission_trajs mission %d enable_yaw %d", cur_mission_id, mission_trajs_yaw_enable);
    }
}

void SwarmPilot::start_spec_trajs(const drone_onboard_command & cmd) {
    if (cmd.param1 == 1) {
        eight_trajectory_enable = true;
        eight_trajectory_yaw_enable = cmd.param2;
        eight_trajectory_timer_period = cmd.param3/10000.0;
        eight_traj_mode = cmd.param7;
        mission_trajectory_timer_t = 0;
        eight_trajectory_center = Eigen::Vector3d(cmd.param4/10000.0, cmd.param5/10000.0, cmd.param6/10000.0);
        ROS_INFO("[SWAMR_PILOT] Start 8 trajectort: enable Yaw: %d mode %d, T %3.1f center [%3.2f, %3.2f, %3.2f]",
            eight_trajectory_yaw_enable, eight_traj_mode, eight_trajectory_timer_period,
            eight_trajectory_center.x(), eight_trajectory_center.y(), eight_trajectory_center.z());
    }
}


void SwarmPilot::end_mission() {
    if (mission_trajs_enable || eight_trajectory_enable) {
        mission_trajs_enable = false;
        eight_trajectory_enable = false;
        mission_trajectory_timer_t = 0;
        cur_mission_index = -1;
        formation_control->end_formation();
        ROS_INFO("[SWARM_PILOT] Terminate current mission.");
    }
}

void SwarmPilot::on_uwb_remote_node(const remote_uwb_info & info) {
    self_id = info.self_id;
}

void SwarmPilot::send_start_exploration(const drone_onboard_command & cmd) {
    geometry_msgs::PoseStamped pose_tgt;
    pose_tgt.header.stamp = ros::Time::now();
    pose_tgt.header.frame_id = "world";
    exprolaration_pub.publish(pose_tgt);
    ROS_INFO("[SWAMR_PILOT] Sending exploration command.");

}

void SwarmPilot::send_planning_command(drone_onboard_command cmd) {
    if (cmd.command_type == drone_onboard_command::CTRL_PLANING_TGT_COMMAND &&
        is_planning_control_available()) {
        if (enable_planner) {
            geometry_msgs::PoseStamped pose_tgt;
            pose_tgt.header.stamp = ros::Time::now();
            pose_tgt.header.frame_id = "world";
            
            pose_tgt.pose.position.x = cmd.param1 / 10000.0;
            pose_tgt.pose.position.y = cmd.param2 / 10000.0;
            pose_tgt.pose.position.z = cmd.param3 / 10000.0;

            Eigen::Quaterniond _quat(Eigen::AngleAxisd(-cmd.param4/10000.0, Eigen::Vector3d::UnitZ()));
            // pose_tgt.
            pose_tgt.pose.orientation.w = _quat.w();
            pose_tgt.pose.orientation.x = _quat.x();
            pose_tgt.pose.orientation.y = _quat.y();
            pose_tgt.pose.orientation.z = _quat.z();
            ROS_INFO("[SWAMR_PILOT] Sending traj fly to [%3.2f, %3.2f, %3.2f]", pose_tgt.pose.position.x, pose_tgt.pose.position.y, pose_tgt.pose.position.z);
            planning_tgt_pub.publish(pose_tgt);
        } else {
            cmd.command_type = drone_onboard_command::CTRL_POS_COMMAND;
            onboardcmd_pub.publish(cmd);
        }
    } else {
        ROS_WARN("[SWAMR_PILOT] Reject fly to. Planning not ready.");
    }
}

void SwarmPilot::send_swarm_traj(const bspline::Bspline & bspl) {
    swarm_traj_pub.publish(bspl);
}

void SwarmPilot::on_mavlink_msg_remote_cmd(ros::Time stamp, int node_id, const mavlink_swarm_remote_command_t & cmd) {
    // if (cmd.command_type >= 100) {
    //     traj_mission_callback(cmd.command_type);
    //     return;
    // }

    drone_onboard_command onboardCommand;

    ROS_INFO("[SWAMR_PILOT] Recv onboard cmd from %d target %d type %d is planning ok %d Params 1-3: %d %d %d 4-6: %d %d %d, 7-10: %d %d %d %d",
                node_id, cmd.target_id,
                cmd.command_type,
                is_planning_control_available(),
                cmd.param1, cmd.param2, cmd.param3,
                cmd.param4, cmd.param5, cmd.param6,
                cmd.param7, cmd.param8, cmd.param9,
                cmd.param10);
    
    onboardCommand.command_type = cmd.command_type;
    onboardCommand.param1 = cmd.param1;
    onboardCommand.param2 = cmd.param2;
    onboardCommand.param3 = cmd.param3;
    onboardCommand.param4 = cmd.param4;
    onboardCommand.param5 = cmd.param5;
    onboardCommand.param6 = cmd.param6;
    onboardCommand.param7 = cmd.param7;
    onboardCommand.param8 = cmd.param8;
    onboardCommand.param9 = cmd.param9;
    onboardCommand.param10 = cmd.param10;

    if (cmd.command_type >= 100) {
        //This is proxied pos ctrl cmd for formation control
        formation_control->on_position_command(onboardCommand, cmd.target_id);
        return;
    }

    if (node_id != accept_cmd_node_id && accept_cmd_node_id >= 0) {
        ROS_WARN("[SWAMR_PILOT] Command from unacceptable drone %d, reject", node_id);
        return;
    } 
    if (cmd.target_id >=0 && cmd.target_id != self_id) {
        ROS_WARN("[SWAMR_PILOT] Control target %d not this drone, reject", cmd.target_id);            
        return;
    }

    // In this case, we should end the eight traj to let other command works.
    end_mission();
    switch (cmd.command_type) {
        case drone_onboard_command::CTRL_SPEC_TRAJS: {
            if (is_planning_control_available()) {
                start_spec_trajs(onboardCommand);
            } else {
                ROS_WARN("[SWAMR_PILOT] CTRL_SPEC_TRAJS command reject: Planning control not available.");
            }
            break;
        }
        case drone_onboard_command::CTRL_FORMATION_IDLE:
        case drone_onboard_command::CTRL_FORMATION_HOLD_0:
        case drone_onboard_command::CTRL_FORMATION_HOLD_1:
        case drone_onboard_command::CTRL_FORMATION_FLY_0:
        case drone_onboard_command::CTRL_FORMATION_FLY_1: {
            if (is_planning_control_available()) {
                formation_control->set_swarm_formation_mode(onboardCommand.command_type, onboardCommand.param1, onboardCommand.param2,
                    Eigen::Vector3d(onboardCommand.param3/10000, onboardCommand.param4/10000, onboardCommand.param5/10000),
                    onboardCommand.param6/10000
                );
            } else {
                ROS_WARN("[SWAMR_PILOT] Planning command reject: Planning control not available.");
            }
            break;
        }
        case drone_onboard_command::CTRL_PLANING_TGT_COMMAND: {
            if(is_planning_control_available()) {
                send_planning_command(onboardCommand);
            } else {
                ROS_WARN("[SWAMR_PILOT] Planning command reject: Planning control not available.");
            }
            break;
        } 
        case drone_onboard_command::CTRL_TASK_EXPROLARATION: {
            if (is_planning_control_available()) {
                send_start_exploration(onboardCommand);
            } else {
                ROS_WARN("[SWAMR_PILOT] Expo command reject: Planning control not available.");
            }
            break;
        }
        case drone_onboard_command::CTRL_MISSION_TRAJS: {
            if (is_planning_control_available()) {
                start_mission_trajs(onboardCommand);
            } else {
                ROS_WARN("[SWAMR_PILOT] CTRL_MISSION_TRAJS reject: Planning control not available.");
            }
            break;
        }
        case drone_onboard_command::CTRL_END_MISSION: {
            end_mission();
            break;
        } 
        default:
            ROS_INFO("[SWAMR_PILOT] Command proxy to drone_commander.");
            onboardcmd_pub.publish(onboardCommand);
    }
}

void SwarmPilot::on_drone_commander_state(const drone_commander_state & _state) {
    mavlink_message_t msg;
    if (self_id < 0) {
        ROS_WARN("[SWAMR_PILOT] Don't have self id, unable to send drone status");
    }
    cmd_state = _state;

    if ((ros::Time::now() - last_send_drone_status).toSec() > (1.0 / send_drone_status_freq) ) {
        last_send_drone_status = ros::Time::now();
        // _state.bat_remain = 345.375f * _state.bat_vol - 4757.3;
        mavlink_msg_drone_status_pack(self_id, 0, &msg, ROSTIME2LPS(ros::Time::now()), _state.flight_status, _state.control_auth,
                _state.commander_ctrl_mode, _state.ctrl_input_state, _state.rc_valid, _state.onboard_cmd_valid, _state.djisdk_valid,
                _state.vo_valid,_state.vo_latency, _state.bat_vol, _state.bat_remain, _state.pos.x, _state.pos.y, _state.pos.z, _state.yaw);
        // ROS_INFO_THROTTLE(1.0, "[SWAMR_PILOT] Sending swarm status");
        ROS_INFO("[SWAMR_PILOT] Sending swarm status");
        send_mavlink_message(msg);
    }

}

void SwarmPilot::on_odometry(const nav_msgs::Odometry & odom) {
    if ((ros::Time::now() - last_send_odom).toSec() < 1/send_odom_freq) {
        return;
    }
    last_send_odom = ros::Time::now();
    mavlink_message_t msg;
    auto pos = odom.pose.pose.position;
    auto quat = odom.pose.pose.orientation;
    auto vel = odom.twist.twist.linear;
    mavlink_msg_drone_odom_gt_pack(self_id, 0, &msg, ROSTIME2LPS(ros::Time::now()), 
        self_id, pos.x*1000, pos.y*1000, pos.z*1000,
        quat.w*10000, quat.x*10000, quat.y*10000, quat.z*10000, 
        vel.x*1000, vel.y*1000, vel.z*1000);
    send_mavlink_message(msg);
}

void SwarmPilot::send_mavlink_message(mavlink_message_t & msg, int send_method) {
    int len = mavlink_msg_to_send_buffer(buf , &msg);
    data_buffer buffer;
    buffer.data = std::vector<uint8_t>(buf, buf+len);
    buffer.send_method = send_method;
    uwb_send_pub.publish(buffer);
}

void SwarmPilot::incoming_broadcast_data_callback(std::vector<uint8_t> data, int sender_drone_id, ros::Time stamp) {
//        ROS_INFO("Recv incoming msg %d", data.data.size());
    mavlink_message_t msg;
    mavlink_status_t status;
    for (size_t i = 0; i <data.size(); i++){
        uint8_t  c = data[i];
        if (mavlink_parse_char(0, c, &msg, &status)) {
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND:
                    mavlink_swarm_remote_command_t cmd;
                    mavlink_msg_swarm_remote_command_decode(&msg, &cmd);
                    on_mavlink_msg_remote_cmd(stamp, sender_drone_id, cmd);
                    break;
                case MAVLINK_MSG_ID_DRONE_STATUS:
                    mavlink_drone_status_t status;
                    mavlink_msg_drone_status_decode(&msg, &status);
                    on_mavlink_drone_status(stamp, sender_drone_id, status);
                default:
                    break;
            }
        }
    }
}

void SwarmPilot::on_mavlink_drone_status(ros::Time stamp, int node_id, const mavlink_drone_status_t & msg) {
    if (swarm_network_status.find(node_id) == swarm_network_status.end()) {
        ROS_INFO("[SWARM_PILOT] Drone %d connected at %.1fs.", node_id, stamp.toSec());
    }
    if (!swarm_network_status[node_id].active) {
        ROS_INFO("[SWARM_PILOT] Drone %d reconnected %.1fs.", node_id, stamp.toSec());
    }
    swarm_network_status[node_id].active = true;
    swarm_network_status[node_id].last_heartbeat = ros::Time::now();
}

void SwarmPilot::incoming_broadcast_data_sub(const incoming_broadcast_data & data) {
    incoming_broadcast_data_callback(data.data, data.remote_id, data.header.stamp);
}

void SwarmPilot::drone_network_callback(const swarmcomm_msgs::drone_network_status & status) {
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


void SwarmPilot::network_monitior_timer_callback(const ros::TimerEvent &e) {
    ros::Time stamp = e.current_real;
    swarmcomm_msgs::swarm_network_status s_status;
    s_status.header.stamp = stamp;
    for (auto & it : swarm_network_status) {
        swarmcomm_msgs::drone_network_status d_status;
        d_status.header.stamp = stamp;
        auto & drone_status = it.second;
        auto _id = it.first;
        float dtlast = (stamp - drone_status.last_heartbeat).toSec();
        if (drone_status.active && dtlast > heartbeat_timeout) {
            ROS_INFO("[SWARM_PILOT] Drone %d lost at %.1f timeout %.1f/%.1f.", _id, stamp.toSec(), dtlast, heartbeat_timeout);
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
    swarm_network_status_pub.publish(s_status);
}
