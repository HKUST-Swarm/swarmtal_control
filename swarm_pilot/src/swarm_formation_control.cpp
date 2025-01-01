#include "swarm_formation_control.h"
#include "swarm_pilot.h"
#include "swarm_pilot_utils.h"

SwarmFormationControl::SwarmFormationControl(int _self_id, SwarmPilot * _pilot, double filter_Ts):
    self_id(_self_id), pilot(_pilot), Ts(filter_Ts) {
}

void SwarmFormationControl::on_swarm_localization(const swarm_msgs::msg::SwarmFused & swarm_fused) {
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

void SwarmFormationControl::on_swarm_basecoor(const swarm_msgs::msg::SwarmDroneBasecoor & swarm_fused) {
    for (size_t i = 0; i < swarm_fused.ids.size(); i++) {
        auto _id = swarm_fused.ids[i];
        auto pos = swarm_fused.drone_basecoor[i];
        auto yaw = swarm_fused.drone_baseyaw[i];
        swarm_transformation[_id] = Swarm::Pose(pos, yaw);
    }
}

void SwarmFormationControl::on_position_command(DroneOnboardCommand cmd, int _id) {
    if (formation_mode <= DroneOnboardCommand::CTRL_FORMATION_IDLE || _id != master_id || _id == self_id ||
    !pilot->is_planning_control_available()) {
        return;
    }

    Eigen::Vector3d pos_sp, vel_sp, acc_sp;
    double yaw_sp;
    
    if (formation_mode == DroneOnboardCommand::CTRL_FORMATION_HOLD_0 
        && swarm_transformation.find(master_id) != swarm_transformation.end()) {
        if (master_id != self_id) {
            if (cmd.command_type - 100 == DroneOnboardCommand::CTRL_POS_COMMAND) {
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

                RCLCPP_INFO(pilot->get_logger(), "[SWARM_PILOT] CTRL_FORMATION_HOLD_0 POS TGT %3.2f %3.2f %3.2f MASTER SP %3.2f %3.2f %3.2f POS %3.2f %3.2f %3.2f DPOS %3.2f %3.2f %3.2f\n", 
                    self_desired_pos.x(), self_desired_pos.y(), self_desired_pos.z(),
                    pos_sp.x(), pos_sp.y(), pos_sp.z(),
                    swarm_pos[master_id].x(), swarm_pos[master_id].y(), swarm_pos[master_id].z(),
                    dpos.x(), dpos.y(), dpos.z());
                
                pilot->send_position_command(self_desired_pos, self_desired_yaw, self_desired_vel);
            } else if (cmd.command_type - 100 == DroneOnboardCommand::CTRL_VEL_COMMAND) {
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

                RCLCPP_INFO(pilot->get_logger(), "[SWARM_PILOT] CTRL_FORMATION_HOLD_0 VEL TGT %3.2f %3.2f %3.2f MASTER POS %3.2f %3.2f %3.2f DPOS %3.2f %3.2f %3.2f\n", 
                    self_desired_vel.x(), self_desired_vel.y(), self_desired_vel.z(),
                    swarm_pos[master_id].x(), swarm_pos[master_id].y(), swarm_pos[master_id].z(),
                    dpos.x(), dpos.y(), dpos.z());
                
                pilot->send_velocity_command(self_desired_vel, self_desired_yaw);
            }
        }
    }


    // if (formation_mode == DroneOnboardCommand::CTRL_FORMATION_HOLD_1 
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

    // if (formation_mode == DroneOnboardCommand::CTRL_FORMATION_FLY_0 && 
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
    formation_mode = DroneOnboardCommand::CTRL_FORMATION_IDLE ;
    RCLCPP_INFO(pilot->get_logger(), "[SWARM_PILOT] Formation fly terminated");
    return;
}

void SwarmFormationControl::set_swarm_formation_mode(uint8_t _formation_mode, int master_id, int sub_mode, Eigen::Vector3d dpos, double dyaw) {
    RCLCPP_INFO(pilot->get_logger(), "[SWARM_PILOT] set_swarm_formation_mode _formation_mode %d master_id %d sub_mode %d self_id %d",
        _formation_mode, master_id, sub_mode, self_id);
    if (!pilot->is_planning_control_available()) {
        return;
    }

    if (_formation_mode == DroneOnboardCommand::CTRL_FORMATION_IDLE && (master_id == -1 || master_id == self_id)) {
        end_formation();
    }

    if (swarm_pos.find(master_id) != swarm_pos.end() 
        && swarm_pos.find(self_id) != swarm_pos.end()) {
        formation_mode = _formation_mode;
    } else {
        RCLCPP_WARN(pilot->get_logger(), "[SWARM_PILOT] Swarm Relative Localization not ready... reject formation");
        return;
    }

    if (formation_mode == DroneOnboardCommand::CTRL_FORMATION_HOLD_0) {
        this->master_id = master_id;
        if (sub_mode == 0) {
            this->dpos = swarm_pos[self_id] - swarm_pos[master_id];
            this->dyaw = -swarm_yaw[self_id] - (-swarm_yaw[master_id]);
        } else if (sub_mode == 1)  {
            this->dpos = dpos;
            this->dyaw = dyaw;
        }
    }

    if (formation_mode == DroneOnboardCommand::CTRL_FORMATION_HOLD_1) {
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

void SwarmFormationControl::on_drone_position_command(DronePosCtrlCmd pos_cmd) {
    if (!pilot->is_planning_control_available()) {
        return;
    }
    // TODO: ros2
    // if (self_id == master_id && formation_mode > DroneOnboardCommand::CTRL_FORMATION_IDLE) {
    //     //Then broadcast this message to all
    //     auto ts = pilot->ROSTIME2LPS(pilot->node_->now());
    //     mavlink_message_t msg;
    //     if (pos_cmd.ctrl_mode == drone_pos_ctrl_cmd::CTRL_CMD_POS_MODE) {
    //         mavlink_msg_swarm_remote_command_pack(self_id, 0, &msg, ts, self_id,
    //             DroneOnboardCommand::CTRL_POS_COMMAND + 100,
    //             pos_cmd.pos_sp.x * 10000,
    //             pos_cmd.pos_sp.y * 10000,
    //             pos_cmd.pos_sp.z * 10000,
    //             pos_cmd.yaw_sp*10000,
    //             pos_cmd.vel_sp.x * 10000,
    //             pos_cmd.vel_sp.y * 10000,
    //             pos_cmd.vel_sp.z * 10000,
    //             pos_cmd.acc_sp.x * 10000,
    //             pos_cmd.acc_sp.y * 10000,
    //             pos_cmd.acc_sp.z * 10000
    //         );
        
    //     } else if (pos_cmd.ctrl_mode == drone_pos_ctrl_cmd::CTRL_CMD_VEL_MODE) {
    //         mavlink_msg_swarm_remote_command_pack(self_id, 0, &msg, ts, self_id,
    //             DroneOnboardCommand::CTRL_VEL_COMMAND + 100,
    //             pos_cmd.vel_sp.x * 10000,
    //             pos_cmd.vel_sp.y * 10000,
    //             pos_cmd.vel_sp.z * 10000,
    //             pos_cmd.yaw_sp*10000,
    //             pos_cmd.acc_sp.x * 10000,
    //             pos_cmd.acc_sp.y * 10000,
    //             pos_cmd.acc_sp.z * 10000,
    //             0,
    //             0,
    //             0
    //         );
    //     }
    //     //Send only by WiFi
    //     pilot->send_mavlink_message(msg, 1);
    // }
}

