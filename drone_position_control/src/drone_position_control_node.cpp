#include <iostream>
#include <drone_position_control/rotor_position_control.h>
#include "ros/ros.h"
#include "stdio.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <ctime>
#include <time.h>
#include <stdlib.h>
#include <swarmtal_msgs/drone_pos_control_state.h>
#include <swarmtal_msgs/drone_pos_ctrl_cmd.h>
#include <sensor_msgs/Joy.h>
#include <drone_position_control/swarm_util.h>
#include <sensor_msgs/Imu.h>
#include <swarmtal_msgs/drone_onboard_command.h>
#include <swarmtal_msgs/drone_commander_state.h>
#include <mavros_msgs/PositionTarget.h>

using namespace swarmtal_msgs;
#define MAX_CMD_LOST_TIME 0.5f
// #define USE_DJI_THRUST_CTRL
#define ANGULARRATE_MIX 0.9
#define MAX_ACC 8 // max accerelation

#define PX4 0 
#define DJI_SDK 1
// #define FCHardware DJI_SDK
#define FCHardware PX4

#if FCHardware == PX4
#include <mavros_msgs/AttitudeTarget.h>
#endif



class DronePosControl {
    ros::NodeHandle & nh;
    
    ros::Subscriber odom_sub, commander_state_sub;
    ros::Subscriber drone_pos_cmd_sub;
    ros::Subscriber fc_att_sub;
    ros::Subscriber imu_data_sub;
    ros::Subscriber sub_atti_target;

    RotorPositionControl * pos_ctrl = nullptr;

    swarmtal_msgs::drone_pos_control_state state;
    swarmtal_msgs::drone_commander_state commander_state;

    Eigen::Vector3d pos_sp = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel_sp = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel_ff = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d acc_ff = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d acc_sp = Eigen::Vector3d(0, 0, 0);

    Eigen::Vector3d odom_gc_pos = Eigen::Vector3d(0, 0, 0);

    Eigen::Vector3d angular_rate = Eigen::Vector3d(0, 0, 0);

    Quaterniond att_sp;
    float z_sp = 0;

    //RPY, this is in NED
    Eigen::Vector3d odom_att_rpy;
    Eigen::Vector3d fc_att_rpy;
    Eigen::Vector3d fc_att_target;


    ros::Publisher state_pub;
    ros::Publisher control_pub, control_pos_vel_px4_pub;

    std::string log_path;

    double yaw_offset = 0;
    double yaw_odom = 0;
    double yaw_fc = 0;
    double thrust_limit;
    double filtered_thrust = 0;

    ros::Time last_cmd_ts, start_time;

    AttiCtrlOut atti_out;


    FILE * log_file;

    void recordCSV() {
        if (log_file == nullptr) {
            return;
        }
        fprintf(
                      //0 1 2  3  4  5  6  7   8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30
            log_file, "%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                (ros::Time::now() - start_time).toSec(),//1
                state.ctrl_mode,//2
                state.pose.position.x,state.pose.position.y,state.pose.position.z,//5
                state.global_vel.x,state.global_vel.y,state.global_vel.z,//8
                odom_att_rpy.x(), odom_att_rpy.y(), odom_att_rpy.z(),//11 in FLU
                pos_sp.x(), pos_sp.y(), pos_sp.z(),//14
                vel_sp.x(), vel_sp.y(), vel_sp.z(),//17
                acc_sp.x(), acc_sp.y(), acc_sp.z(),//20
                atti_out.roll_sp, atti_out.pitch_sp, atti_out.yaw_sp,//23 in NED
                atti_out.thrust_sp,//24,
                fc_att_rpy.x(), fc_att_rpy.y(), fc_att_rpy.z(),//27 in NED
                angular_rate.x(), angular_rate.y(), angular_rate.z(), //30
                state.imu_data.linear_acceleration.x, state.imu_data.linear_acceleration.y, state.imu_data.linear_acceleration.z, //33
                vel_ff.x(), vel_ff.y(), vel_ff.z()
            );
        fflush(log_file);
        
    }

    void read_controller_param(RotorPosCtrlParam & ctrlP) {
        nh.param<double>("pid_param/p_x/p", ctrlP.p_x.p, 0);
        nh.param<double>("pid_param/p_x/i", ctrlP.p_x.i, 0);
        nh.param<double>("pid_param/p_x/d", ctrlP.p_x.d, 0);
        nh.param<double>("pid_param/p_x/b", ctrlP.p_x.b, 1.0);
        nh.param<double>("pid_param/p_x/c", ctrlP.p_x.c, 1.0);
        nh.param<double>("pid_param/p_x/tf", ctrlP.p_x.tf, 0);
        nh.param<double>("pid_param/p_x/max_i", ctrlP.p_x.max_err_i, 30);

        nh.param<double>("pid_param/p_y/p", ctrlP.p_y.p, 0);
        nh.param<double>("pid_param/p_y/i", ctrlP.p_y.i, 0);
        nh.param<double>("pid_param/p_y/d", ctrlP.p_y.d, 0);
        nh.param<double>("pid_param/p_y/b", ctrlP.p_y.b, 1.0);
        nh.param<double>("pid_param/p_y/c", ctrlP.p_y.c, 1.0);
        nh.param<double>("pid_param/p_y/tf", ctrlP.p_y.tf, 0);
        nh.param<double>("pid_param/p_y/max_i", ctrlP.p_y.max_err_i, 30);

        nh.param<double>("pid_param/p_z/p", ctrlP.p_z.p, 0);
        nh.param<double>("pid_param/p_z/i", ctrlP.p_z.i, 0);
        nh.param<double>("pid_param/p_z/d", ctrlP.p_z.d, 0);
        nh.param<double>("pid_param/p_z/b", ctrlP.p_z.b, 1.0);
        nh.param<double>("pid_param/p_z/c", ctrlP.p_z.c, 1.0);
        nh.param<double>("pid_param/p_z/tf", ctrlP.p_z.tf, 0);
        nh.param<double>("pid_param/p_z/max_i", ctrlP.p_z.max_err_i, 30);

        nh.param<double>("pid_param/v_x/p", ctrlP.v_x.p, 0);
        nh.param<double>("pid_param/v_x/i", ctrlP.v_x.i, 0);
        nh.param<double>("pid_param/v_x/d", ctrlP.v_x.d, 0);
        nh.param<double>("pid_param/v_x/b", ctrlP.v_x.b, 1.0);
        nh.param<double>("pid_param/v_x/c", ctrlP.v_x.c, 1.0);
        nh.param<double>("pid_param/v_x/tf", ctrlP.v_x.tf, 0);
        nh.param<double>("pid_param/v_x/max_i", ctrlP.v_x.max_err_i, 15);

        nh.param<double>("pid_param/v_y/p", ctrlP.v_y.p, 0);
        nh.param<double>("pid_param/v_y/i", ctrlP.v_y.i, 0);
        nh.param<double>("pid_param/v_y/d", ctrlP.v_y.d, 0);
        nh.param<double>("pid_param/v_y/b", ctrlP.v_y.b, 1.0);
        nh.param<double>("pid_param/v_y/c", ctrlP.v_y.c, 1.0);
        nh.param<double>("pid_param/v_y/tf", ctrlP.v_y.tf, 0);
        nh.param<double>("pid_param/v_y/max_i", ctrlP.v_y.max_err_i, 15);


        nh.param<double>("pid_param/v_z/p", ctrlP.v_z.p, 0);
        nh.param<double>("pid_param/v_z/i", ctrlP.v_z.i, 0);
        nh.param<double>("pid_param/v_z/d", ctrlP.v_z.d, 0);
        nh.param<double>("pid_param/v_z/b", ctrlP.v_z.b, 1.0);
        nh.param<double>("pid_param/v_z/c", ctrlP.v_z.c, 1.0);
        nh.param<double>("pid_param/v_z/tf", ctrlP.v_z.tf, 0);
        nh.param<double>("pid_param/v_z/max_i", ctrlP.v_z.max_err_i, 10);

        nh.param<double>("pid_param/thr/max_i", ctrlP.thrust_ctrl.abx.max_err_i, 0);
        nh.param<double>("pid_param/thr/level_thrust", ctrlP.thrust_ctrl.level_thrust, 0.15);
        nh.param<double>("pid_param/thr/thrust_limit", thrust_limit, 0.3);

        nh.param<double>("odom_gc_pos/x", odom_gc_pos.x(), 0);
        nh.param<double>("odom_gc_pos/y", odom_gc_pos.y(), 0);
        nh.param<double>("odom_gc_pos/z", odom_gc_pos.z(), 0);
    }
    ros::Timer control_timer;
    Eigen::Matrix3d R_ENU2NED;
    Eigen::Matrix3d R_FLU2FRD; 

    Eigen::Quaterniond FLU2NED(const Eigen::Quaterniond & q) {
        Matrix3d R = R_FLU2FRD * q.toRotationMatrix() * R_FLU2FRD;
        return Eigen::Quaterniond(R);
    }

    Eigen::Quaterniond ENU2NED(const Eigen::Quaterniond & q) {
        Matrix3d R = R_ENU2NED*q.toRotationMatrix()*R_FLU2FRD;
        return Eigen::Quaterniond(R);
    }

    Eigen::Quaterniond NED2ENU(const Eigen::Quaterniond & q) {
        Matrix3d R = R_ENU2NED.transpose()*q.toRotationMatrix()*R_FLU2FRD;
        return Eigen::Quaterniond(R);
    }

public:
    DronePosControl(ros::NodeHandle & _nh):
    nh(_nh) {
        R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;
        R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;

        RotorPosCtrlParam ctrlP;

        ctrlP.ctrl_frame = CTRL_FRAME::VEL_WORLD_ACC_WORLD;
        ctrlP.coor_sys = FRAME_COOR_SYS::FLU;

        read_controller_param(ctrlP);

        ROS_INFO("Init pos control");
        pos_ctrl = new RotorPositionControl(ctrlP);
        ROS_INFO("Pos control success");

        control_timer = nh.createTimer(ros::Duration(0.02), &DronePosControl::control_update, this);
        
        nh.param<std::string>("log_path", log_path, "/home/dji/drone_log_latest");

        init_log_file();

        state_pub = nh.advertise<swarmtal_msgs::drone_pos_control_state>("drone_pos_control_state", 10);
        
        odom_sub = nh.subscribe("odometry", 1 , &DronePosControl::OnVisualOdometry, this, ros::TransportHints().tcpNoDelay());
        drone_pos_cmd_sub = nh.subscribe("drone_pos_cmd", 1 , &DronePosControl::OnSwarmPosCommand, this, ros::TransportHints().tcpNoDelay());
        fc_att_sub = nh.subscribe("fc_attitude", 1, &DronePosControl::onFCAttitude, this, ros::TransportHints().tcpNoDelay());
        imu_data_sub = nh.subscribe("fc_imu", 1, &DronePosControl::on_imu_data, this, ros::TransportHints().tcpNoDelay());
        commander_state_sub = nh.subscribe("/drone_commander/swarm_commander_state", 1, &DronePosControl::on_commander_state, this, ros::TransportHints().tcpNoDelay());
#if FCHardware == DJI_SDK
        control_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk_control", 1);
#else
        control_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
        control_pos_vel_px4_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
        sub_atti_target = nh.subscribe("/mavros/setpoint_raw/target_attitude", 1, &DronePosControl::onFCAttitudeTarget, this, ros::TransportHints().tcpNoDelay());
#endif
        start_time = last_cmd_ts = ros::Time::now();
    }   

    void init_log_file() {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80] = {0};
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        int r = rand();  
        char str[100] = {0};

        sprintf(buffer, "/home/dji/swarm_log_latest/control.csv", r);

        FILE* flog_list = fopen("/home/dji/swarm_log_latest/log_list.txt", "a");
        if (flog_list != nullptr) {
            fprintf(flog_list,"%s\n", buffer);
            fflush(flog_list);
            fclose(flog_list);
        } else {
            ROS_ERROR("Can't open loglist file");
        }

        ROS_INFO("opening %s as log", buffer);

        log_file = fopen(buffer,"w");
        if (log_file != nullptr) {
            ROS_INFO("Log inited");
        } else {
            ROS_ERROR("Can't open log file");
        }
    }

    void on_imu_data(const sensor_msgs::Imu & _imu) {
        state.imu_data = _imu;
        Eigen::Vector3d acc(
            state.imu_data.linear_acceleration.x,
            state.imu_data.linear_acceleration.y,
            state.imu_data.linear_acceleration.z
        );

        //This is a FLU angular rate
        angular_rate.x() = _imu.angular_velocity.x;// * (1-ANGULARRATE_MIX) + ANGULARRATE_MIX*angular_rate.x();
        angular_rate.y() = _imu.angular_velocity.y;// * (1-ANGULARRATE_MIX) + ANGULARRATE_MIX*angular_rate.y();
        angular_rate.z() = _imu.angular_velocity.z;// * (1-ANGULARRATE_MIX) + ANGULARRATE_MIX*angular_rate.z();
        pos_ctrl->set_body_acc(acc);
#if FCHardware == DJI_SDK
        geometry_msgs::Quaternion quat = _imu.orientation;
        Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
        Eigen::Matrix3d RFLU2ENU = q.toRotationMatrix();
        q = Eigen::Quaterniond(RFLU2ENU*R_FLU2FRD.transpose());
        Eigen::Vector3d rpy = quat2eulers(q);
        //Original rpy is ENU, we need NED rpy
        fc_att_rpy = rpy;
        yaw_fc = constrainAngle(rpy.z());
#endif
    }

#if FCHardware == DJI_SDK
    void onFCAttitude(const geometry_msgs::QuaternionStamped & _quat) {
        // geometry_msgs::Quaternion quat = _quat.quaternion;
        // Eigen::Quaterniond q(quat.w, 
            // quat.x, quat.y, quat.z);
        // Eigen::Vector3d rpy = quat2eulers(q);

        //Original rpy is FLU, we need NED rpy
        // fc_att_rpy = rpy;
        // yaw_fc = constrainAngle(-rpy.z() + M_PI/2);

        // ROS_INFO("Yaw FC is %3.2f %3.2f", rpy.z(), yaw_fc);
    }
#else
    void onFCAttitude(const sensor_msgs::Imu & _imu) {
        geometry_msgs::Quaternion quat = _imu.orientation;
        auto q = ENU2NED(Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
        Eigen::Vector3d rpy = quat2eulers(q);
        fc_att_rpy = rpy;
        yaw_fc = constrainAngle(rpy.z());

        Eigen::Vector3d rpy_raw = quat2eulers(Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
        // printf("Atti sp yaw %3.2f/%3.2f(state) deg pitch %3.2f deg roll %3.2f deg yaw_offset %3.2f\n", 
        //         atti_out.yaw_sp*180/M_PI, state.yaw_sp*180/M_PI, atti_out.pitch_sp*180/M_PI, atti_out.roll_sp*180/M_PI, 
        //         yaw_offset*180/M_PI);
        // printf("NED Yaw %3.2f deg pitch %3.2f deg roll %3.2f deg\n", rpy.z()*180/M_PI, rpy.y()*180/M_PI, rpy.x()*180/M_PI);
        printf("FC Raw Yaw %3.2f deg pitch %3.2f deg roll %3.2f deg\n", rpy_raw.z()*180/M_PI, rpy_raw.y()*180/M_PI, rpy_raw.x()*180/M_PI);
    }
#endif

    void onFCAttitudeTarget(const mavros_msgs::AttitudeTarget atti_target) {
        auto quat = atti_target.orientation;
        auto q = ENU2NED(Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
        Eigen::Quaterniond q_raw(quat.w, quat.x, quat.y, quat.z);
        Eigen::Vector3d rpy = quat2eulers(q_raw);
        printf("onFCAttitudeTarget(ENU) yaw %3.2f deg pitch %3.2f deg roll %3.2f deg\n", rpy.z()*180/M_PI, 
            rpy.y()*180/M_PI, rpy.x()*180/M_PI);
    }

    void set_drone_global_pos_vel_att(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Quaterniond quat) {
        pos_ctrl->set_pos(pos);
        pos_ctrl->set_global_vel(vel);

        state.pose.position.x = pos.x();
        state.pose.position.y = pos.y();
        state.pose.position.z = pos.z();

        state.global_vel.x = vel.x();
        state.global_vel.y = vel.y();
        state.global_vel.z = vel.z();

        pos_ctrl->set_attitude(quat);
    }

    void on_commander_state(const swarmtal_msgs::drone_commander_state & _cmd_state) {
        commander_state = _cmd_state;
    }

    void OnSwarmPosCommand(const swarmtal_msgs::drone_pos_ctrl_cmd & _cmd) {
        vel_ff = Eigen::Vector3d(0, 0, 0);
        acc_ff = Eigen::Vector3d(0, 0, 0);
        switch (_cmd.ctrl_mode) {
            case drone_pos_ctrl_cmd::CTRL_CMD_POS_MODE: {
                pos_sp.x() = _cmd.pos_sp.x;
                pos_sp.y() = _cmd.pos_sp.y;
                pos_sp.z() = _cmd.pos_sp.z;
                vel_ff.x() = _cmd.vel_sp.x;
                vel_ff.y() = _cmd.vel_sp.y;
                vel_ff.z() = _cmd.vel_sp.z;
                acc_ff.x() = _cmd.acc_sp.x;
                acc_ff.y() = _cmd.acc_sp.y;
                acc_ff.z() = _cmd.acc_sp.z;
                break;
            } 
            case drone_pos_ctrl_cmd::CTRL_CMD_VEL_MODE: {
                vel_sp.x() = _cmd.vel_sp.x;
                vel_sp.y() = _cmd.vel_sp.y;
                vel_sp.z() = _cmd.vel_sp.z;
                acc_ff.x() = _cmd.acc_sp.x;
                acc_ff.y() = _cmd.acc_sp.y;
                acc_ff.z() = _cmd.acc_sp.z;
                break;
            }
            case drone_pos_ctrl_cmd::CTRL_CMD_ATT_THRUST_MODE:
            case drone_pos_ctrl_cmd::CTRL_CMD_ATT_VELZ_MODE: {
                att_sp.w() = _cmd.att_sp.w;
                att_sp.x() = _cmd.att_sp.x;
                att_sp.y() = _cmd.att_sp.y;
                att_sp.z() = _cmd.att_sp.z;
                z_sp = _cmd.z_sp;
                break;
            }
            case drone_pos_ctrl_cmd::CTRL_CMD_IDLE_MODE: 
            default: {
                return;
                break;
            }
        }

        //TODO:Write attitude

        state.ctrl_mode = _cmd.ctrl_mode;

        state.yaw_sp =  constrainAngle(_cmd.yaw_sp);

        state.max_vel = _cmd.max_vel;

        state.use_fc_yaw = _cmd.use_fc_yaw;

        last_cmd_ts = ros::Time::now();
    }

    Eigen::Quaterniond yaw_offset_mocap_conversion() {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw_offset, Vector3d::UnitZ()));
    }


    void OnVisualOdometry(const nav_msgs::Odometry & odom) {
        static Eigen::Vector3d ang_vel_last = Eigen::Vector3d(0, 0, 0);
        auto pose = odom.pose.pose;
        auto velocity = odom.twist.twist.linear;
        // auto angvel = odom.twist.twist.angular;

        Eigen::Vector3d pos(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );

        Eigen::Vector3d vel(
            velocity.x,
            velocity.y,
            velocity.z
        );

        auto ang_vel = angular_rate;
        Eigen::Matrix3d omgx;
        omgx << 0, -ang_vel.z(), ang_vel.y(),
                ang_vel.z(), 0, -ang_vel.x(),
                -ang_vel.y(), ang_vel.x(), 0;


        Eigen::Quaterniond quat(pose.orientation.w, 
            pose.orientation.x, pose.orientation.y, pose.orientation.z);
        

        pos = pos - quat*odom_gc_pos;
        vel = vel - omgx*quat.toRotationMatrix()*odom_gc_pos;

        set_drone_global_pos_vel_att(pos, vel, quat);

        Eigen::Quaterniond q_in_ned = FLU2NED(quat);
        odom_att_rpy = quat2eulers(q_in_ned); //in NED
        yaw_odom = odom_att_rpy.z(); //Odom is also FLU, but we want NED RPY for FC Fix and output
        yaw_offset = constrainAngle(yaw_fc - yaw_odom);
        // printf("Yaw odom %3.2f yaw_fc %3.2f offset %3.2f\n", yaw_odom*57.3, yaw_fc*57.3, yaw_offset*57.3);
    }


    void set_drone_attitude_target(AttiCtrlOut atti_out) {
#if FCHardware == DJI_SDK
        //Use dji ros to set drone attitude target
        sensor_msgs::Joy dji_command_so3; //! @note for dji ros wrapper
        dji_command_so3.header.stamp    = ros::Time::now();
        dji_command_so3.header.frame_id = std::string("FLU");
        uint8_t flag;
        if (atti_out.thrust_mode == AttiCtrlOut::THRUST_MODE_THRUST) {
            flag = VERTICAL_THRUST | HORIZONTAL_ANGLE | YAW_ANGLE | HORIZONTAL_BODY | STABLE_DISABLE;
        } else {
            flag = VERTICAL_VELOCITY  | HORIZONTAL_ANGLE | YAW_ANGLE | HORIZONTAL_BODY | STABLE_DISABLE;
        }

        double yaw_sp =  atti_out.yaw_sp;
        double roll_sp = atti_out.roll_sp;
        double pitch_sp = atti_out.pitch_sp;
        
        if (!state.use_fc_yaw) {
            yaw_sp = constrainAngle(yaw_sp + yaw_offset);
        }
        
        dji_command_so3.axes.push_back(roll_sp);       // x
        dji_command_so3.axes.push_back(pitch_sp);       // y
        if (atti_out.thrust_mode == AttiCtrlOut::THRUST_MODE_THRUST) {
            atti_out.thrust_sp = float_constrain(atti_out.thrust_sp, 0.02, thrust_limit);
            dji_command_so3.axes.push_back(atti_out.thrust_sp*100); // z
        } else {
            dji_command_so3.axes.push_back(atti_out.thrust_sp); // z
        }
        dji_command_so3.axes.push_back(yaw_sp);       // w
        dji_command_so3.axes.push_back(flag);

        control_pub.publish(dji_command_so3);
#else
        if (atti_out.thrust_mode == AttiCtrlOut::THRUST_MODE_THRUST) {
            mavros_msgs::AttitudeTarget att_target;
            att_target.header.stamp = ros::Time::now();
            att_target.header.frame_id = "world";
            att_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
            auto atti_sp_out = atti_out.atti_sp;
            if (!state.use_fc_yaw) {
                //Rotated 
                atti_sp_out = Eigen::AngleAxisd(yaw_offset, Vector3d::UnitZ())*atti_sp_out;
            }
            atti_sp_out = NED2ENU(atti_sp_out);
            att_target.orientation.w = atti_sp_out.w();
            att_target.orientation.x = atti_sp_out.x();
            att_target.orientation.y = atti_sp_out.y();
            att_target.orientation.z = atti_sp_out.z();
            att_target.body_rate.z = 0; //yaw rate tmp to be zero
            att_target.thrust = atti_out.thrust_sp;
            control_pub.publish(att_target);
            auto rpy = quat2eulers(atti_sp_out);
            printf("Real SP yaw %3.2f pitch %3.2f roll %3.2f\n", rpy.z()*57.3, rpy.y()*57.3, rpy.x()*57.3);
        } else if (atti_out.thrust_mode == AttiCtrlOut::THRUST_MODE_VELZ) {
            //Dummy input only
            mavros_msgs::PositionTarget pos_target;
            pos_target.header.stamp = ros::Time::now();
            pos_target.header.frame_id = "world";
            pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; //Note we send FLU
            Vector3d acc_sp(0., 0., 9.8);
            pos_target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ;
            pos_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY;
            auto atti_sp = atti_out.atti_sp;
            atti_sp = NED2ENU(att_sp);
            //to euler
            Vector3d rpy = quat2eulers(atti_sp);
            pos_target.yaw = rpy(2);
            pos_target.yaw_rate = 0;
            pos_target.velocity.x = 0.0;
            pos_target.velocity.y = 0.0;
            pos_target.velocity.z = atti_out.thrust_sp;
            pos_target.acceleration_or_force.x = 0.0;
            pos_target.acceleration_or_force.y = 0.0;
            pos_target.acceleration_or_force.z = 0.0;
            control_pos_vel_px4_pub.publish(pos_target);
        }
#endif
    }

    void send_dummy_atti_cmd() {
        //sending dummy cmd to help entering offboard mode
        printf("is sending dummy cmd yaw %3.2f\n", yaw_odom*57.3);
        if (commander_state.flight_status == swarmtal_msgs::drone_commander_state::FLIGHT_STATUS_IN_AIR) {
            atti_out.thrust_mode = AttiCtrlOut::THRUST_MODE_THRUST;
            atti_out.thrust_sp = pos_ctrl->thrust_ctrl.get_level_thrust();
            atti_out.atti_sp = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_odom, Vector3d::UnitZ()));
            state.yaw_sp = yaw_odom;
            atti_out.yaw_sp = yaw_odom;
            set_drone_attitude_target(atti_out);
        } else {
            atti_out.thrust_mode = AttiCtrlOut::THRUST_MODE_THRUST;
            atti_out.thrust_sp = 0.0;
            atti_out.atti_sp = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_odom, Vector3d::UnitZ()));
            state.yaw_sp = yaw_odom;
            atti_out.yaw_sp = yaw_odom;
            set_drone_attitude_target(atti_out);
        }
    }

    void control_update(const ros::TimerEvent & e) {
        state.count ++;
        float dt = (e.current_real - e.last_real).toSec();
        
        if ((ros::Time::now() - last_cmd_ts).toSec() > MAX_CMD_LOST_TIME) {
            state.ctrl_mode = drone_pos_ctrl_cmd::CTRL_CMD_IDLE_MODE;
        }
        if (state.ctrl_mode == drone_pos_ctrl_cmd::CTRL_CMD_IDLE_MODE) {
            //IDLE
            ROS_INFO_THROTTLE(1.0, "Position controller idle mode");
            vel_ff = Eigen::Vector3d(0, 0, 0);
            pos_ctrl->reset();
            send_dummy_atti_cmd();
            return;
        } 
        atti_out.thrust_mode = AttiCtrlOut::THRUST_MODE_THRUST;
        
        if (state.ctrl_mode < drone_pos_ctrl_cmd::CTRL_CMD_ATT_THRUST_MODE) {
            if (state.ctrl_mode == drone_pos_ctrl_cmd::CTRL_CMD_POS_MODE) {
                vel_sp = pos_ctrl->control_pos(pos_sp, dt) + vel_ff;
                vel_sp.x() = float_constrain(vel_sp.x(), -state.max_vel.x, state.max_vel.x);
                vel_sp.y() = float_constrain(vel_sp.y(), -state.max_vel.y, state.max_vel.y);
                vel_sp.z() = float_constrain(vel_sp.z(), -state.max_vel.z, state.max_vel.z);
            } else {
                pos_ctrl->position_controller_reset();
            }
            
            acc_sp = pos_ctrl->control_vel(vel_sp, dt);

            acc_sp.z() =  pos_ctrl->control_vel_z(vel_sp.z(), dt);

            acc_sp = acc_sp + acc_ff;
            //Send acc sp to network or
            YawCMD yaw_cmd;
            yaw_cmd.yaw_mode = YAW_MODE_LOCK;
            yaw_cmd.yaw_sp = state.yaw_sp;

            acc_sp.x() = float_constrain(acc_sp.x(), -MAX_ACC, MAX_ACC);
            acc_sp.y() = float_constrain(acc_sp.y(), -MAX_ACC, MAX_ACC);
            acc_sp.z() = float_constrain(acc_sp.z(), -MAX_ACC, MAX_ACC);


            atti_out =  pos_ctrl->control_acc(acc_sp, yaw_cmd, dt);
#ifdef USE_DJI_THRUST_CTRL
            // ROS_INFO("Using direct velocity mode");
            atti_out.thrust_sp = vel_sp.z();
            atti_out.thrust_mode = AttiCtrlOut::THRUST_MODE_VELZ;
#else
            atti_out.thrust_sp = acc_sp.z() + pos_ctrl->thrust_ctrl.get_level_thrust();
            atti_out.thrust_mode = AttiCtrlOut::THRUST_MODE_THRUST;
            filtered_thrust = lowpass_filter(atti_out.thrust_sp, 1, filtered_thrust, dt);
#endif
            if (state.count % 5 == 0)
            {
                ROS_INFO("Mode %d Possp/pos %3.2f %3.2f %3.2f/ %3.2f %3.2f %3.2f",
                    state.ctrl_mode,
                    pos_sp.x(), pos_sp.y(), pos_sp.z(),
                    pos_ctrl->pos.x(),pos_ctrl->pos.y(),pos_ctrl->pos.z()
                );
                ROS_INFO("Velsp/vel %3.2f %3.2f %3.2f / %3.2f %3.2f %3.2f", 
                    vel_sp.x(), vel_sp.y(), vel_sp.z(),
                    pos_ctrl->vel.x(),pos_ctrl->vel.y(),pos_ctrl->vel.z()
                );
                ROS_INFO("smooth_thr %f accsp %3.2f %3.2f %3.2f, abx %3.2f/%3.2f",
                    filtered_thrust,
                    acc_sp.x(),
                    acc_sp.y(),
                    acc_sp.z(),
                    atti_out.abx_sp,
                    pos_ctrl->thrust_ctrl.acc
                );
                ROS_INFO("R %4.3f P %4.3f Y %4.3f thr : %3.2f", 
                    atti_out.roll_sp * 57.3,
                    atti_out.pitch_sp * 57.3,
                    atti_out.yaw_sp * 57.3,
                    atti_out.thrust_sp
                );
            }
        } else {
            atti_out.atti_sp = att_sp;

            Eigen::Vector3d rpy = quat2eulers(atti_out.atti_sp);
            atti_out.roll_sp = rpy.x();
            atti_out.pitch_sp = rpy.y();
            atti_out.yaw_sp = rpy.z();
            atti_out.thrust_sp = z_sp;

            if (state.ctrl_mode == drone_pos_ctrl_cmd::CTRL_CMD_ATT_VELZ_MODE) {
                atti_out.thrust_mode = AttiCtrlOut::THRUST_MODE_VELZ;
            }
        }
            
        printf("is sending cmd yaw %3.2f\n", atti_out.yaw_sp*57.3);
        set_drone_attitude_target(atti_out);
        
        state.pos_sp.x = pos_sp.x();
        state.pos_sp.y = pos_sp.y();
        state.pos_sp.z = pos_sp.z();
        state.acc_cmd.x = acc_sp.x();
        state.acc_cmd.y = acc_sp.y();
        state.acc_cmd.z = acc_sp.z();

        state.vel_cmd.x = vel_sp.x();
        state.vel_cmd.y = vel_sp.y();
        state.vel_cmd.z = vel_sp.z();

        state.thrust_cmd = atti_out.thrust_sp;

        state_pub.publish(state);

        recordCSV();
    }

};


int main(int argc, char** argv)
{

    ROS_INFO("drone_POS_CONTROL_INIT\nIniting\n");

    ros::init(argc, argv, "drone_position_control");

    ros::NodeHandle nh("drone_position_control");

    DronePosControl pos_control(nh);

    ROS_INFO("Pos control node ready");
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();
}
