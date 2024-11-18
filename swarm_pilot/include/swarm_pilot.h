#pragma once
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <swarmcomm_msgs/incoming_broadcast_data.h>
#include <swarmcomm_msgs/data_buffer.h>
#include <swarmtal_msgs/drone_onboard_command.h>
#include <swarmtal_msgs/drone_pos_ctrl_cmd.h>
#include <swarmtal_msgs/drone_commander_state.h>
#include <mavlink/swarm/mavlink.h>
#include <swarmcomm_msgs/remote_uwb_info.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <swarm_msgs/swarm_drone_basecoor.h>
#include <swarm_msgs/swarm_fused.h>
#include <swarm_msgs/Pose.h>
#include <bspline/Bspline.h>
#include <swarmcomm_msgs/swarm_network_status.h>
#include <nav_msgs/Odometry.h>

using namespace swarmcomm_msgs;
using namespace swarmtal_msgs;

    
class SwarmPilot;
class SwarmFormationControl {
    int self_id;
    int formation_mode = drone_onboard_command::CTRL_FORMATION_IDLE;
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

    void on_swarm_localization(const swarm_msgs::swarm_fused & swarm_fused);
    void on_swarm_basecoor(const swarm_msgs::swarm_drone_basecoor & swarm_fused);

    void on_swarm_traj(const bspline::Bspline & bspl);

    void on_position_command(drone_onboard_command cmd, int _id);
    void on_drone_position_command(drone_pos_ctrl_cmd pos_cmd);
    void set_swarm_formation_mode(uint8_t _formation_mode, int master_id, int sub_mode, Eigen::Vector3d dpos = Eigen::Vector3d::Zero(), double dyaw = 0);
    void end_formation();
};

struct NetworkStatus {
    ros::Time last_heartbeat = ros::Time(0);
    int drone_id;
    bool active = 0;
    double quality = -1;//0-100
    double bandwidth = -1; //0-100
    int hops = -1; //Hops to the target.
};

class SwarmPilot {
    ros::NodeHandle nh;

    ros::Subscriber incoming_data_sub, drone_cmd_state_sub, uwb_remote_sub, uwb_timeref_sub, local_cmd_sub;
    ros::Subscriber swarm_local_sub;
    ros::Subscriber swarm_basecoor_sub;
    ros::Subscriber swarm_traj_sub;
    ros::Subscriber drone_network_sub;
    ros::Subscriber odom_sub;
    ros::Publisher swarm_traj_pub;
    ros::Publisher onboardcmd_pub;
    ros::Publisher uwb_send_pub;
    ros::Publisher planning_tgt_pub;
    ros::Publisher traj_pub, exprolaration_pub;
    ros::Publisher swarm_network_status_pub;

    ros::Publisher position_cmd_pub;

    ros::Timer eight_trajectory_timer, net_timer;
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
    double send_drone_status_freq = 1.0;
    double heartbeat_timeout = 0.5;
    double send_odom_freq = 50.0;

    ros::Time last_send_odom;

    drone_commander_state cmd_state;
    ros::Time last_send_drone_status;

    int self_id = -1;
    uint8_t buf[1000] = {0};
    sensor_msgs::TimeReference uwb_time_ref;

    std::map<int, NetworkStatus> swarm_network_status;

    SwarmFormationControl * formation_control = nullptr;
    void on_uwb_timeref(const sensor_msgs::TimeReference &ref);

    bool planning_debug_mode = false;
    bool enable_planner;


public:
    bool is_planning_control_available();

    ros::Time LPS2ROSTIME(const int32_t &lps_time);
    int32_t ROSTIME2LPS(ros::Time ros_time);

    void send_position_command(Eigen::Vector3d pos, double yaw, Eigen::Vector3d vel = Eigen::Vector3d::Zero(), bool enable_planning = false);
    void send_velocity_command(Eigen::Vector3d vel, double yaw);

    SwarmPilot(ros::NodeHandle & _nh);
    
    void on_uwb_remote_node(const remote_uwb_info & info);

    void send_planning_command(drone_onboard_command cmd);

    void send_start_exploration(const drone_onboard_command & cmd);

    void on_mavlink_msg_remote_cmd(ros::Time stamp, int node_id, const mavlink_swarm_remote_command_t & cmd);
    
    void on_mavlink_drone_status(ros::Time stamp, int node_id, const mavlink_drone_status_t & msg);

    void on_drone_commander_state(const drone_commander_state & _state);
    void on_odometry(const nav_msgs::Odometry & odom);

    void send_mavlink_message(mavlink_message_t & msg, int send_method = 2);

    void incoming_broadcast_data_callback(std::vector<uint8_t> data, int sender_drone_id, ros::Time stamp);

    void incoming_broadcast_data_sub(const incoming_broadcast_data & data);

    void send_swarm_traj(const bspline::Bspline & bspl);

    void eight_trajectory_timer_callback(const ros::TimerEvent &e);
    void timer_callback(const ros::TimerEvent &e);
    
    void drone_network_callback(const swarmcomm_msgs::drone_network_status & status);
    void network_monitior_timer_callback(const ros::TimerEvent &e);
    void mission_trajs_timer_callback(const ros::TimerEvent &e);

    void start_mission_trajs(const drone_onboard_command & cmd);
    void start_spec_trajs(const drone_onboard_command & cmd);
    void end_mission();

    void load_missions(ros::NodeHandle & _nh);


};

