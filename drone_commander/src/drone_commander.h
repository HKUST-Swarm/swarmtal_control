#include <ros/ros.h>
#include <swarmtal_msgs/drone_pos_ctrl_cmd.h>
#include <swarmtal_msgs/drone_onboard_command.h>
#include <swarmtal_msgs/drone_commander_state.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>


#include <mavros_msgs/RCIn.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace swarmtal_msgs;
using namespace Eigen;


using DCMD=drone_commander_state;
using OCMD=drone_onboard_command;
using DPCL=drone_pos_ctrl_cmd;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

inline Eigen::Vector3d quat2eulers(Eigen::Quaterniond quat);
class DroneCommander {
public:
    struct DroneCommanderParam {
        bool use_px4_pos_ctrl = true;
        double max_vo_latency = 0.2;
        double battery_remain_cutoff = 240.0;
        double battery_remain_param_a = 345.375;
        double battery_remain_param_b = -4757.3;
        double landing_thrust = 0.035;
        bool is_px4 = false;
    };
    DroneCommander(ros::NodeHandle & _nh);
protected:
    ros::NodeHandle & nh;
    drone_commander_state state;
    DroneCommanderParam param;

    ros::Subscriber vo_sub;
    ros::Subscriber onboard_cmd_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber flight_status_sub;
    ros::Subscriber ctrl_dev_sub;
    ros::Subscriber fc_att_sub;
    ros::Subscriber bat_sub;
    ros::Subscriber imu_data_sub, vo_sub_slow, imu_fused_data_sub;
    ros::Subscriber fc_state_sub, fc_extened_state_sub;

    ros::Timer loop_timer;

    ros::Time last_rc_ts;
    ros::Time last_onboard_cmd_ts;
    ros::Time last_vo_ts;
    ros::Time last_flight_status_ts;
    ros::Time last_try_arm_time;
    ros::Time last_vo_image_ts;
    ros::Time last_send_odom_to_fc;

    int fail_arm_times = 0;

    nav_msgs::Odometry odometry;
    sensor_msgs::Joy rc;

    ros::Time boot_time;

    ros::Publisher commander_state_pub;
    ros::Publisher ctrl_cmd_pub, control_pos_vel_px4_pub, control_att_pub, mavros_system_status_pub, mavros_odom_pub;

    drone_pos_ctrl_cmd * ctrl_cmd = nullptr;

    ros::ServiceClient control_auth_client;
    ros::ServiceClient drone_task_control, drone_landing_control;

    Eigen::Vector3d hover_pos = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d takeoff_origin = Eigen::Vector3d(0, 0, 0);

    bool takeoff_inited = false;
    bool landing_inited = false;

    int control_count = 0;

    int last_hover_count = -1;


    double yaw_fc = 0;
    double yaw_vo = 0;


    bool yaw_sp_inited = false;

    bool rc_fail_detection = true;


    bool in_fc_landing = false;

    bool is_landing_tail = false;
    bool is_touch_ground = false;

    bool pos_sp_inited = false;

    mavros_msgs::State px4_fcu_state;
    Eigen::Matrix3d R_ENU2NED;
    Eigen::Matrix3d R_FLU2FRD; 

    void init_states();
    void init_subscribes();

    void vo_callback_image(const nav_msgs::Odometry & _odom);
    void vo_callback(const nav_msgs::Odometry & _odom);
    void rc_callback(const sensor_msgs::Joy & _rc);
    void rc_mavros_callback(const mavros_msgs::RCIn & _rc);
    void flight_status_callback(const std_msgs::UInt8 & _flight_status);
    void onboard_cmd_callback(const drone_onboard_command & _cmd);
    void fc_attitude_callback(const geometry_msgs::QuaternionStamped & _quat);
    void loop(const ros::TimerEvent & _e);
    void battery_callback(const sensor_msgs::BatteryState & _bat);
    void on_imu_data(const sensor_msgs::Imu & _imu);
    void on_imu_data_fused(const sensor_msgs::Imu & _imu);

    bool is_odom_valid(const nav_msgs::Odometry & _odom);
    bool is_rc_valid(const sensor_msgs::Joy & _rc);

    bool check_control_auth();

    void try_arm(bool arm);

    void try_control_auth(bool auth);

    void process_control();

    void process_input_source();

    bool rc_request_onboard();
    bool rc_request_vo();
    bool rc_moving_stick();

    void process_control_mode();

    void prepare_control_hover();

    bool set_hover_target_position(double x, double y, double z);

    void process_control_idle();
    void process_control_takeoff();
    void process_control_landing();
    void process_control_posvel();
    void process_control_att();
    void process_control_mission() {};

    void process_rc_input();
    void process_none_input();
    void process_onboard_input();

    void reset_ctrl_cmd();
    void reset_ctrl_cmd_max_vel();
    void reset_yaw_sp();

    void request_ctrl_mode(uint32_t req_ctrl_mode);
    
    void send_ctrl_cmd();
    void send_control_cmd_px4();

    void set_att_setpoint(double roll, double pitch, double yawrate, double z, bool z_use_vel=true, bool yaw_use_rate=true, bool use_fc_yaw = false);
    void set_pos_setpoint(double x, double y, double z, double yaw=NAN, double vx_ff=0, double vy_ff=0, double vz_ff=0, double ax_ff=0, double ay_ff=0, double az_ff=0);
    void set_vel_setpoint(double vx, double vy, double vz, double yaw=NAN, double ax_ff=0, double ay_ff=0, double az_ff=0);

    bool request_drone_landing();
    void fc_state_callback(const mavros_msgs::State & _state);
    bool callArmService(bool arm);
    void fc_extended_state_callback(const mavros_msgs::ExtendedState & _state);

    bool nead_control_by_this();
    void setupFCControl();

    void sendPX4SystemActive();
    void sendPX4SystemInactive();

    Eigen::Quaterniond FLU2NED(const Eigen::Quaterniond & q) {
        Matrix3d R = R_FLU2FRD * q.toRotationMatrix() * R_FLU2FRD;
        return Eigen::Quaterniond(R);
    }

    Eigen::Quaterniond ENU2NED(const Eigen::Quaterniond & q) {
        Matrix3d R = R_ENU2NED*q.toRotationMatrix()*R_FLU2FRD;
        return Eigen::Quaterniond(R);
    }

};

