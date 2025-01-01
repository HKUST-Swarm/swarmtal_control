/*************************************************************************************
 * MIT License
 * 
 * Copyright (c) 2025 xuhao3e8
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this 
 * software and associated documentation files (the "Software"), to deal in the Software 
 * without restriction, including without limitation the rights to use, copy, modify, 
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to 
 * permit persons to whom the Software is furnished to do so, subject to the following 
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all 
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: xuhao3e8 <xuhao3e8@gmail.com>
 *************************************************************************************/

#ifndef DRONE_COMMANDER_HPP
#define DRONE_COMMANDER_HPP

#include <memory>
#include <cmath>
#include <cstdio>
#include <string>

// ROS2 Core
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

// Mavros Messages
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/companion_process_status.hpp"
#include "mavros_msgs/msg/rc_in.hpp"

// Swarmtal Messages
#include "swarmtal_msgs/msg/drone_pos_ctrl_cmd.hpp"
#include "swarmtal_msgs/msg/drone_onboard_command.hpp"
#include "swarmtal_msgs/msg/drone_commander_state.hpp"

// Standard ROS2 messages
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"

// Eigen
#include <eigen3/Eigen/Dense>

// Short alias for swarmtal_msgs
namespace swarmtal_msgs_ros2 = swarmtal_msgs::msg;
using DCMD = swarmtal_msgs_ros2::DroneCommanderState;
using OCMD = swarmtal_msgs_ros2::DroneOnboardCommand;
using DPCL = swarmtal_msgs_ros2::DronePosCtrlCmd;

/**
 * @brief MAV state enumerations (example in C++11 enum class style).
 */
enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION
};

/**
 * @brief Convert a quaternion to Euler angles (roll, pitch, yaw).
 * @param quat Input quaternion.
 * @return A Vector3d containing roll, pitch, and yaw (in radians).
 */
Eigen::Vector3d quat2eulers(const Eigen::Quaterniond & quat);

/**
 * @brief A ROS2-based DroneCommander class: manages drone state, control, and callbacks.
 */
class DroneCommander : public rclcpp::Node
{
public:
  /**
   * @brief Struct to store parameters for DroneCommander.
   */
  struct DroneCommanderParam
  {
    double max_loss_rc             {1.0};
    double max_loss_sdk            {1.0};
    double max_odom_velocity       {25.0};
    double rc_deadzone_rpy         {0.1};
    double rc_deadzone_thrust      {0.2};
    double pwm_center              {1500.0};
    double pwm_100                 {500.0};
    double pwm_deadzone_rpy        {50.0};
    double pwm_deadzone_thr        {100.0};
    double rc_max_tilt_vel         {3.0};
    double rc_max_z_vel            {2.0};
    double default_max_tilt_vel    {5.0};
    double default_max_z_vel       {3.0};
    double rc_max_yaw_rate         {1.57};
    double rc_max_tilt_angle       {0.52};
    double takeoff_vel_z           {1.0};
    double landing_vel_z           {-0.3};
    double landing_vel_z_emergency {-2.0};
    double max_auto_z_error        {0.05};
    double max_auto_tilt_error     {0.05};
    double min_takeoff_height      {0.5};
    double min_try_arm_duration    {1.0};
    int    max_try_arm_times       {5};
    double max_loss_onboard_cmd    {60.0};
    double landing_att_mode_height {0.1};
    double landing_att_min_height  {0.1};
    double loop_duration           {0.02};
    double danger_speed_hover      {4.5};  // 通常用 rc_max_tilt_vel + 1.5 计算，可在程序中根据需求更新
    double landing_vel_z_battery_low {-0.5};

    // Additional original parameters
    bool   use_px4_pos_ctrl         {true};
    double max_vo_latency           {0.4};
    double battery_remain_cutoff    {240.0};
    double battery_remain_param_a   {345.375};
    double battery_remain_param_b   {-4757.3};
    double landing_thrust           {0.2};
    bool   is_px4                   {true};
  };


  DroneCommander();

private:
  // ------------------- Internal states -------------------
  DCMD state_;
  DroneCommanderParam param_;
  DPCL * ctrl_cmd_ = nullptr;  // pointer to state_.ctrl_cmd

  // Time trackers
  rclcpp::Time boot_time_;
  rclcpp::Time last_rc_ts_;
  rclcpp::Time last_onboard_cmd_ts_;
  rclcpp::Time last_vo_ts_;
  rclcpp::Time last_flight_status_ts_;
  rclcpp::Time last_try_arm_time_;
  rclcpp::Time last_vo_image_ts_;
  rclcpp::Time last_send_odom_to_fc_;

  // Some counters / booleans
  int fail_arm_times_ = 0;
  bool yaw_sp_inited_ = false;
  bool rc_fail_detection_ = true;
  bool in_fc_landing_ = false;
  bool is_landing_tail_ = false;
  bool is_touch_ground_ = false;
  bool pos_sp_inited_ = false;
  bool takeoff_inited_ = false;
  bool landing_inited_ = false;
  int control_count_ = 0;
  int last_hover_count_ = -1;

  // Additional states
  nav_msgs::msg::Odometry odometry_;
  sensor_msgs::msg::Joy rc_;
  double yaw_fc_ = 0.0;
  double yaw_vo_ = 0.0;

  // Transform matrices
  Eigen::Matrix3d R_ENU2NED_;
  Eigen::Matrix3d R_FLU2FRD_;

  // Some coordinate variables
  Eigen::Vector3d hover_pos_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d takeoff_origin_ = Eigen::Vector3d(0, 0, 0);

  // ------------------- ROS2 Interfaces -------------------
  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_slow_;
  rclcpp::Subscription<OCMD>::SharedPtr onboard_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rc_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_mavros_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flight_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr fc_att_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_fused_data_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr fc_state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr fc_extended_state_sub_;

  // Publishers
  rclcpp::Publisher<DCMD>::SharedPtr commander_state_pub_;
  rclcpp::Publisher<DPCL>::SharedPtr ctrl_cmd_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr control_pos_vel_px4_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr control_att_pub_;
  rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_pub_;

  // Service clients
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr control_auth_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr drone_landing_control_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // ------------------- Internal methods -------------------
  void initStates();
  void declareAllParameters();
  void getAllParameters();
  void initROS2Interfaces();
  void loopTimerCallback();

  // Utility
  bool callArmService(bool arm);
  void tryArm(bool arm);
  void tryControlAuth(bool auth);
  bool needControlByThis();
  bool checkControlAuth();
  bool isOdomValid(const nav_msgs::msg::Odometry & odom);
  void resetYawSp();
  void resetCtrlCmd();
  void resetCtrlCmdMaxVel();
  double lowpassFilter(double input, double fc, double output_last, double dt);

  // Coordinates
  Eigen::Quaterniond FLU2NED(const Eigen::Quaterniond & q);
  Eigen::Quaterniond ENU2NED(const Eigen::Quaterniond & q);

  // ROS2 Callbacks
  void voCallbackImage(const nav_msgs::msg::Odometry & odom);
  void voCallback(const nav_msgs::msg::Odometry & odom);
  void rcCallback(const sensor_msgs::msg::Joy & joy);
  void rcMavrosCallback(const mavros_msgs::msg::RCIn & rc_in);
  void batteryCallback(const sensor_msgs::msg::BatteryState & bat);
  void fcStateCallback(const mavros_msgs::msg::State & st);
  void fcExtendedStateCallback(const mavros_msgs::msg::ExtendedState & est);
  void onboardCmdCallback(const OCMD & cmd);
  void onImuData(const sensor_msgs::msg::Imu & imu);
  void onImuDataFused(const sensor_msgs::msg::Imu & imu);

  // Control logic
  void processInputSource();
  void processRcInput();
  void processOnboardInput();
  void processNoneInput();
  void processControl();
  void processControlIdle();
  void processControlPosvel();
  void processControlAtt();
  void processControlTakeoff();
  void processControlLanding();
  void processControlMission();
  void requestCtrlMode(uint32_t req_mode);
  void processControlMode();
  void sendControlCmdPX4();
  void sendCtrlCmd();
  bool setHoverTargetPosition(double x, double y, double z);
  void prepareControlHover();

  // Setpoint methods
  void setAttSetpoint(double roll, double pitch, double yawrate, double z,
                      bool z_use_vel=true, bool yaw_use_rate=true, bool use_fc_yaw=false);
  void setPosSetpoint(double x, double y, double z,
                      double yaw = std::numeric_limits<double>::quiet_NaN(),
                      double vx_ff = 0, double vy_ff = 0, double vz_ff = 0,
                      double ax_ff = 0, double ay_ff = 0, double az_ff = 0);
  void setVelSetpoint(double vx, double vy, double vz,
                      double yaw = std::numeric_limits<double>::quiet_NaN(),
                      double ax_ff = 0, double ay_ff = 0, double az_ff = 0);

  // Onboard command
  void handleOnboardCommand(const OCMD & cmd);
  bool rcRequestOnboard();
  bool rcRequestVo();
  bool rcMovingStick();
};

#endif // DRONE_COMMANDER_HPP
