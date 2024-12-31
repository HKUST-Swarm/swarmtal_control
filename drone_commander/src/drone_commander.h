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
#include <eigen3/Eigen/Dense>

// ROS2 Core
#include "rclcpp/rclcpp.hpp"

// ROS2 Messages
#include "swarmtal_msgs/msg/drone_pos_ctrl_cmd.hpp"
#include "swarmtal_msgs/msg/drone_onboard_command.hpp"
#include "swarmtal_msgs/msg/drone_commander_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"

// MAVROS in ROS2 (ensure mavros_msgs is available in ROS2)
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"

// Eigen
using namespace Eigen;

// Short aliases for message types from swarmtal_msgs
using DCMD = swarmtal_msgs::msg::DroneCommanderState;
using OCMD = swarmtal_msgs::msg::DroneOnboardCommand;
using DPCL = swarmtal_msgs::msg::DronePosCtrlCmd;

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
inline Eigen::Vector3d quat2eulers(const Eigen::Quaterniond & quat);

/**
 * @brief The DroneCommander class handles drone state management, control, and callbacks.
 */
class DroneCommander
{
public:
  /**
   * @brief Struct to store parameters for DroneCommander.
   */
  struct DroneCommanderParam {
    bool use_px4_pos_ctrl = true;
    double max_vo_latency = 0.2;
    double battery_remain_cutoff = 240.0;
    double battery_remain_param_a = 345.375;
    double battery_remain_param_b = -4757.3;
    double landing_thrust = 0.035;
    bool is_px4 = false;
  };

  /**
   * @brief Constructor: pass in a shared pointer to an rclcpp Node (or have DroneCommander inherit from Node).
   * @param node The rclcpp::Node pointer used to create publishers/subscriptions/timers/etc.
   */
  explicit DroneCommander(const rclcpp::Node::SharedPtr & node);

protected:
  // ---------------------------------------------------------
  // Internal references and states
  // ---------------------------------------------------------

  /// Node pointer for ROS2 operations
  rclcpp::Node::SharedPtr node_;

  /// Drone commander state
  DCMD state;

  /// Parameters for DroneCommander
  DroneCommanderParam param;

  // ---------------------------------------------------------
  // Subscriptions
  // ---------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_slow_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rc_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_mavros_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flight_status_sub_;
  rclcpp::Subscription<OCMD>::SharedPtr onboard_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr fc_att_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_fused_data_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr fc_state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr fc_extended_state_sub_;

  // ---------------------------------------------------------
  // Timer
  // ---------------------------------------------------------
  /// Timer for the main loop (replacing ros::Timer)
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // ---------------------------------------------------------
  // Time trackers
  // ---------------------------------------------------------
  rclcpp::Time last_rc_ts_;
  rclcpp::Time last_onboard_cmd_ts_;
  rclcpp::Time last_vo_ts_;
  rclcpp::Time last_flight_status_ts_;
  rclcpp::Time last_try_arm_time_;
  rclcpp::Time last_vo_image_ts_;
  rclcpp::Time last_send_odom_to_fc_;
  rclcpp::Time boot_time_;

  // ---------------------------------------------------------
  // Other states and counters
  // ---------------------------------------------------------
  int fail_arm_times_ = 0;
  nav_msgs::msg::Odometry odometry_;
  sensor_msgs::msg::Joy rc_;
  int control_count_ = 0;
  int last_hover_count_ = -1;
  double yaw_fc_ = 0.0;
  double yaw_vo_ = 0.0;
  bool yaw_sp_inited_ = false;
  bool rc_fail_detection_ = true;
  bool in_fc_landing_ = false;
  bool is_landing_tail_ = false;
  bool is_touch_ground_ = false;
  bool pos_sp_inited_ = false;
  bool takeoff_inited_ = false;
  bool landing_inited_ = false;

  // ---------------------------------------------------------
  // Publishers
  // ---------------------------------------------------------
  rclcpp::Publisher<DCMD>::SharedPtr commander_state_pub_;
  rclcpp::Publisher<DPCL>::SharedPtr ctrl_cmd_pub_;
  rclcpp::Publisher<DPCL>::SharedPtr control_pos_vel_px4_pub_;
  rclcpp::Publisher<DPCL>::SharedPtr control_att_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mavros_system_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_pub_;

  // ---------------------------------------------------------
  // Service clients (in ROS2: rclcpp::Client<srv_type>::SharedPtr)
  // ---------------------------------------------------------
  rclcpp::Client<std::shared_ptr<void>>::SharedPtr control_auth_client_;
  rclcpp::Client<std::shared_ptr<void>>::SharedPtr drone_task_control_;
  rclcpp::Client<std::shared_ptr<void>>::SharedPtr drone_landing_control_;

  // ---------------------------------------------------------
  // Internal control command pointer
  // ---------------------------------------------------------
  DPCL * ctrl_cmd_ = nullptr;

  // ---------------------------------------------------------
  // Coordinate transforms
  // ---------------------------------------------------------
  Eigen::Vector3d hover_pos_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d takeoff_origin_ = Eigen::Vector3d(0, 0, 0);

  Eigen::Matrix3d R_ENU2NED_;
  Eigen::Matrix3d R_FLU2FRD_;

  // ---------------------------------------------------------
  // Internal methods
  // ---------------------------------------------------------

  /**
   * @brief Initialize drone states or internal variables.
   */
  void initStates();

  /**
   * @brief Initialize subscriptions, publishers, and timers in ROS2.
   */
  void initROS2Interfaces();

  // ------------------- Callbacks -------------------
  void voCallback(const nav_msgs::msg::Odometry & odom);
  void voCallbackImage(const nav_msgs::msg::Odometry & odom);
  void rcCallback(const sensor_msgs::msg::Joy & rc);
  void rcMavrosCallback(const mavros_msgs::msg::RCIn & rc);
  void flightStatusCallback(const std_msgs::msg::UInt8 & flight_status);
  void onboardCmdCallback(const OCMD & cmd);
  void fcAttitudeCallback(const geometry_msgs::msg::QuaternionStamped & quat);
  void batteryCallback(const sensor_msgs::msg::BatteryState & bat);
  void onImuData(const sensor_msgs::msg::Imu & imu);
  void onImuDataFused(const sensor_msgs::msg::Imu & imu);
  void fcStateCallback(const mavros_msgs::msg::State & state_msg);
  void fcExtendedStateCallback(const mavros_msgs::msg::ExtendedState & ext_state_msg);

  /**
   * @brief Main loop callback (replacing ros::TimerEvent).
   * In ROS2, typically the timer callback has no special event argument.
   */
  void loopTimerCallback();

  // ---------------------------------------------------------
  // Helper methods
  // ---------------------------------------------------------
  bool isOdomValid(const nav_msgs::msg::Odometry & odom);
  bool isRcValid(const sensor_msgs::msg::Joy & rc);
  bool checkControlAuth();
  void tryArm(bool arm);
  void tryControlAuth(bool auth);
  void processControl();
  void processInputSource();
  bool rcRequestOnboard();
  bool rcRequestVo();
  bool rcMovingStick();
  void processControlMode();
  void prepareControlHover();
  bool setHoverTargetPosition(double x, double y, double z);
  void processControlIdle();
  void processControlTakeoff();
  void processControlLanding();
  void processControlPosvel();
  void processControlAtt();
  void processControlMission() {}
  void processRcInput();
  void processNoneInput();
  void processOnboardInput();
  void resetCtrlCmd();
  void resetCtrlCmdMaxVel();
  void resetYawSp();
  void requestCtrlMode(uint32_t req_ctrl_mode);
  void sendCtrlCmd();
  void sendControlCmdPx4();
  void setAttSetpoint(double roll, double pitch, double yawrate, double z,
                      bool z_use_vel = true,
                      bool yaw_use_rate = true,
                      bool use_fc_yaw = false);
  void setPosSetpoint(double x, double y, double z,
                      double yaw = std::numeric_limits<double>::quiet_NaN(),
                      double vx_ff = 0, double vy_ff = 0, double vz_ff = 0,
                      double ax_ff = 0, double ay_ff = 0, double az_ff = 0);
  void setVelSetpoint(double vx, double vy, double vz,
                      double yaw = std::numeric_limits<double>::quiet_NaN(),
                      double ax_ff = 0, double ay_ff = 0, double az_ff = 0);
  bool requestDroneLanding();
  bool callArmService(bool arm);
  bool needControlByThis();
  void setupFCControl();
  void sendPX4SystemActive();
  void sendPX4SystemInactive();

  /**
   * @brief Convert quaternion from FLU to NED coordinate system.
   */
  Eigen::Quaterniond FLU2NED(const Eigen::Quaterniond & q) {
      Eigen::Matrix3d R = R_FLU2FRD_ * q.toRotationMatrix() * R_FLU2FRD_;
      return Eigen::Quaterniond(R);
  }

  /**
   * @brief Convert quaternion from ENU to NED coordinate system.
   */
  Eigen::Quaterniond ENU2NED(const Eigen::Quaterniond & q) {
      Eigen::Matrix3d R = R_ENU2NED_ * q.toRotationMatrix() * R_FLU2FRD_;
      return Eigen::Quaterniond(R);
  }
};

#endif // DRONE_COMMANDER_HPP
