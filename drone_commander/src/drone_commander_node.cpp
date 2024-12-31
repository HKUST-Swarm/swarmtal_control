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

#include <memory>
#include <cmath>
#include <cstdio>
#include <string>

// ROS2 Core
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

// Mavros Messages (ensure mavros_msgs is properly available in your ROS2 environment)
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/companion_process_status.hpp"
#include "mavros_msgs/msg/rc_in.hpp"

// Swarmtal Messages (already migrated to ROS2)
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

// Some #defines and constants from original code
#define MAX_LOSS_RC 1.0f
#define MAX_LOSS_SDK 1.0f
#define MAX_ODOM_VELOCITY 25.0f
#define RC_DEADZONE_RPY 0.1
#define RC_DEADZONE_THRUST 0.2
#define PWM_CENTER 1500.0
#define PWM_100 500.0
#define PWM_DEADZONE_RPY 50.0
#define PWM_DEADZONE_THR 100.0
#define RC_MAX_TILT_VEL 3.0
#define RC_MAX_Z_VEL 2.0
#define DEFAULT_MAX_TITL_VEL 5.0
#define DEFAULT_MAX_Z_VEL 3.0
#define RC_MAX_YAW_RATE 1.57
#define RC_MAX_TILT_ANGLE 0.52
#define TAKEOFF_VEL_Z 1.0
#define LANDING_VEL_Z -0.3
#define LANDING_VEL_Z_EMERGENCY -2.0
#define MAX_AUTO_Z_ERROR 0.05
#define MAX_AUTO_TILT_ERROR 0.05
#define MIN_TAKEOFF_HEIGHT 0.5
#define MIN_TRY_ARM_DURATION 1.0
#define MAX_TRY_ARM_TIMES 5
#define MAX_LOSS_ONBOARD_CMD 60.0
#define LANDING_ATT_MODE_HEIGHT 0.1
#define LANDING_ATT_MIN_HEIGHT 0.1
#define LOOP_DURATION 0.02
#define MAGIC_YAW_NAN 666666
#define DANGER_SPEED_HOVER (RC_MAX_TILT_VEL+1.5)
#define LANDING_VEL_Z_BATTERY_LOW -0.5
#define EPS 0.01

using namespace Eigen;
using namespace std::chrono_literals;

// Short alias for swarmtal msgs
namespace swarmtal_msgs_ros2 = swarmtal_msgs::msg;
using DCMD = swarmtal_msgs_ros2::DroneCommanderState;
using OCMD = swarmtal_msgs_ros2::DroneOnboardCommand;
using DPCL = swarmtal_msgs_ros2::DronePosCtrlCmd;

inline double float_constrain(double v, double min, double max)
{
  if (v < min) return min;
  if (v > max) return max;
  return v;
}

double expo(const double &value, const double &e)
{
  double x = float_constrain(value, -1.0, 1.0);
  double ec = float_constrain(e, 0.0, 1.0);
  return (1 - ec) * x + ec * x * x * x;
}

double superexpo(const double &value, double e = 0.5, double g = 0.5)
{
  double x = float_constrain(value, -1.0, 1.0);
  double gc = float_constrain(g, 0.0, 0.99);
  return expo(x, e) * (1 - gc) / (1 - std::fabs(x) * gc);
}

inline double constrainAngle(double x) {
  x = std::fmod(x + M_PI, 2*M_PI);
  if (x < 0) {
    x += 2*M_PI;
  }
  return x - M_PI;
}

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
inline Eigen::Vector3d quat2eulers(const Eigen::Quaterniond & quat) {
  Eigen::Vector3d rpy;
  rpy.x() = std::atan2(2.0 * (quat.w() * quat.x() + quat.y() * quat.z()),
                       1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
  rpy.y() = std::asin(2.0 * (quat.w() * quat.y() - quat.z() * quat.x()));
  rpy.z() = std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
                       1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
  return rpy;
}

/**
 * @brief A ROS2-based DroneCommander class: manages drone state, control, and callbacks.
 */
class DroneCommander : public rclcpp::Node
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

  DroneCommander()
  : Node("drone_commander")
  {
    RCLCPP_INFO(this->get_logger(), "DroneCommander node is initializing...");

    // Initialize transform matrices
    R_ENU2NED_ << 0, 1, 0,
                  1, 0, 0,
                  0, 0, -1;
    R_FLU2FRD_ <<  1, 0, 0,
                  0,-1, 0,
                  0, 0,-1;

    param_.is_px4 = true;

    initStates();
    declareParameters();
    getParameters();
    initROS2Interfaces();

    boot_time_ = this->now();
    last_flight_status_ts_ = this->now();
    last_rc_ts_ = this->now();
    last_vo_ts_ = this->now();
    last_onboard_cmd_ts_ = this->now();
    last_try_arm_time_ = this->now();
    last_send_odom_to_fc_ = this->now();

    // Create main control loop timer (50 Hz from LOOP_DURATION=0.02s)
    loop_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(LOOP_DURATION),
      std::bind(&DroneCommander::loopTimerCallback, this)
    );

    resetCtrlCmdMaxVel();

    RCLCPP_INFO(this->get_logger(), "DroneCommander node has been initialized. Spinning...");
  }

private:
  // ------------------- Internal states and parameters -------------------
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
  // For arming, in the original code: /mavros/cmd/arming (CommandBool)
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // ------------------- Functions -------------------
  void initStates()
  {
    state_.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
    state_.flight_status = DCMD::FLIGHT_STATUS_IDLE;
    state_.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
    state_.djisdk_valid = false;
    state_.is_armed = false;
    state_.rc_valid = false;
    rc_.axes.resize(16);
    state_.onboard_cmd_valid = false;
    state_.vo_valid = false;
    state_.control_auth = DCMD::CTRL_AUTH_RC;

    // The ctrl_cmd pointer references state_.ctrl_cmd
    ctrl_cmd_ = &state_.ctrl_cmd;
  }

  void declareParameters()
  {
    this->declare_parameter<bool>("rc_fail_detection", true);
    this->declare_parameter<double>("landing_thrust", 0.2);
    this->declare_parameter<double>("MAX_VO_LATENCY", 0.4);
    this->declare_parameter<double>("BATTERY_REMAIN_PARAM_A", 345.375);
    this->declare_parameter<double>("BATTERY_REMAIN_PARAM_B", -4757.3);
    this->declare_parameter<double>("BATTERY_REMAIN_CUTOFF", 240.0);
  }

  void getParameters()
  {
    rc_fail_detection_ = this->get_parameter("rc_fail_detection").as_bool();
    param_.landing_thrust = this->get_parameter("landing_thrust").as_double();
    param_.max_vo_latency = this->get_parameter("MAX_VO_LATENCY").as_double();
    param_.battery_remain_param_a = this->get_parameter("BATTERY_REMAIN_PARAM_A").as_double();
    param_.battery_remain_param_b = this->get_parameter("BATTERY_REMAIN_PARAM_B").as_double();
    param_.battery_remain_cutoff  = this->get_parameter("BATTERY_REMAIN_CUTOFF").as_double();

    RCLCPP_INFO(this->get_logger(), 
      "rc_fail_detection=%d, landing_thrust=%.2f, MAX_VO_LATENCY=%.2f, battery_remain_cutoff=%.1f",
      rc_fail_detection_, param_.landing_thrust, param_.max_vo_latency, param_.battery_remain_cutoff
    );
  }

  /**
   * @brief Set up all subscriptions, publishers, clients, etc.
   */
  void initROS2Interfaces()
  {
    // Publishers
    commander_state_pub_ = this->create_publisher<DCMD>("swarm_commander_state", 1);
    ctrl_cmd_pub_ = this->create_publisher<DPCL>("/drone_position_control/drone_pos_cmd", 1);
    control_pos_vel_px4_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 1);
    control_att_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    mavros_system_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>("/mavros/companion_process/status", 1);
    mavros_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mavros/odometry/out", 10);

    // Subscriptions (use lambda or std::bind)
    // vo_sub
    vo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "visual_odometry", 
      1, 
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->voCallback(*msg);
      }
    );
    // vo_sub_slow
    vo_sub_slow_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "visual_odometry_image", 
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->voCallbackImage(*msg);
      }
    );
    // onboard_cmd_sub
    onboard_cmd_sub_ = this->create_subscription<OCMD>(
      "onboard_command",
      10,
      [this](const OCMD::SharedPtr cmd) {
        this->onboardCmdCallback(*cmd);
      }
    );
    // rc_sub (in the original code, it was from dji or something)
    rc_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "rc",
      1,
      [this](const sensor_msgs::msg::Joy::SharedPtr joy) {
        this->rcCallback(*joy);
      }
    );
    // rc_mavros_sub
    rc_mavros_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
      "rc_mavros_in",
      1,
      [this](const mavros_msgs::msg::RCIn::SharedPtr rc) {
        this->rcMavrosCallback(*rc);
      }
    );
    // Battery
    bat_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery",
      1,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr bat) {
        this->batteryCallback(*bat);
      }
    );
    // IMU
    imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "fc_imu",
      1,
      [this](const sensor_msgs::msg::Imu::SharedPtr imu) {
        this->onImuData(*imu);
      }
    );
    imu_fused_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "fc_imu_fused",
      1,
      [this](const sensor_msgs::msg::Imu::SharedPtr imu) {
        this->onImuDataFused(*imu);
      }
    );
    // FC state
    fc_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state",
      10,
      [this](const mavros_msgs::msg::State::SharedPtr st) {
        this->fcStateCallback(*st);
      }
    );
    fc_extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state",
      10,
      [this](const mavros_msgs::msg::ExtendedState::SharedPtr est) {
        this->fcExtendedStateCallback(*est);
      }
    );

    // Service clients
    control_auth_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    drone_landing_control_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    RCLCPP_INFO(this->get_logger(), "DroneCommander: waiting for PX4 services...");
    // Optionally wait for them (blocking) or do it asynchronously
    // e.g., control_auth_client_->wait_for_service();

    RCLCPP_INFO(this->get_logger(), "DroneCommander: services ready (if found).");
  }

  /**
   * @brief Main loop callback (replacing ros::TimerEvent).
   */
  void loopTimerCallback()
  {
    control_count_++;
    auto now_t = this->now();

    // Check flight_status_time
    if (state_.djisdk_valid && (now_t - last_flight_status_ts_).seconds() > MAX_LOSS_SDK) {
      RCLCPP_INFO(this->get_logger(), "Flight Status lost for %.2f s, invalid", (now_t - last_flight_status_ts_).seconds());
      state_.djisdk_valid = false;
    }

    state_.vo_latency = (now_t - last_vo_image_ts_).seconds();
    if (state_.vo_valid && state_.vo_latency > param_.max_vo_latency) {
      state_.vo_valid = false;
      RCLCPP_INFO(this->get_logger(), "VO lost for %.2f s, invalid", state_.vo_latency);
    }

    if (state_.rc_valid && (now_t - last_rc_ts_).seconds() > MAX_LOSS_RC ) {
      state_.rc_valid = false;
      RCLCPP_INFO(this->get_logger(), "RC lost for %.2f s, invalid", (now_t - last_rc_ts_).seconds());
    }

    if (state_.onboard_cmd_valid && (now_t - last_onboard_cmd_ts_).seconds() > MAX_LOSS_ONBOARD_CMD ) {
      state_.onboard_cmd_valid = false;
      RCLCPP_INFO(this->get_logger(), "ONBOARD cmd lost for %.2f s, invalid", (now_t - last_onboard_cmd_ts_).seconds());
    }

    // Debug print every 10 cycles
    static int count = 0;
    if (count++ % 10 == 0) {
      std::printf("P[%.2f, %.2f, %.2f] TGT [%.2f, %.2f, %.2f]\n"
                  "ctrl_input_state %d, flight_status %d ctrl_auth %d ctrl_mode %d armed %d in_air %d rc_valid %d onboard_cmd %d vo_valid %d sdk_valid %d\n",
                  odometry_.pose.pose.position.x,
                  odometry_.pose.pose.position.y,
                  odometry_.pose.pose.position.z,
                  ctrl_cmd_->pos_sp.x,
                  ctrl_cmd_->pos_sp.y,
                  ctrl_cmd_->pos_sp.z,
                  state_.ctrl_input_state,
                  state_.flight_status,
                  state_.control_auth,
                  state_.commander_ctrl_mode,
                  state_.is_armed,
                  (state_.flight_status == DCMD::FLIGHT_STATUS_IN_AIR),
                  state_.rc_valid,
                  state_.onboard_cmd_valid,
                  state_.vo_valid,
                  state_.djisdk_valid
      );
    }

    if (!state_.djisdk_valid) {
      commander_state_pub_->publish(state_);
      return;
    }

    if (!yaw_sp_inited_) {
      resetYawSp();
    }

    if (!(state_.control_auth == DCMD::CTRL_AUTH_THIS)) {
      resetCtrlCmd();
    }

    processInputSource();
    processControlMode();
    processControl();

    // Publish the updated commander state
    commander_state_pub_->publish(state_);
  }

  // ==================== Helper methods start ====================

  /**
   * @brief Attempt to call /mavros/cmd/arming service
   */
  bool callArmService(bool arm)
  {
    if (!arm_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Arming client not ready!");
      return false;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    auto future = arm_client_->async_send_request(req);

    // Spin until we get the result
    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      bool ok = future.get()->success;
      RCLCPP_INFO(this->get_logger(), "Try arm=%d success=%d", arm, ok);
      return ok;
    } else {
      RCLCPP_ERROR(this->get_logger(), "callArmService() failed to call service");
      return false;
    }
  }

  void tryArm(bool arm)
  {
    if (arm == state_.is_armed) {
      return;
    }
    if (fail_arm_times_ > MAX_TRY_ARM_TIMES) {
      RCLCPP_INFO(this->get_logger(), "Fail arm too many times, giving up. Request IDLE!");
      requestCtrlMode(DCMD::CTRL_MODE_IDLE);
      return;
    }
    if (!arm) {
      requestCtrlMode(DCMD::CTRL_MODE_IDLE);
    }
    if (state_.djisdk_valid && state_.flight_status == DCMD::FLIGHT_STATUS_IDLE && arm) {
      bool res = callArmService(arm);
      if (!res) {
        fail_arm_times_++;
      }
    }
    if (state_.djisdk_valid && !arm) {
      bool res = callArmService(arm);
      if (!res) {
        fail_arm_times_++;
      }
    }
    last_try_arm_time_ = this->now();
  }

  void tryControlAuth(bool auth)
  {
    if (!state_.is_armed) {
      // May auth only if armed
      return;
    }
    if (!control_auth_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "SetMode service not ready!");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    if (auth) {
      req->custom_mode = "OFFBOARD";
    } else {
      req->custom_mode = "ALTCTL";
    }
    auto future = control_auth_client_->async_send_request(req);
    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      bool result_ok = future.get()->mode_sent;
      if (!result_ok) {
        RCLCPP_INFO(this->get_logger(), "SetMode %s failed", req->custom_mode.c_str());
      }
    }
  }

  bool needControlByThis()
  {
    if (param_.is_px4) {
      return rcRequestVo() || rcRequestOnboard();
    } else {
      return rcRequestVo();
    }
  }

  bool checkControlAuth()
  {
    bool require_auth_this = true;
    if (!state_.djisdk_valid) {
      return false;
    }
    if (state_.rc_valid) {
      if (needControlByThis()) {
        require_auth_this = true;
      } else {
        require_auth_this = false;
      }
    }
    if (!state_.rc_valid) {
      // If RC is invalid, still try to get auth
      require_auth_this = true;
    }
    if ((require_auth_this && state_.control_auth != DCMD::CTRL_AUTH_THIS) ||
        (!require_auth_this && state_.control_auth == DCMD::CTRL_AUTH_THIS))
    {
      tryControlAuth(require_auth_this);
    }
    return (state_.control_auth == DCMD::CTRL_AUTH_THIS);
  }

  // ==================== Callbacks ====================
  void voCallbackImage(const nav_msgs::msg::Odometry & odom)
  {
    last_vo_image_ts_ = odom.header.stamp;
  }

  void voCallback(const nav_msgs::msg::Odometry & odom)
  {
    bool vo_valid = isOdomValid(odom);
    auto pose = odom.pose.pose;
    auto quat = FLU2NED(Eigen::Quaterniond(pose.orientation.w,
                                           pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z));
    Eigen::Vector3d rpy = quat2eulers(quat);
    yaw_vo_ = rpy.z();

    if (!state_.vo_valid && vo_valid) {
      // VO becomes valid for the first time => reset yaw setpoint with VO yaw
      resetYawSp();
    }
    state_.vo_valid = vo_valid;
    if (state_.vo_valid) {
      odometry_ = odom;
      last_vo_ts_ = odometry_.header.stamp;
    }

    state_.pos.x = pose.position.x;
    state_.pos.y = pose.position.y;
    state_.pos.z = pose.position.z;
    state_.vel.x = odom.twist.twist.linear.x;
    state_.vel.y = odom.twist.twist.linear.y;
    state_.vel.z = odom.twist.twist.linear.z;
    state_.yaw = yaw_vo_;
  }

  void rcCallback(const sensor_msgs::msg::Joy & joy)
  {
    state_.rc_valid = isRcValid(joy);
    if (state_.rc_valid) {
      rc_ = joy;
      last_rc_ts_ = this->now();
    }
    state_.djisdk_valid = true;
  }

  void rcMavrosCallback(const mavros_msgs::msg::RCIn & rc_in)
  {
    // Convert RCIn to Joy style
    if (rc_.axes.size() < rc_in.channels.size()) {
      rc_.axes.resize(rc_in.channels.size());
    }
    for (size_t i = 0; i < rc_in.channels.size(); i++) {
      rc_.axes[i] = rc_in.channels[i];
    }
    last_rc_ts_ = this->now();
    state_.rc_valid = true;
    state_.djisdk_valid = true;
    last_flight_status_ts_ = this->now();
  }

  void batteryCallback(const sensor_msgs::msg::BatteryState & bat)
  {
    state_.bat_vol = bat.voltage;
    double battery_life_tmp = param_.battery_remain_param_a * state_.bat_vol + param_.battery_remain_param_b;
    state_.bat_remain = lowpassFilter(battery_life_tmp, 2.0, state_.bat_remain, 0.1);

    // If battery is too low, force landing
    if (state_.bat_remain <= param_.battery_remain_cutoff &&
        state_.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
      state_.landing_mode = DCMD::LANDING_MODE_XYVEL;
      state_.landing_velocity = LANDING_VEL_Z_BATTERY_LOW;
      requestCtrlMode(DCMD::CTRL_MODE_LANDING);
      processControlLanding();
    }
  }

  void fcStateCallback(const mavros_msgs::msg::State & st)
  {
    state_.is_armed = st.armed;
    state_.djisdk_valid = true;
    if (st.mode == "OFFBOARD") {
      state_.control_auth = DCMD::CTRL_AUTH_THIS;
    } else {
      state_.control_auth = DCMD::CTRL_AUTH_RC;
    }
    if (st.system_status == 3 || !state_.is_armed) {
      state_.flight_status = DCMD::FLIGHT_STATUS_IDLE;
    }
    if (state_.is_armed && state_.flight_status == DCMD::FLIGHT_STATUS_IDLE) {
      state_.flight_status = DCMD::FLIGHT_STATUS_ARMED;
    }
    last_flight_status_ts_ = this->now();
  }

  void fcExtendedStateCallback(const mavros_msgs::msg::ExtendedState & est)
  {
    if (est.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR) {
      state_.flight_status = DCMD::FLIGHT_STATUS_IN_AIR;
    } else if (est.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND && state_.is_armed) {
      state_.flight_status = DCMD::FLIGHT_STATUS_ARMED;
    }
  }

  void onboardCmdCallback(const OCMD & cmd)
  {
    state_.onboard_cmd_valid = true;
    last_onboard_cmd_ts_ = this->now();
    if (state_.ctrl_input_state != DCMD::CTRL_INPUT_ONBOARD) {
      processInputSource();
    }
    if (state_.ctrl_input_state == DCMD::CTRL_INPUT_ONBOARD) {
      handleOnboardCommand(cmd);
    }
  }

  void onImuData(const sensor_msgs::msg::Imu & imu)
  {
    state_.imu_data = imu;
    last_flight_status_ts_ = this->now();
  }

  void onImuDataFused(const sensor_msgs::msg::Imu & imu)
  {
    auto quat = Eigen::Quaterniond(imu.orientation.w,
                                   imu.orientation.x,
                                   imu.orientation.y,
                                   imu.orientation.z);
    auto q_ned = ENU2NED(quat);
    Eigen::Vector3d rpy = quat2eulers(q_ned);
    yaw_fc_ = rpy.z();
    last_flight_status_ts_ = this->now();
  }

  // ==================== Control logic (like original) ====================
  void processInputSource()
  {
    // If no input
    if (state_.ctrl_input_state == DCMD::CTRL_INPUT_NONE) {
      if (state_.rc_valid) {
        state_.ctrl_input_state = DCMD::CTRL_INPUT_RC;
        RCLCPP_INFO(this->get_logger(), "Change Source to RC");
      } else if (state_.onboard_cmd_valid) {
        state_.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;
        RCLCPP_INFO(this->get_logger(), "Change Source to Onboard because RC invalid but Onboard valid");
      }
    }

    // If currently using RC
    if (state_.ctrl_input_state == DCMD::CTRL_INPUT_RC) {
      if (!state_.rc_valid) {
        state_.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
        RCLCPP_INFO(this->get_logger(), "Change Source to None because RC Failure");
        if (state_.onboard_cmd_valid) {
          state_.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;
          RCLCPP_INFO(this->get_logger(), "Change Source to Onboard because RC Failure but Onboard valid");
        }
      } else if (rcRequestOnboard() && state_.onboard_cmd_valid) {
        state_.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;
        RCLCPP_INFO(this->get_logger(), "Change Source to Onboard because RC request & Onboard valid");
      }
    }

    // If currently using Onboard
    if (state_.ctrl_input_state == DCMD::CTRL_INPUT_ONBOARD) {
      if (!state_.onboard_cmd_valid) {
        if (state_.rc_valid) {
          state_.ctrl_input_state = DCMD::CTRL_INPUT_RC;
          RCLCPP_INFO(this->get_logger(), "Onboard invalid => switch to RC");
        } else {
          state_.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
          RCLCPP_INFO(this->get_logger(), "Onboard invalid => switch to None");
        }
      }
    }

    switch (state_.ctrl_input_state) {
      case DCMD::CTRL_INPUT_RC:
        processRcInput();
        break;
      case DCMD::CTRL_INPUT_ONBOARD:
        processOnboardInput();
        break;
      default:
      case DCMD::CTRL_INPUT_NONE:
        processNoneInput();
        break;
    }
  }

  void processRcInput()
  {
    if (state_.control_auth != DCMD::CTRL_AUTH_THIS && !param_.is_px4) {
      state_.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
      return;
    }
    if (rcMovingStick()) {
      requestCtrlMode(DCMD::CTRL_MODE_POSVEL);
    } else {
      if (state_.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION &&
          state_.commander_ctrl_mode != DCMD::CTRL_MODE_TAKEOFF &&
          state_.commander_ctrl_mode != DCMD::CTRL_MODE_LANDING) {
        requestCtrlMode(DCMD::CTRL_MODE_HOVER);
      }
    }

    double y = 0;
    double x = 0;
    double r = 0;
    double z = 0;
    if (state_.rc_valid) {
      // rc_.axes[0..3] presumably: roll, pitch, thrust, yaw
      y = - superexpo((rc_.axes[0] - PWM_CENTER)/PWM_100);
      x =   superexpo((rc_.axes[1] - PWM_CENTER)/PWM_100);
      z =   superexpo((rc_.axes[2] - PWM_CENTER)/PWM_100);
      r =   superexpo((rc_.axes[3] - PWM_CENTER)/PWM_100);
    }

    switch (state_.commander_ctrl_mode) {
      case DCMD::CTRL_MODE_POSVEL:
      {
        ctrl_cmd_->yaw_sp = constrainAngle(ctrl_cmd_->yaw_sp + r * RC_MAX_YAW_RATE * LOOP_DURATION);
        double vxd = x * RC_MAX_TILT_VEL;
        double vyd = y * RC_MAX_TILT_VEL;

        // transform to ENU => NED or vice versa
        ctrl_cmd_->vel_sp.x =  vxd * std::cos(yaw_vo_) + vyd * std::sin(yaw_vo_);
        ctrl_cmd_->vel_sp.y = -vxd * std::sin(yaw_vo_) + vyd * std::cos(yaw_vo_);
        ctrl_cmd_->vel_sp.z =  z * RC_MAX_Z_VEL;

        if (!pos_sp_inited_) {
          ctrl_cmd_->pos_sp.x = odometry_.pose.pose.position.x;
          ctrl_cmd_->pos_sp.y = odometry_.pose.pose.position.y;
          ctrl_cmd_->pos_sp.z = odometry_.pose.pose.position.z;
          pos_sp_inited_ = true;
          resetYawSp();
        }

        if (state_.is_armed && state_.control_auth == DCMD::CTRL_AUTH_THIS) {
          if (state_.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
            ctrl_cmd_->pos_sp.x += ctrl_cmd_->vel_sp.x * LOOP_DURATION;
            ctrl_cmd_->pos_sp.y += ctrl_cmd_->vel_sp.y * LOOP_DURATION;
            ctrl_cmd_->pos_sp.z += ctrl_cmd_->vel_sp.z * LOOP_DURATION;
            ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_POS_MODE;
          } else {
            ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_VEL_MODE;
            pos_sp_inited_ = false;
          }
        } else {
          ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
          pos_sp_inited_ = false;
        }
        break;
      }
      case DCMD::CTRL_MODE_TAKEOFF:
      case DCMD::CTRL_MODE_LANDING:
      case DCMD::CTRL_MODE_MISSION:
        break;
      case DCMD::CTRL_MODE_HOVER:
        prepareControlHover();
        break;
      case DCMD::CTRL_MODE_IDLE:
      case DCMD::CTRL_MODE_ATT:
      case DCMD::CTRL_MODE_ALT:
      default:
      {
        resetYawSp();
        setAttSetpoint(-y * RC_MAX_TILT_ANGLE, -x * RC_MAX_TILT_ANGLE, r * RC_MAX_YAW_RATE, z, true, true, true);
        break;
      }
    }
  }

  void processOnboardInput()
  {
    if (rcMovingStick()) {
      state_.onboard_cmd_valid = false;
      state_.ctrl_input_state = DCMD::CTRL_INPUT_RC;
      RCLCPP_INFO(this->get_logger(), "Change Source to RC due to RC moving stick");
    }
  }

  void processNoneInput()
  {
    if (state_.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION &&
        state_.commander_ctrl_mode != DCMD::CTRL_MODE_TAKEOFF &&
        state_.commander_ctrl_mode != DCMD::CTRL_MODE_LANDING) {
      requestCtrlMode(DCMD::CTRL_MODE_HOVER);
    }
  }

  void processControl()
  {
    if (state_.control_auth != DCMD::CTRL_AUTH_THIS) {
      if (state_.commander_ctrl_mode == DCMD::CTRL_MODE_TAKEOFF) {
        processControlTakeoff();
      } else {
        state_.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
      }
      return;
    }
    switch (state_.commander_ctrl_mode) {
      case DCMD::CTRL_MODE_HOVER:
        prepareControlHover();
        processControlPosvel();
        break;
      case DCMD::CTRL_MODE_POSVEL:
        processControlPosvel();
        break;
      case DCMD::CTRL_MODE_ATT:
      case DCMD::CTRL_MODE_ALT:
        processControlAtt();
        break;
      case DCMD::CTRL_MODE_TAKEOFF:
        processControlTakeoff();
        break;
      case DCMD::CTRL_MODE_LANDING:
        processControlLanding();
        break;
      case DCMD::CTRL_MODE_MISSION:
        processControlMission();
        break;
      case DCMD::CTRL_MODE_IDLE:
      default:
        processControlIdle();
        break;
    }
  }

  void processControlIdle()
  {
    setAttSetpoint(0, 0, 0, 0, false);
    sendCtrlCmd();
  }

  void processControlPosvel()
  {
    bool is_cmd_valid = true;
    if (is_cmd_valid) {
      sendCtrlCmd();
    } else {
      RCLCPP_ERROR(this->get_logger(), "POSVEL control cmd invalid!");
    }
  }

  void processControlAtt()
  {
    bool is_cmd_valid = true;
    if (is_cmd_valid) {
      sendCtrlCmd();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Attitude control cmd invalid!");
    }
  }

  void processControlTakeoff()
  {
    bool is_in_air = (state_.flight_status == DCMD::FLIGHT_STATUS_IN_AIR);
    bool is_takeoff_finish = false;
    auto pos = odometry_.pose.pose.position;
    if (!state_.vo_valid) {
      takeoff_inited_ = false;
      requestCtrlMode(DCMD::CTRL_MODE_LANDING);
      return;
    }
    if (!takeoff_inited_) {
      if (is_in_air && pos.z > MIN_TAKEOFF_HEIGHT) {
        RCLCPP_INFO(this->get_logger(), "Already in air");
        is_takeoff_finish = true;
      }
      takeoff_inited_ = true;
      if (state_.vo_valid) {
        takeoff_origin_.x() = pos.x;
        takeoff_origin_.y() = pos.y;
        takeoff_origin_.z() = pos.z;
        RCLCPP_INFO(this->get_logger(), "Initing takeoff, origin: %.2f %.2f %.2f",
                    takeoff_origin_.x(), takeoff_origin_.y(), takeoff_origin_.z());
      }
    }
    if (!state_.is_armed) {
      RCLCPP_INFO(this->get_logger(), "Trying to takeoff but not armed => tryArm(true)");
      tryArm(true);
    }

    if (state_.vo_valid) {
      double z_err = std::fabs(pos.z - (takeoff_origin_.z() + state_.takeoff_target_height));
      if (z_err < MAX_AUTO_Z_ERROR && 
          std::fabs(pos.x - takeoff_origin_.x()) < MAX_AUTO_TILT_ERROR &&
          std::fabs(pos.y - takeoff_origin_.y()) < MAX_AUTO_TILT_ERROR) {
        is_takeoff_finish = true;
        RCLCPP_INFO(this->get_logger(), "Takeoff finish");
      }
    } else {
      is_takeoff_finish = is_in_air;
    }

    if (is_takeoff_finish) {
      RCLCPP_INFO(this->get_logger(), "Finish takeoff, switch to HOVER");
      requestCtrlMode(DCMD::CTRL_MODE_HOVER);
      if (state_.commander_ctrl_mode == DCMD::CTRL_MODE_HOVER) {
        setHoverTargetPosition(takeoff_origin_.x(),
                               takeoff_origin_.y(),
                               state_.takeoff_target_height + takeoff_origin_.z());
      }
      takeoff_inited_ = false;
      resetCtrlCmdMaxVel();
      return;
    }

    // Not finished yet
    if (is_in_air || (pos.z - takeoff_origin_.z() > MIN_TAKEOFF_HEIGHT)) {
      ctrl_cmd_->max_vel.z = state_.takeoff_velocity;
      setPosSetpoint(takeoff_origin_.x(),
                     takeoff_origin_.y(),
                     state_.takeoff_target_height + takeoff_origin_.z());
    } else {
      setVelSetpoint(0, 0, state_.takeoff_velocity);
    }
    sendCtrlCmd();
  }

  void processControlLanding()
  {
    bool is_landing_finish = (state_.flight_status < DCMD::FLIGHT_STATUS_IN_AIR);
    if (is_landing_finish) {
      RCLCPP_INFO(this->get_logger(), "Landing finished => disarm");
      is_landing_tail_ = false;
      is_touch_ground_ = true;
      tryArm(false);
      if (!state_.is_armed) {
        requestCtrlMode(DCMD::CTRL_MODE_IDLE);
        RCLCPP_INFO(this->get_logger(), "Landing done => IDLE");
      }
      return;
    }
    // Check for IMU-based ground contact?
    if (state_.imu_data.linear_acceleration.z > 15.0) {
      RCLCPP_INFO(this->get_logger(), "Detect ground contact => is_touch_ground_=true");
      is_touch_ground_ = true;
    }
    if (is_landing_tail_) {
      if (is_touch_ground_) {
        RCLCPP_INFO(this->get_logger(), "Touch ground, thrust=0.0");
        setAttSetpoint(0, 0, yaw_vo_, 0.0, false, false);
        is_landing_finish = true;
      } else {
        setAttSetpoint(0, 0, yaw_vo_, param_.landing_thrust, false, false);
      }
      sendCtrlCmd();
    } else {
      // Normal XY velocity landing
      if (state_.vo_valid && (state_.landing_mode == DCMD::LANDING_MODE_XYVEL)) {
        if (state_.pos.z > LANDING_ATT_MODE_HEIGHT) {
          setVelSetpoint(0, 0, state_.landing_velocity);
        } else {
          RCLCPP_INFO(this->get_logger(), "Switching to attitude landing tail-mode");
          is_landing_tail_ = true;
          setAttSetpoint(0, 0, yaw_vo_, state_.landing_velocity, true, false);
        }
      } else {
        setAttSetpoint(0, 0, yaw_vo_, param_.landing_thrust, false, false);
      }
      sendCtrlCmd();
    }
  }

  void processControlMission()
  {
    // Not fully implemented
    // ...
    sendCtrlCmd();
  }

  void requestCtrlMode(uint32_t req_mode)
  {
    switch (req_mode) {
      case DCMD::CTRL_MODE_LANDING:
      {
        if (state_.flight_status < DCMD::FLIGHT_STATUS_ARMED) {
          state_.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
        } else {
          state_.commander_ctrl_mode = req_mode;
        }
        return;
      }
      case DCMD::CTRL_MODE_TAKEOFF:
      {
        if (state_.flight_status == DCMD::FLIGHT_STATUS_IN_AIR &&
            state_.commander_ctrl_mode != DCMD::CTRL_MODE_TAKEOFF &&
            state_.commander_ctrl_mode != DCMD::CTRL_MODE_LANDING)
        {
          RCLCPP_INFO(this->get_logger(), "Already in air => directly hover");
          requestCtrlMode(DCMD::CTRL_MODE_HOVER);
        } else {
          state_.commander_ctrl_mode = req_mode;
        }
        break;
      }
      case DCMD::CTRL_MODE_MISSION:
      case DCMD::CTRL_MODE_HOVER:
      case DCMD::CTRL_MODE_POSVEL:
      {
        if (state_.flight_status < DCMD::FLIGHT_STATUS_IN_AIR) {
          state_.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
        } else {
          if (state_.vo_valid) {
            state_.commander_ctrl_mode = req_mode;
          } else {
            RCLCPP_WARN(this->get_logger(), "VO failed => emergency landing");
            state_.commander_ctrl_mode = DCMD::CTRL_MODE_LANDING;
            state_.landing_velocity = LANDING_VEL_Z_EMERGENCY;
            return;
          }
        }
        break;
      }
      case DCMD::CTRL_MODE_IDLE:
        state_.commander_ctrl_mode = req_mode;
        break;
      default:
      case DCMD::CTRL_MODE_ALT:
      case DCMD::CTRL_MODE_ATT:
      {
        if (state_.flight_status < DCMD::FLIGHT_STATUS_IN_AIR) {
          state_.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
        } else {
          state_.commander_ctrl_mode = req_mode;
        }
        break;
      }
    }

    if (state_.ctrl_input_state == DCMD::CTRL_INPUT_NONE) {
      if (state_.commander_ctrl_mode != DCMD::CTRL_MODE_LANDING &&
          state_.commander_ctrl_mode != DCMD::CTRL_MODE_TAKEOFF &&
          state_.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION)
      {
        if (req_mode != DCMD::CTRL_MODE_HOVER) {
          requestCtrlMode(DCMD::CTRL_MODE_HOVER);
        }
      }
    }
  }

  void processControlMode()
  {
    requestCtrlMode(state_.commander_ctrl_mode);
  }

  // ==================== Send Commands ====================
  void sendControlCmdPX4()
  {
    if (!state_.is_armed) {
      return;
    }
    mavros_msgs::msg::PositionTarget pos_target;
    pos_target.header.stamp = this->now();
    pos_target.header.frame_id = "world";
    pos_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    if ((ctrl_cmd_->ctrl_mode == DPCL::CTRL_CMD_POS_MODE) ||
        (ctrl_cmd_->ctrl_mode == DPCL::CTRL_CMD_VEL_MODE))
    {
      auto & vel_sp = ctrl_cmd_->vel_sp;
      auto & acc_sp = ctrl_cmd_->acc_sp;

      if (ctrl_cmd_->ctrl_mode == DPCL::CTRL_CMD_POS_MODE) {
        pos_target.position.x = ctrl_cmd_->pos_sp.x;
        pos_target.position.y = ctrl_cmd_->pos_sp.y;
        pos_target.position.z = ctrl_cmd_->pos_sp.z;
        if (std::fabs(vel_sp.x) < EPS && std::fabs(vel_sp.y) < EPS && std::fabs(vel_sp.z) < EPS) {
          pos_target.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_VX |
                                  mavros_msgs::msg::PositionTarget::IGNORE_VY |
                                  mavros_msgs::msg::PositionTarget::IGNORE_VZ;
        }
      } else {
        pos_target.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_PX |
                                mavros_msgs::msg::PositionTarget::IGNORE_PY |
                                mavros_msgs::msg::PositionTarget::IGNORE_PZ;
      }

      if (std::fabs(acc_sp.x) < EPS && std::fabs(acc_sp.y) < EPS && std::fabs(acc_sp.z) < EPS) {
        pos_target.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
      }
      pos_target.velocity = vel_sp;
      pos_target.acceleration_or_force = acc_sp;
      pos_target.yaw = ctrl_cmd_->yaw_sp;
      pos_target.yaw_rate = 0;
      control_pos_vel_px4_pub_->publish(pos_target);
    }

    if (ctrl_cmd_->ctrl_mode == DPCL::CTRL_CMD_ATT_VELZ_MODE) {
      Eigen::Vector3d acc_sp(0.0, 0.0, 9.8);
      Eigen::Quaterniond q_sp(ctrl_cmd_->att_sp.w,
                              ctrl_cmd_->att_sp.x,
                              ctrl_cmd_->att_sp.y,
                              ctrl_cmd_->att_sp.z);
      acc_sp = q_sp.toRotationMatrix() * acc_sp;

      pos_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                             mavros_msgs::msg::PositionTarget::IGNORE_PY |
                             mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                             mavros_msgs::msg::PositionTarget::IGNORE_VX |
                             mavros_msgs::msg::PositionTarget::IGNORE_VY;
      pos_target.velocity.x = 0.0;
      pos_target.velocity.y = 0.0;
      pos_target.velocity.z = ctrl_cmd_->z_sp;
      pos_target.acceleration_or_force.x = acc_sp.x();
      pos_target.acceleration_or_force.y = acc_sp.y();
      pos_target.acceleration_or_force.z = 0.0;
      pos_target.yaw = ctrl_cmd_->yaw_sp;
      pos_target.yaw_rate = 0;
      control_pos_vel_px4_pub_->publish(pos_target);
    } else if (ctrl_cmd_->ctrl_mode == DPCL::CTRL_CMD_ATT_THRUST_MODE) {
      mavros_msgs::msg::AttitudeTarget att_target;
      att_target.header.stamp = this->now();
      att_target.header.frame_id = "world";
      att_target.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                             mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                             mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
      att_target.orientation = ctrl_cmd_->att_sp;
      att_target.thrust = ctrl_cmd_->z_sp;
      control_att_pub_->publish(att_target);
    }
  }

  void sendCtrlCmd()
  {
    if (ctrl_cmd_->ctrl_mode != DPCL::CTRL_CMD_POS_MODE) {
      pos_sp_inited_ = false;
    }
    if (param_.use_px4_pos_ctrl) {
      sendControlCmdPX4();
    } else {
      if (!state_.is_armed || state_.control_auth != DCMD::CTRL_AUTH_THIS) {
        ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
      }
      ctrl_cmd_pub_->publish(*ctrl_cmd_);
    }
  }

  // ==================== Internal utility ====================
  bool setHoverTargetPosition(double x, double y, double z)
  {
    if (state_.is_armed && state_.vo_valid && 
        (state_.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) &&
        (state_.control_auth == DCMD::CTRL_AUTH_THIS))
    {
      hover_pos_ = Eigen::Vector3d(x, y, z);
      setPosSetpoint(x, y, z);
      last_hover_count_ = control_count_;
      return true;
    }
    return false;
  }

  void prepareControlHover()
  {
    bool fail_to_hover = false;
    if (last_hover_count_ < control_count_ - 1 && state_.is_armed && state_.vo_valid) {
      // Need to start a new hover
      auto vx = odometry_.twist.twist.linear.x;
      auto vy = odometry_.twist.twist.linear.y;
      auto vz = odometry_.twist.twist.linear.z;
      if (std::fabs(vx) > DANGER_SPEED_HOVER ||
          std::fabs(vy) > DANGER_SPEED_HOVER ||
          std::fabs(vz) > DANGER_SPEED_HOVER)
      {
        fail_to_hover = true;
      } else {
        bool succ = setHoverTargetPosition(odometry_.pose.pose.position.x,
                                           odometry_.pose.pose.position.y,
                                           odometry_.pose.pose.position.z);
        if (succ) {
          RCLCPP_INFO(this->get_logger(),
            "Entering hover mode => hover @ %.2f %.2f %.2f, yaw_sp=%.2f deg",
            hover_pos_.x(), hover_pos_.y(), hover_pos_.z(),
            ctrl_cmd_->yaw_sp * 57.3
          );
        } else {
          fail_to_hover = true;
        }
      }
    }
    if (fail_to_hover) {
      if (state_.is_armed && state_.control_auth == DCMD::CTRL_AUTH_THIS) {
        RCLCPP_INFO(this->get_logger(), "Trying to hover failed => emergency landing");
        state_.landing_mode = DCMD::LANDING_MODE_ATT;
        state_.landing_velocity = LANDING_VEL_Z_EMERGENCY;
        requestCtrlMode(DCMD::CTRL_MODE_LANDING);
      }
    } else {
      last_hover_count_ = control_count_;
    }
  }

  void resetCtrlCmd()
  {
    pos_sp_inited_ = false;
    last_hover_count_ = 0;
    takeoff_inited_ = false;
    ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    ctrl_cmd_->pos_sp.x = 0;
    ctrl_cmd_->pos_sp.y = 0;
    ctrl_cmd_->pos_sp.z = 0;
    ctrl_cmd_->vel_sp.x = 0;
    ctrl_cmd_->vel_sp.y = 0;
    ctrl_cmd_->vel_sp.z = 0;
    ctrl_cmd_->att_sp.w = 0;
    ctrl_cmd_->att_sp.x = 0;
    ctrl_cmd_->att_sp.y = 0;
    ctrl_cmd_->att_sp.z = 0;
    ctrl_cmd_->z_sp = 0;
    resetCtrlCmdMaxVel();
  }

  void resetCtrlCmdMaxVel()
  {
    ctrl_cmd_->max_vel.x = DEFAULT_MAX_TITL_VEL;
    ctrl_cmd_->max_vel.y = DEFAULT_MAX_TITL_VEL;
    ctrl_cmd_->max_vel.z = DEFAULT_MAX_Z_VEL;
  }

  bool isOdomValid(const nav_msgs::msg::Odometry & odom)
  {
    double vx = odom.twist.twist.linear.x;
    double vy = odom.twist.twist.linear.y;
    double vz = odom.twist.twist.linear.z;
    if (std::fabs(vx) > MAX_ODOM_VELOCITY ||
        std::fabs(vy) > MAX_ODOM_VELOCITY ||
        std::fabs(vz) > MAX_ODOM_VELOCITY)
    {
      return false;
    }
    auto now_t = this->now();
    if ((now_t - last_vo_image_ts_).seconds() > param_.max_vo_latency) {
      return false;
    }
    return true;
  }

  bool isRcValid(const sensor_msgs::msg::Joy & joy)
  {
    // In original code: `return px4_fcu_state.manual_input;`
    // We don't store that here, so let's assume if we get JOY message => valid
    return true;
  }

  void resetYawSp()
  {
    if (state_.djisdk_valid) {
      if (state_.vo_valid) {
        ctrl_cmd_->yaw_sp = yaw_vo_;
      } else {
        ctrl_cmd_->yaw_sp = yaw_fc_;
      }
      yaw_sp_inited_ = true;
    }
  }

  // ============ Setpoint methods =============
  void setAttSetpoint(double roll, double pitch, double yawrate, double z,
                      bool z_use_vel=true,
                      bool yaw_use_rate=true,
                      bool use_fc_yaw=false)
  {
    ctrl_cmd_->use_fc_yaw = use_fc_yaw;
    if (yaw_use_rate) {
      ctrl_cmd_->yaw_sp = constrainAngle(ctrl_cmd_->yaw_sp + yawrate * LOOP_DURATION);
    } else {
      ctrl_cmd_->yaw_sp = constrainAngle(yawrate);
    }
    Quaterniond quat_sp =
      AngleAxisd(ctrl_cmd_->yaw_sp, Vector3d::UnitZ()) *
      AngleAxisd(pitch, Vector3d::UnitY()) *
      AngleAxisd(roll, Vector3d::UnitX());
    ctrl_cmd_->att_sp.w = quat_sp.w();
    ctrl_cmd_->att_sp.x = quat_sp.x();
    ctrl_cmd_->att_sp.y = quat_sp.y();
    ctrl_cmd_->att_sp.z = quat_sp.z();
    ctrl_cmd_->z_sp = z;
    if ((state_.is_armed && state_.control_auth == DCMD::CTRL_AUTH_THIS) || param_.is_px4) {
      if (z_use_vel) {
        ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_ATT_VELZ_MODE;
      } else {
        ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_ATT_THRUST_MODE;
      }
    } else {
      ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    }
  }

  void setPosSetpoint(double x, double y, double z,
                      double yaw = std::numeric_limits<double>::quiet_NaN(),
                      double vx_ff = 0, double vy_ff = 0, double vz_ff = 0,
                      double ax_ff = 0, double ay_ff = 0, double az_ff = 0)
  {
    ctrl_cmd_->pos_sp.x = x;
    ctrl_cmd_->pos_sp.y = y;
    ctrl_cmd_->pos_sp.z = z;
    ctrl_cmd_->vel_sp.x = vx_ff;
    ctrl_cmd_->vel_sp.y = vy_ff;
    ctrl_cmd_->vel_sp.z = vz_ff;
    ctrl_cmd_->acc_sp.x = ax_ff;
    ctrl_cmd_->acc_sp.y = ay_ff;
    ctrl_cmd_->acc_sp.z = az_ff;
    ctrl_cmd_->use_fc_yaw = false;
    if (!std::isnan(yaw)) {
      ctrl_cmd_->yaw_sp = constrainAngle(yaw);
    }
    if ((state_.is_armed && state_.control_auth == DCMD::CTRL_AUTH_THIS) || param_.is_px4) {
      ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_POS_MODE;
    } else {
      ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    }
  }

  void setVelSetpoint(double vx, double vy, double vz,
                      double yaw = std::numeric_limits<double>::quiet_NaN(),
                      double ax_ff = 0, double ay_ff = 0, double az_ff = 0)
  {
    ctrl_cmd_->vel_sp.x = vx;
    ctrl_cmd_->vel_sp.y = vy;
    ctrl_cmd_->vel_sp.z = vz;
    ctrl_cmd_->acc_sp.x = ax_ff;
    ctrl_cmd_->acc_sp.y = ay_ff;
    ctrl_cmd_->acc_sp.z = az_ff;
    if (!std::isnan(yaw)) {
      ctrl_cmd_->yaw_sp = constrainAngle(yaw);
    }
    ctrl_cmd_->use_fc_yaw = false;
    if ((state_.is_armed && state_.control_auth == DCMD::CTRL_AUTH_THIS) || param_.is_px4) {
      ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_VEL_MODE;
    } else {
      ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    }
  }

  /**
   * @brief Handling the OnboardCommand as in original code.
   */
  void handleOnboardCommand(const OCMD & _cmd)
  {
    switch (_cmd.command_type) {
      case OCMD::CTRL_POS_COMMAND:
      {
        requestCtrlMode(DCMD::CTRL_MODE_POSVEL);
        double x = ((double)_cmd.param1) / 10000.0;
        double y = ((double)_cmd.param2) / 10000.0;
        double z = ((double)_cmd.param3) / 10000.0;
        double yaw = ((double)_cmd.param4) / 10000.0;
        double vx_ff = ((double)_cmd.param5) / 10000.0;
        double vy_ff = ((double)_cmd.param6) / 10000.0;
        double vz_ff = ((double)_cmd.param7) / 10000.0;
        double ax_ff = ((double)_cmd.param8) / 10000.0;
        double ay_ff = ((double)_cmd.param9) / 10000.0;
        double az_ff = ((double)_cmd.param10) / 10000.0;

        if (_cmd.param4 == MAGIC_YAW_NAN) {
          setPosSetpoint(x, y, z, NAN, vx_ff, vy_ff, vz_ff, ax_ff, ay_ff, az_ff);
        } else {
          setPosSetpoint(x, y, z, yaw, vx_ff, vy_ff, vz_ff, ax_ff, ay_ff, az_ff);
        }
        break;
      }
      case OCMD::CTRL_VEL_COMMAND:
      {
        requestCtrlMode(DCMD::CTRL_MODE_POSVEL);
        double vx = ((double)_cmd.param1) / 10000.0;
        double vy = ((double)_cmd.param2) / 10000.0;
        double vz = ((double)_cmd.param3) / 10000.0;
        double yaw = ((double)_cmd.param4) / 10000.0;
        double ax_ff = ((double)_cmd.param5) / 10000.0;
        double ay_ff = ((double)_cmd.param6) / 10000.0;
        double az_ff = ((double)_cmd.param7) / 10000.0;

        if (_cmd.param4 == MAGIC_YAW_NAN) {
          // original code had a bug? We'll just setVelSetpoint
          setVelSetpoint(vx, vy, vz, NAN, ax_ff, ay_ff, az_ff);
        } else {
          setVelSetpoint(vx, vy, vz, yaw, ax_ff, ay_ff, az_ff);
        }
        break;
      }
      case OCMD::CTRL_ATT_COMMAND:
      {
        requestCtrlMode(DCMD::CTRL_MODE_ATT);
        double roll    = ((double)_cmd.param1) / 10000.0;
        double pitch   = ((double)_cmd.param2) / 10000.0;
        double yawrate = ((double)_cmd.param3) / 10000.0;
        double z       = ((double)_cmd.param4) / 10000.0;
        bool z_use_vel   = (_cmd.param5 == 0);
        bool yaw_use_rate= (_cmd.param6 == 0);
        setAttSetpoint(roll, pitch, yawrate, z, z_use_vel, yaw_use_rate);
        break;
      }
      case OCMD::CTRL_MISSION_LOAD_COMMAND:
      {
        requestCtrlMode(DCMD::CTRL_MODE_MISSION);
        break;
      }
      case OCMD::CTRL_MISSION_END_COMMAND:
      {
        requestCtrlMode(DCMD::CTRL_MODE_HOVER);
        break;
      }
      case OCMD::CTRL_TAKEOF_COMMAND:
      {
        fail_arm_times_ = 0;
        double h = ((double)_cmd.param1) / 10000.0;
        if (h < MIN_TAKEOFF_HEIGHT) {
          h = MIN_TAKEOFF_HEIGHT;
        }
        RCLCPP_INFO(this->get_logger(), "Onboard => TAKEOFF => target=%.2f", h);
        requestCtrlMode(DCMD::CTRL_MODE_TAKEOFF);
        state_.takeoff_target_height = h;
        state_.takeoff_velocity = ((double)_cmd.param2) / 10000.0;
        break;
      }
      case OCMD::CTRL_LANDING_COMMAND:
      {
        RCLCPP_INFO(this->get_logger(), "Onboard => LANDING");
        if (_cmd.param1 < 0) {
          state_.landing_mode = DCMD::LANDING_MODE_ATT;
          is_landing_tail_ = true;
        } else if (_cmd.param1 == 1) {
          state_.landing_mode = DCMD::LANDING_MODE_ATT;
        } else {
          state_.landing_mode = DCMD::LANDING_MODE_XYVEL;
          is_landing_tail_ = false;
          is_touch_ground_ = false;
        }
        state_.landing_velocity = -((double)_cmd.param2) / 10000.0;
        requestCtrlMode(DCMD::CTRL_MODE_LANDING);
        break;
      }
      case OCMD::CTRL_HOVER_COMMAND:
      {
        requestCtrlMode(DCMD::CTRL_MODE_HOVER);
        break;
      }
      case OCMD::CTRL_ARM_COMMAND:
      {
        fail_arm_times_ = 0;
        RCLCPP_INFO(this->get_logger(), "Onboard => ARM=%d", _cmd.param1);
        tryArm(_cmd.param1 > 0);
        break;
      }
      default:
        break;
    }
  }

  bool rcRequestOnboard()
  {
    // In original code: (rc.axes[6] > 1800 && rc.axes[7] > 1800)
    if (rc_.axes.size() > 7) {
      return (rc_.axes[6] > 1800 && rc_.axes[7] > 1800);
    }
    return false;
  }

  bool rcRequestVo()
  {
    // In original code: (rc.axes[6] > 1800)
    if (rc_.axes.size() > 6) {
      return (rc_.axes[6] > 1800);
    }
    return false;
  }

  bool rcMovingStick()
  {
    if (!state_.rc_valid) return false;
    if (rc_.axes.size() < 4) return false;
    bool if_move = std::fabs(rc_.axes[0] - PWM_CENTER) > PWM_DEADZONE_RPY;
    if_move = if_move || (std::fabs(rc_.axes[1] - PWM_CENTER) > PWM_DEADZONE_RPY);
    if_move = if_move || (std::fabs(rc_.axes[3] - PWM_CENTER) > PWM_DEADZONE_RPY);
    if_move = if_move || (std::fabs(rc_.axes[2] - PWM_CENTER) > PWM_DEADZONE_THR);
    return if_move;
  }

  // Low pass filter
  double lowpassFilter(double input, double fc, double output_last, double dt)
  {
    double RC = 1.0 / (fc * 2.0 * M_PI);
    double alpha = dt / (RC + dt);
    return output_last + alpha * (input - output_last);
  }

  // Convert FLU -> NED
  Eigen::Quaterniond FLU2NED(const Eigen::Quaterniond & q)
  {
    Eigen::Matrix3d R = R_FLU2FRD_ * q.toRotationMatrix() * R_FLU2FRD_;
    return Eigen::Quaterniond(R);
  }

  // Convert ENU -> NED
  Eigen::Quaterniond ENU2NED(const Eigen::Quaterniond & q)
  {
    Eigen::Matrix3d R = R_ENU2NED_ * q.toRotationMatrix() * R_FLU2FRD_;
    return Eigen::Quaterniond(R);
  }
}; // end class DroneCommander


int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SWARM_COMMANDER_CONTROL_INIT: Initializing DroneCommander node...");
  
  // Create node
  auto node = std::make_shared<DroneCommander>();

  // Multi-threaded spinner
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
