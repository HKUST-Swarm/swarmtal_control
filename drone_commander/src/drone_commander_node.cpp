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

#include "drone_commander.h"

#include <rclcpp/logging.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/clock.hpp>

#include <cmath>
#include <chrono>
#include <cstdio>
#include <string>
#include <limits>

static const int MAGIC_YAW_NAN = 666666; // For reference from original code
static const double EPS=0.01;

using namespace Eigen;

// ============= Free function for quaternion -> Euler =============
Eigen::Vector3d quat2eulers(const Eigen::Quaterniond & quat)
{
  Eigen::Vector3d rpy;
  rpy.x() = std::atan2(2.0 * (quat.w() * quat.x() + quat.y() * quat.z()),
                       1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
  rpy.y() = std::asin(2.0 * (quat.w() * quat.y() - quat.z() * quat.x()));
  rpy.z() = std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
                       1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
  return rpy;
}

inline double constrainAngle(double x){
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

inline double float_constrain(double v, double min, double max)
{
    if (v < min) {
        return min;
    }
    if (v > max) {
        return max;
    }
    return v;
}

inline double expo(const double &value, const double &e)
{
	double x = float_constrain(value, - 1, 1);
	double ec = float_constrain(e, 0, 1);
	return (1 - ec) * x + ec * x * x * x;
}

inline const double superexpo(const double &value, double e = 0.5, double g = 0.5)
{
	double x = float_constrain(value, - 1, 1);
	double gc = float_constrain(g, 0, 0.99);
	return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
}

// ===================================================================
// DroneCommander constructor
// ===================================================================
DroneCommander::DroneCommander()
: Node("drone_commander")
{
  RCLCPP_INFO(this->get_logger(), "DroneCommander node is initializing...");

  // Initialize transform matrices
  R_ENU2NED_ <<  0,  1,  0,
                 1,  0,  0,
                 0,  0, -1;

  R_FLU2FRD_ <<   1,  0,  0,
                  0, -1,  0,
                  0,  0, -1;

  initStates();

  // 首先声明所有参数并给定默认值
  declareAllParameters();
  // 从参数服务器获取真实的值
  getAllParameters();

  // 让 ctrl_cmd_ 指向 state_.ctrl_cmd
  ctrl_cmd_ = &state_.ctrl_cmd;

  initROS2Interfaces();

  boot_time_ = this->now();
  last_flight_status_ts_ = this->now();
  last_rc_ts_ = this->now();
  last_vo_ts_ = this->now();
  last_vo_image_ts_ = this->now();
  last_onboard_cmd_ts_ = this->now();
  last_try_arm_time_ = this->now();
  last_send_odom_to_fc_ = this->now();

  // Create main control loop timer
  loop_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(param_.loop_duration),
    std::bind(&DroneCommander::loopTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "DroneCommander node has been initialized. Spinning...");
}

void DroneCommander::initStates()
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
}

void DroneCommander::declareAllParameters()
{
  // 将原先宏定义的默认值放到这里
  // 1) 先声明 param_.xxx，给出默认值
  // 2) 用户也可以通过命令行/launch文件修改
  this->declare_parameter<int>("drone_id",                   1);
  this->declare_parameter<double>("max_loss_rc",             1.0);
  this->declare_parameter<double>("max_loss_sdk",            1.0);
  this->declare_parameter<double>("max_odom_velocity",       25.0);
  this->declare_parameter<double>("rc_deadzone_rpy",         0.1);
  this->declare_parameter<double>("rc_deadzone_thrust",      0.2);
  this->declare_parameter<double>("pwm_center",              1500.0);
  this->declare_parameter<double>("pwm_100",                 500.0);
  this->declare_parameter<double>("pwm_deadzone_rpy",        50.0);
  this->declare_parameter<double>("pwm_deadzone_thr",        100.0);
  this->declare_parameter<double>("rc_max_tilt_vel",         3.0);
  this->declare_parameter<double>("rc_max_z_vel",            2.0);
  this->declare_parameter<double>("default_max_tilt_vel",    5.0);
  this->declare_parameter<double>("default_max_z_vel",       3.0);
  this->declare_parameter<double>("rc_max_yaw_rate",         1.57);
  this->declare_parameter<double>("rc_max_tilt_angle",       0.52);
  this->declare_parameter<double>("takeoff_vel_z",           1.0);
  this->declare_parameter<double>("landing_vel_z",           -0.3);
  this->declare_parameter<double>("landing_vel_z_emergency", -2.0);
  this->declare_parameter<double>("max_auto_z_error",        0.05);
  this->declare_parameter<double>("max_auto_tilt_error",     0.05);
  this->declare_parameter<double>("min_takeoff_height",      0.5);
  this->declare_parameter<double>("min_try_arm_duration",    1.0);
  this->declare_parameter<int>("max_try_arm_times",          5);
  this->declare_parameter<double>("max_loss_onboard_cmd",    60.0);
  this->declare_parameter<double>("landing_att_mode_height", 0.1);
  this->declare_parameter<double>("landing_att_min_height",  0.1);
  this->declare_parameter<double>("loop_duration",           0.02);
  this->declare_parameter<double>("danger_speed_hover",       4.5);
  this->declare_parameter<double>("landing_vel_z_battery_low", -0.5);

  // 其他原先 DroneCommanderParam 中的参数
  this->declare_parameter<bool>("use_px4_pos_ctrl", true);
  this->declare_parameter<double>("max_vo_latency", 0.4);
  this->declare_parameter<double>("battery_remain_cutoff", 240.0);
  this->declare_parameter<double>("battery_remain_param_a", 345.375);
  this->declare_parameter<double>("battery_remain_param_b", -4757.3);
  this->declare_parameter<double>("landing_thrust", 0.2);
}

void DroneCommander::getAllParameters()
{
  param_.drone_id               = this->get_parameter("drone_id").as_int();
  param_.max_loss_rc             = this->get_parameter("max_loss_rc").as_double();
  param_.max_loss_sdk            = this->get_parameter("max_loss_sdk").as_double();
  param_.max_odom_velocity       = this->get_parameter("max_odom_velocity").as_double();
  param_.rc_deadzone_rpy         = this->get_parameter("rc_deadzone_rpy").as_double();
  param_.rc_deadzone_thrust      = this->get_parameter("rc_deadzone_thrust").as_double();
  param_.pwm_center              = this->get_parameter("pwm_center").as_double();
  param_.pwm_100                 = this->get_parameter("pwm_100").as_double();
  param_.pwm_deadzone_rpy        = this->get_parameter("pwm_deadzone_rpy").as_double();
  param_.pwm_deadzone_thr        = this->get_parameter("pwm_deadzone_thr").as_double();
  param_.rc_max_tilt_vel         = this->get_parameter("rc_max_tilt_vel").as_double();
  param_.rc_max_z_vel            = this->get_parameter("rc_max_z_vel").as_double();
  param_.default_max_tilt_vel    = this->get_parameter("default_max_tilt_vel").as_double();
  param_.default_max_z_vel       = this->get_parameter("default_max_z_vel").as_double();
  param_.rc_max_yaw_rate         = this->get_parameter("rc_max_yaw_rate").as_double();
  param_.rc_max_tilt_angle       = this->get_parameter("rc_max_tilt_angle").as_double();
  param_.takeoff_vel_z           = this->get_parameter("takeoff_vel_z").as_double();
  param_.landing_vel_z           = this->get_parameter("landing_vel_z").as_double();
  param_.landing_vel_z_emergency = this->get_parameter("landing_vel_z_emergency").as_double();
  param_.max_auto_z_error        = this->get_parameter("max_auto_z_error").as_double();
  param_.max_auto_tilt_error     = this->get_parameter("max_auto_tilt_error").as_double();
  param_.min_takeoff_height      = this->get_parameter("min_takeoff_height").as_double();
  param_.min_try_arm_duration    = this->get_parameter("min_try_arm_duration").as_double();
  param_.max_try_arm_times       = this->get_parameter("max_try_arm_times").as_int();
  param_.max_loss_onboard_cmd    = this->get_parameter("max_loss_onboard_cmd").as_double();
  param_.landing_att_mode_height = this->get_parameter("landing_att_mode_height").as_double();
  param_.landing_att_min_height  = this->get_parameter("landing_att_min_height").as_double();
  param_.loop_duration           = this->get_parameter("loop_duration").as_double();
  param_.danger_speed_hover      = this->get_parameter("danger_speed_hover").as_double();
  param_.landing_vel_z_battery_low = this->get_parameter("landing_vel_z_battery_low").as_double();

  param_.use_px4_pos_ctrl        = this->get_parameter("use_px4_pos_ctrl").as_bool();
  param_.max_vo_latency          = this->get_parameter("max_vo_latency").as_double();
  param_.battery_remain_cutoff   = this->get_parameter("battery_remain_cutoff").as_double();
  param_.battery_remain_param_a  = this->get_parameter("battery_remain_param_a").as_double();
  param_.battery_remain_param_b  = this->get_parameter("battery_remain_param_b").as_double();
  param_.landing_thrust          = this->get_parameter("landing_thrust").as_double();

  // 打印日志
  RCLCPP_INFO(this->get_logger(), "DroneCommander: init at drone %d", param_.drone_id);
  RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
  RCLCPP_INFO(this->get_logger(), 
    "max_loss_rc=%.2f, max_loss_sdk=%.2f, max_odom_velocity=%.2f, rc_deadzone_rpy=%.2f, rc_deadzone_thrust=%.2f, loop_duration=%.2f",
    param_.max_loss_rc, param_.max_loss_sdk, param_.max_odom_velocity,
    param_.rc_deadzone_rpy, param_.rc_deadzone_thrust, param_.loop_duration
  );
  RCLCPP_INFO(this->get_logger(),
    "rc_max_tilt_vel=%.2f, rc_max_z_vel=%.2f, rc_max_yaw_rate=%.2f, rc_max_tilt_angle=%.2f, param_.danger_speed_hover=%.2f",
    param_.rc_max_tilt_vel, param_.rc_max_z_vel, param_.rc_max_yaw_rate,
    param_.rc_max_tilt_angle, param_.danger_speed_hover
  );
}

void DroneCommander::initROS2Interfaces()
{
  // Publishers
  commander_state_pub_ = this->create_publisher<DCMD>("drone_commander/swarm_commander_state", 1);
  ctrl_cmd_pub_ = this->create_publisher<DPCL>("drone_position_control/drone_pos_cmd", 1);
  control_pos_vel_px4_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 1);
  control_att_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
  mavros_system_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>("mavros/companion_process/status", 1);
  mavros_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("mavros/odometry/out", 10);

  // Subscriptions
  vo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "visual_odometry",
    1,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->voCallback(*msg); }
  );
  vo_sub_slow_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "visual_odometry_image",
    10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg){ this->voCallbackImage(*msg); }
  );
  onboard_cmd_sub_ = this->create_subscription<OCMD>(
    "drone_commander/onboard_command",
    10,
    [this](const OCMD::SharedPtr cmd){ this->onboardCmdCallback(*cmd); }
  );
  rc_mavros_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
    "mavros/rc/in",
    1,
    [this](const mavros_msgs::msg::RCIn::SharedPtr rc){ this->rcMavrosCallback(*rc); }
  );
  bat_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "mavros/battery",
    1,
    [this](const sensor_msgs::msg::BatteryState::SharedPtr bat){ this->batteryCallback(*bat); }
  );
  imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "mavros/imu/data_raw",
    1,
    [this](const sensor_msgs::msg::Imu::SharedPtr imu){ this->onImuData(*imu); }
  );
  imu_fused_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "mavros/imu/data",
    1,
    [this](const sensor_msgs::msg::Imu::SharedPtr imu){ this->onImuDataFused(*imu); }
  );
  fc_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "mavros/state",
    10,
    [this](const mavros_msgs::msg::State::SharedPtr st){ this->fcStateCallback(*st); }
  );
  fc_extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
    "mavros/extended_state",
    10,
    [this](const mavros_msgs::msg::ExtendedState::SharedPtr est){ this->fcExtendedStateCallback(*est); }
  );

  // Service clients
  control_auth_client_   = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  drone_landing_control_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
  arm_client_            = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

  RCLCPP_INFO(this->get_logger(), "DroneCommander: services and topics ready (if found).");
}

// ===================================================================
// loopTimerCallback
// ===================================================================
void DroneCommander::loopTimerCallback()
{
  control_count_++;
  auto now_t = this->now();

  // Check flight_status_time
  if (state_.djisdk_valid && (now_t - last_flight_status_ts_).seconds() > param_.max_loss_sdk) {
    RCLCPP_INFO(this->get_logger(), "Flight Status lost for %.2f s, invalid", (now_t - last_flight_status_ts_).seconds());
    state_.djisdk_valid = false;
  }

  state_.vo_latency = (now_t - last_vo_image_ts_).seconds();
  if (state_.vo_valid && state_.vo_latency > param_.max_vo_latency) {
    state_.vo_valid = false;
    RCLCPP_INFO(this->get_logger(), "VO lost for %.2f s, invalid", state_.vo_latency);
  }

  if (state_.rc_valid && (now_t - last_rc_ts_).seconds() > param_.max_loss_rc) {
    state_.rc_valid = false;
    RCLCPP_INFO(this->get_logger(), "RC lost for %.2f s, invalid", (now_t - last_rc_ts_).seconds());
  }

  if (state_.onboard_cmd_valid && (now_t - last_onboard_cmd_ts_).seconds() > param_.max_loss_onboard_cmd) {
    state_.onboard_cmd_valid = false;
    RCLCPP_INFO(this->get_logger(), "ONBOARD cmd lost for %.2f s, invalid", (now_t - last_onboard_cmd_ts_).seconds());
  }

  // Debug print every 10 cycles
  static int dbg_count = 0;
  if (dbg_count++ % 10 == 0) {
    std::printf(
      "P[%.2f, %.2f, %.2f] TGT [%.2f, %.2f, %.2f]\n"
      "ctrl_input_state %d, flight_status %d, ctrl_auth %d, ctrl_mode %d, armed %d, in_air %d, rc_valid %d, onboard_cmd %d, vo_valid %d, sdk_valid %d\n",
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

  if (state_.control_auth != DCMD::CTRL_AUTH_THIS) {
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
bool DroneCommander::callArmService(bool arm)
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

void DroneCommander::tryArm(bool arm)
{
  if (arm == state_.is_armed) {
    return;
  }
  if (fail_arm_times_ > param_.max_try_arm_times) {
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

void DroneCommander::tryControlAuth(bool auth)
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

bool DroneCommander::needControlByThis()
{
  return rcRequestVo() || rcRequestOnboard();
}

bool DroneCommander::checkControlAuth()
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
void DroneCommander::voCallbackImage(const nav_msgs::msg::Odometry & odom)
{
  last_vo_image_ts_ = odom.header.stamp;
}

void DroneCommander::voCallback(const nav_msgs::msg::Odometry & odom)
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

void DroneCommander::rcMavrosCallback(const mavros_msgs::msg::RCIn & rc_in)
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

void DroneCommander::batteryCallback(const sensor_msgs::msg::BatteryState & bat)
{
  state_.bat_vol = bat.voltage;
  double battery_life_tmp = param_.battery_remain_param_a * state_.bat_vol + param_.battery_remain_param_b;
  state_.bat_remain = lowpassFilter(battery_life_tmp, 2.0, state_.bat_remain, 0.1);

  // If battery is too low, force landing
  if (state_.bat_remain <= param_.battery_remain_cutoff &&
      state_.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
    state_.landing_mode = DCMD::LANDING_MODE_XYVEL;
    state_.landing_velocity = param_.landing_vel_z_battery_low;
    requestCtrlMode(DCMD::CTRL_MODE_LANDING);
    processControlLanding();
  }
}

void DroneCommander::fcStateCallback(const mavros_msgs::msg::State & st)
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

void DroneCommander::fcExtendedStateCallback(const mavros_msgs::msg::ExtendedState & est)
{
  if (est.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR) {
    state_.flight_status = DCMD::FLIGHT_STATUS_IN_AIR;
  } else if (est.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND && state_.is_armed) {
    state_.flight_status = DCMD::FLIGHT_STATUS_ARMED;
  }
}

void DroneCommander::onboardCmdCallback(const OCMD & cmd)
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

void DroneCommander::onImuData(const sensor_msgs::msg::Imu & imu)
{
  state_.imu_data = imu;
  last_flight_status_ts_ = this->now();
}

void DroneCommander::onImuDataFused(const sensor_msgs::msg::Imu & imu)
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
void DroneCommander::processInputSource()
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

void DroneCommander::processRcInput()
{
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
    y = - superexpo((rc_.axes[0] - param_.pwm_center)/param_.pwm_100);
    x =   superexpo((rc_.axes[1] - param_.pwm_center)/param_.pwm_100);
    z =   superexpo((rc_.axes[2] - param_.pwm_center)/param_.pwm_100);
    r =   superexpo((rc_.axes[3] - param_.pwm_center)/param_.pwm_100);
  }

  switch (state_.commander_ctrl_mode) {
    case DCMD::CTRL_MODE_POSVEL:
    {
      ctrl_cmd_->yaw_sp = constrainAngle(ctrl_cmd_->yaw_sp + r * param_.rc_max_yaw_rate * param_.loop_duration);
      double vxd = x * param_.rc_max_tilt_vel;
      double vyd = y * param_.rc_max_tilt_vel;

      // transform to ENU => NED or vice versa
      ctrl_cmd_->vel_sp.x =  vxd * std::cos(yaw_vo_) + vyd * std::sin(yaw_vo_);
      ctrl_cmd_->vel_sp.y = -vxd * std::sin(yaw_vo_) + vyd * std::cos(yaw_vo_);
      ctrl_cmd_->vel_sp.z =  z * param_.rc_max_z_vel;

      if (!pos_sp_inited_) {
        ctrl_cmd_->pos_sp.x = odometry_.pose.pose.position.x;
        ctrl_cmd_->pos_sp.y = odometry_.pose.pose.position.y;
        ctrl_cmd_->pos_sp.z = odometry_.pose.pose.position.z;
        pos_sp_inited_ = true;
        resetYawSp();
      }

      if (state_.is_armed && state_.control_auth == DCMD::CTRL_AUTH_THIS) {
        if (state_.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
          ctrl_cmd_->pos_sp.x += ctrl_cmd_->vel_sp.x * param_.loop_duration;
          ctrl_cmd_->pos_sp.y += ctrl_cmd_->vel_sp.y * param_.loop_duration;
          ctrl_cmd_->pos_sp.z += ctrl_cmd_->vel_sp.z * param_.loop_duration;
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
      setAttSetpoint(-y * param_.rc_max_tilt_angle, -x * param_.rc_max_tilt_angle, r * param_.rc_max_yaw_rate, z, true, true, true);
      break;
    }
  }
}

void DroneCommander::processOnboardInput()
{
  if (rcMovingStick()) {
    state_.onboard_cmd_valid = false;
    state_.ctrl_input_state = DCMD::CTRL_INPUT_RC;
    RCLCPP_INFO(this->get_logger(), "Change Source to RC due to RC moving stick");
  }
}

void DroneCommander::processNoneInput()
{
  if (state_.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION &&
      state_.commander_ctrl_mode != DCMD::CTRL_MODE_TAKEOFF &&
      state_.commander_ctrl_mode != DCMD::CTRL_MODE_LANDING) {
    requestCtrlMode(DCMD::CTRL_MODE_HOVER);
  }
}

void DroneCommander::processControl()
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

void DroneCommander::processControlIdle()
{
  setAttSetpoint(0, 0, 0, 0, false);
  sendCtrlCmd();
}

void DroneCommander::processControlPosvel()
{
  bool is_cmd_valid = true;
  if (is_cmd_valid) {
    sendCtrlCmd();
  } else {
    RCLCPP_ERROR(this->get_logger(), "POSVEL control cmd invalid!");
  }
}

void DroneCommander::processControlAtt()
{
  bool is_cmd_valid = true;
  if (is_cmd_valid) {
    sendCtrlCmd();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Attitude control cmd invalid!");
  }
}

void DroneCommander::processControlTakeoff()
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
    if (is_in_air && pos.z > param_.min_takeoff_height) {
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
    if (z_err < param_.max_auto_z_error && 
        std::fabs(pos.x - takeoff_origin_.x()) < param_.max_auto_tilt_error &&
        std::fabs(pos.y - takeoff_origin_.y()) < param_.max_auto_tilt_error) {
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
  if (is_in_air || (pos.z - takeoff_origin_.z() > param_.min_takeoff_height)) {
    ctrl_cmd_->max_vel.z = state_.takeoff_velocity;
    setPosSetpoint(takeoff_origin_.x(),
                    takeoff_origin_.y(),
                    state_.takeoff_target_height + takeoff_origin_.z());
  } else {
    setVelSetpoint(0, 0, state_.takeoff_velocity);
  }
  sendCtrlCmd();
}

void DroneCommander::processControlLanding()
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
      if (state_.pos.z > param_.landing_att_mode_height) {
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

void DroneCommander::processControlMission()
{
  // Not fully implemented
  // ...
  sendCtrlCmd();
}

void DroneCommander::requestCtrlMode(uint32_t req_mode)
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
          state_.landing_velocity = param_.landing_vel_z_emergency;
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

void DroneCommander::processControlMode()
{
  requestCtrlMode(state_.commander_ctrl_mode);
}

// ==================== Send Commands ====================
void DroneCommander::sendControlCmdPX4()
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

void DroneCommander::sendCtrlCmd()
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
bool DroneCommander::setHoverTargetPosition(double x, double y, double z)
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

void DroneCommander::prepareControlHover()
{
  bool fail_to_hover = false;
  if (last_hover_count_ < control_count_ - 1 && state_.is_armed && state_.vo_valid) {
    // Need to start a new hover
    auto vx = odometry_.twist.twist.linear.x;
    auto vy = odometry_.twist.twist.linear.y;
    auto vz = odometry_.twist.twist.linear.z;
    if (std::fabs(vx) > param_.danger_speed_hover ||
        std::fabs(vy) > param_.danger_speed_hover ||
        std::fabs(vz) > param_.danger_speed_hover)
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
      state_.landing_velocity = param_.landing_vel_z_emergency;
      requestCtrlMode(DCMD::CTRL_MODE_LANDING);
    }
  } else {
    last_hover_count_ = control_count_;
  }
}

void DroneCommander::resetCtrlCmd()
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

void DroneCommander::resetCtrlCmdMaxVel()
{
  ctrl_cmd_->max_vel.x = param_.default_max_tilt_vel;
  ctrl_cmd_->max_vel.y = param_.default_max_tilt_vel;
  ctrl_cmd_->max_vel.z = param_.default_max_z_vel;
}

bool DroneCommander::isOdomValid(const nav_msgs::msg::Odometry & odom)
{
  double vx = odom.twist.twist.linear.x;
  double vy = odom.twist.twist.linear.y;
  double vz = odom.twist.twist.linear.z;
  if (std::fabs(vx) > param_.max_odom_velocity ||
      std::fabs(vy) > param_.max_odom_velocity ||
      std::fabs(vz) > param_.max_odom_velocity)
  {
    return false;
  }
  auto now_t = this->now();
  if ((now_t - last_vo_image_ts_).seconds() > param_.max_vo_latency) {
    return false;
  }
  return true;
}

void DroneCommander::resetYawSp()
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
void DroneCommander::setAttSetpoint(double roll, double pitch, double yawrate, double z,
                    bool z_use_vel, bool yaw_use_rate, bool use_fc_yaw)
{
  ctrl_cmd_->use_fc_yaw = use_fc_yaw;
  if (yaw_use_rate) {
    ctrl_cmd_->yaw_sp = constrainAngle(ctrl_cmd_->yaw_sp + yawrate * param_.loop_duration);
  } else {
    ctrl_cmd_->yaw_sp = constrainAngle(yawrate);
  }
  Eigen::Quaterniond quat_sp =
    AngleAxisd(ctrl_cmd_->yaw_sp, Vector3d::UnitZ()) *
    AngleAxisd(pitch, Vector3d::UnitY()) *
    AngleAxisd(roll, Vector3d::UnitX());
  ctrl_cmd_->att_sp.w = quat_sp.w();
  ctrl_cmd_->att_sp.x = quat_sp.x();
  ctrl_cmd_->att_sp.y = quat_sp.y();
  ctrl_cmd_->att_sp.z = quat_sp.z();
  ctrl_cmd_->z_sp = z;
  if (z_use_vel) {
    ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_ATT_VELZ_MODE;
  } else {
    ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_ATT_THRUST_MODE;
  }
}

void DroneCommander::setPosSetpoint(double x, double y, double z,
                    double yaw, double vx_ff, double vy_ff, double vz_ff,
                    double ax_ff, double ay_ff, double az_ff)
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
  ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_POS_MODE;
}

void DroneCommander::setVelSetpoint(double vx, double vy, double vz,
                    double yaw, double ax_ff, double ay_ff, double az_ff)
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
  ctrl_cmd_->ctrl_mode = DPCL::CTRL_CMD_VEL_MODE;
}

/**
 * @brief Handling the OnboardCommand as in original code.
 */
void DroneCommander::handleOnboardCommand(const OCMD & _cmd)
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
      if (h < param_.min_takeoff_height) {
        h = param_.min_takeoff_height;
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

bool DroneCommander::rcRequestOnboard()
{
  // In original code: (rc.axes[6] > 1800 && rc.axes[7] > 1800)
  if (rc_.axes.size() > 7) {
    return (rc_.axes[6] > 1800 && rc_.axes[7] > 1800);
  }
  return false;
}

bool DroneCommander::rcRequestVo()
{
  // In original code: (rc.axes[6] > 1800)
  if (rc_.axes.size() > 6) {
    return (rc_.axes[6] > 1800);
  }
  return false;
}

bool DroneCommander::rcMovingStick()
{
  if (!state_.rc_valid) return false;
  if (rc_.axes.size() < 4) return false;
  bool if_move = std::fabs(rc_.axes[0] - param_.pwm_center) > param_.pwm_deadzone_rpy;
  if_move = if_move || (std::fabs(rc_.axes[1] - param_.pwm_center) > param_.pwm_deadzone_rpy);
  if_move = if_move || (std::fabs(rc_.axes[3] - param_.pwm_center) > param_.pwm_deadzone_rpy);
  if_move = if_move || (std::fabs(rc_.axes[2] - param_.pwm_center) > param_.pwm_deadzone_thr);
  return if_move;
}

// Low pass filter
double DroneCommander::lowpassFilter(double input, double fc, double output_last, double dt)
{
  double RC = 1.0 / (fc * 2.0 * M_PI);
  double alpha = dt / (RC + dt);
  return output_last + alpha * (input - output_last);
}

// Convert FLU -> NED
Eigen::Quaterniond DroneCommander::FLU2NED(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R = R_FLU2FRD_ * q.toRotationMatrix() * R_FLU2FRD_;
  return Eigen::Quaterniond(R);
}

// Convert ENU -> NED
Eigen::Quaterniond DroneCommander::ENU2NED(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R = R_ENU2NED_ * q.toRotationMatrix() * R_FLU2FRD_;
  return Eigen::Quaterniond(R);
}


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
