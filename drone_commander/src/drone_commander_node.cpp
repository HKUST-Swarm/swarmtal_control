#define PX4 0 
#define DJI_SDK 1
#define FCHardware DJI_SDK
// #define FCHardware PX4

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

#if FCHardware == DJI_SDK
#include <dji_sdk/ControlDevice.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#endif

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace swarmtal_msgs;
using namespace Eigen;

// #define MAX_VO_LATENCY 0.5f
#define MAX_LOSS_RC 0.1f
#define MAX_LOSS_SDK 0.1f
#define MAX_ODOM_VELOCITY 25.0f

#define RC_DEADZONE_RPY 0.1
#define RC_DEADZONE_THRUST 0.2

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

#define MAX_LOSS_ONBOARD_CMD 60
#define LANDING_ATT_MODE_HEIGHT 0.1
#define LANDING_ATT_MIN_HEIGHT 0.1

#define LOOP_DURATION 0.02

#define DEBUG_OUTPUT
#define DEBUG_HOVER_CTRL

#define MAGIC_YAW_NAN 666666

#define DCMD drone_commander_state
#define OCMD drone_onboard_command
#define DPCL drone_pos_ctrl_cmd

#define DANGER_SPEED_HOVER (RC_MAX_TILT_VEL+1.5)

#define BATTERY_REMAIN_CUTOFF 240.0f
#define BATTERY_REMAIN_PARAM_A 345.375f 
#define BATTERY_REMAIN_PARAM_B -4757.3f
#define BATTERY_THRUST_A -0.005555555555555558f 
#define BATTERY_THRUST_B 0.18333333333333338f
#define LANDING_VEL_Z_BATTERY_LOW -0.5

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

double expo(const double &value, const double &e)
{
	double x = float_constrain(value, - 1, 1);
	double ec = float_constrain(e, 0, 1);
	return (1 - ec) * x + ec * x * x * x;
}

const double superexpo(const double &value, double e = 0.5, double g = 0.5)
{
	double x = float_constrain(value, - 1, 1);
	double gc = float_constrain(g, 0, 0.99);
	return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
}


inline double constrainAngle(double x){
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

inline Eigen::Vector3d quat2eulers(Eigen::Quaterniond quat);
class DroneCommander {
    ros::NodeHandle & nh;
    drone_commander_state state;

    ros::Subscriber vo_sub;
    ros::Subscriber onboard_cmd_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber flight_status_sub;
    ros::Subscriber ctrl_dev_sub;
    ros::Subscriber fc_att_sub;
    ros::Subscriber bat_sub;
    ros::Subscriber imu_data_sub, vo_sub_slow;

    ros::Timer loop_timer;

    ros::Time last_rc_ts;
    ros::Time last_onboard_cmd_ts;
    ros::Time last_vo_ts;
    ros::Time last_flight_status_ts;
    ros::Time last_try_arm_time;
    ros::Time last_vo_image_ts;

    int fail_arm_times = 0;

    nav_msgs::Odometry odometry;
    sensor_msgs::Joy rc;

    ros::Time boot_time;

    ros::Publisher commander_state_pub;
    ros::Publisher ctrl_cmd_pub;

    drone_pos_ctrl_cmd * ctrl_cmd = nullptr;

    ros::ServiceClient control_auth_client;
    ros::ServiceClient drone_task_control;

    Eigen::Vector3d hover_pos = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d takeoff_origin = Eigen::Vector3d(0, 0, 0);

    bool takeoff_inited = false;
    bool landing_inited = false;

    int control_count = 0;

    int last_hover_count = -1;


    double yaw_fc = 0;
    double yaw_vo = 0;

    float MAX_VO_LATENCY;

    bool yaw_sp_inited = false;

    bool rc_fail_detection = true;

    double battery_life = 600.0f;

    bool in_fc_landing = false;

    bool is_landing_tail = false;
    bool is_touch_ground = false;


    double landing_thrust = 0.035;
    double landing_thrust_min = 0.035, landing_thrust_max = 0.06;

    bool pos_sp_inited = false;

public:
    DroneCommander(ros::NodeHandle & _nh):
        nh(_nh) {
        init_states();
        init_subscribes();


        boot_time = ros::Time::now();

        last_flight_status_ts = ros::Time::now();
        last_rc_ts = ros::Time::now();
        last_vo_ts = ros::Time::now();
        last_onboard_cmd_ts = ros::Time::now();
        last_try_arm_time = ros::Time::now();



        commander_state_pub = nh.advertise<drone_commander_state>("swarm_commander_state", 1);

        ctrl_cmd_pub = nh.advertise<drone_pos_ctrl_cmd>("/drone_position_control/drone_pos_cmd", 1);

       control_auth_client = nh.serviceClient<dji_sdk::SDKControlAuthority>("sdk_control_authority");
        drone_task_control = nh.serviceClient<dji_sdk::DroneTaskControl>("sdk_task_control");
        ROS_INFO("Waitting for services");
       control_auth_client.waitForExistence();
        ROS_INFO("Services ready");
        
        ctrl_cmd = &state.ctrl_cmd;
        
        loop_timer = nh.createTimer(ros::Duration(LOOP_DURATION), &DroneCommander::loop, this);

        nh.param<bool>("rc_fail_detection", rc_fail_detection, true);
        nh.param<double>("landing_thrust_min", landing_thrust_min, 0.0);
        nh.param<double>("landing_thrust_max", landing_thrust_max, 0.0);
        nh.param<double>("max_vo_latency", MAX_VO_LATENCY, 0.4);


        if (rc_fail_detection) {
            ROS_INFO("Will detect RC fail");
        } else {
            ROS_INFO("Will NOT detect RC fail");
        }
        reset_ctrl_cmd_max_vel();
    }

    void init_states() {
        state.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
        state.flight_status = DCMD::FLIGHT_STATUS_IDLE;
        state.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
        state.djisdk_valid = false;
        state.is_armed = false;
        state.rc_valid = false;
        state.onboard_cmd_valid = false;
        state.vo_valid = false;

        state.control_auth = DCMD::CTRL_AUTH_RC;
    }
    void init_subscribes() {
        vo_sub = nh.subscribe("visual_odometry", 1, &DroneCommander::vo_callback, this, ros::TransportHints().tcpNoDelay());
        vo_sub_slow = nh.subscribe("visual_odometry_image", 1, &DroneCommander::vo_callback_image, this, ros::TransportHints().tcpNoDelay());
        onboard_cmd_sub = nh.subscribe("onboard_command", 10, &DroneCommander::onboard_cmd_callback, this, ros::TransportHints().tcpNoDelay());
        flight_status_sub = nh.subscribe("flight_status", 1, &DroneCommander::flight_status_callback, this, ros::TransportHints().tcpNoDelay());
        rc_sub = nh.subscribe("rc", 1, &DroneCommander::rc_callback, this, ros::TransportHints().tcpNoDelay());
        ctrl_dev_sub = nh.subscribe("control_device", 1, &DroneCommander::ctrl_dev_callback, this, ros::TransportHints().tcpNoDelay());
        fc_att_sub = nh.subscribe("fc_attitude", 1, &DroneCommander::fc_attitude_callback, this, ros::TransportHints().tcpNoDelay());
        bat_sub = nh.subscribe("battery", 1, &DroneCommander::battery_callback, this,  ros::TransportHints().tcpNoDelay());
        imu_data_sub = nh.subscribe("fc_imu", 1, &DroneCommander::on_imu_data, this, ros::TransportHints().tcpNoDelay());
    }


    void vo_callback_image(const nav_msgs::Odometry & _odom);
    void vo_callback(const nav_msgs::Odometry & _odom);
    void rc_callback(const sensor_msgs::Joy & _rc);
    void flight_status_callback(const std_msgs::UInt8 & _flight_status);
    void onboard_cmd_callback(const drone_onboard_command & _cmd);
    void ctrl_dev_callback(const dji_sdk::ControlDevice & _ctrl_dev);
    void fc_attitude_callback(const geometry_msgs::QuaternionStamped & _quat);
    void loop(const ros::TimerEvent & _e);
    void battery_callback(const sensor_msgs::BatteryState & _bat);
    void on_imu_data(const sensor_msgs::Imu & _imu);

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

    void set_hover_target_position(double x, double y, double z);

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

    void set_att_setpoint(double roll, double pitch, double yawrate, double z, bool z_use_vel=true, bool yaw_use_rate=true, bool use_fc_yaw = false);
    void set_pos_setpoint(double x, double y, double z, double yaw=NAN, double vx_ff=0, double vy_ff=0, double vz_ff=0, double ax_ff=0, double ay_ff=0, double az_ff=0);
    void set_vel_setpoint(double vx, double vy, double vz, double yaw=NAN, double ax_ff=0, double ay_ff=0, double az_ff=0);

    bool request_drone_landing();
};


void DroneCommander::vo_callback_image(const nav_msgs::Odometry & _odom) {
    last_vo_image_ts = _odom.header.stamp;
}


void DroneCommander::loop(const ros::TimerEvent & _e) {
    static int count = 0; 
    control_count ++;
    if (state.djisdk_valid && (ros::Time::now() - last_flight_status_ts).toSec() > MAX_LOSS_SDK) {
        ROS_INFO("Flight Status loss time %3.2f, is invalid", (ros::Time::now() - last_flight_status_ts).toSec());        
        state.djisdk_valid = false;
    }

    state.vo_latency = (ros::Time::now() - last_vo_image_ts).toSec();

    if (state.vo_valid && (ros::Time::now() - last_vo_image_ts).toSec() > MAX_VO_LATENCY) {
        state.vo_valid = false;
        ROS_INFO("VO loss time %3.2f, is invalid", (ros::Time::now() - last_vo_image_ts).toSec());
    }

    if (state.rc_valid && (ros::Time::now() - last_rc_ts).toSec() > MAX_LOSS_RC ) {
        state.rc_valid = false;
        ROS_INFO("RC loss time %3.2f, is invalid", (ros::Time::now() - last_rc_ts).toSec());
    }

    if (state.onboard_cmd_valid&& (ros::Time::now() - last_onboard_cmd_ts).toSec() > MAX_LOSS_ONBOARD_CMD ) {
        state.onboard_cmd_valid = false;
        ROS_INFO("ONBOARD loss time %3.2f, is invalid", (ros::Time::now() - last_onboard_cmd_ts).toSec());
    }


    if (count ++ % 10 == 0)
    {
#ifdef DEBUG_OUTPUT
        if (rc.axes.size() >= 6)
        ROS_INFO("RC valid %d %3.2f %3.2f %3.2f %3.2f %4.0f %4.0f",
            state.rc_valid,
            rc.axes[0],
            rc.axes[1],
            rc.axes[2],
            rc.axes[3],
            rc.axes[4],
            rc.axes[5]
        );

        ROS_INFO("POS     %3.2f      %3.2f     %3.2f TGT %3.2f %3.2f %3.2f\nctrl_input_state %d, flight_status %d\nstate.control_auth %d  ctrl_mode %d, is_armed %d\n rc_valid %d onboard_cmd_valid %d vo_valid%d sdk_valid %d ",
    	    odometry.pose.pose.position.x,
	        odometry.pose.pose.position.y,
	        odometry.pose.pose.position.z,
            ctrl_cmd->pos_sp.x,
            ctrl_cmd->pos_sp.y,
            ctrl_cmd->pos_sp.z,
            state.ctrl_input_state,
            state.flight_status,
            state.control_auth,
            state.commander_ctrl_mode,
            state.is_armed,
            state.rc_valid,
            state.onboard_cmd_valid,
            state.vo_valid,
            state.djisdk_valid
        );
#endif
    }

    if (!state.djisdk_valid) {
        return;
    }
    
    if (!yaw_sp_inited) {
        reset_yaw_sp();
    }

    if (!(state.control_auth == DCMD::CTRL_AUTH_THIS))
        reset_ctrl_cmd();
    
    process_input_source();

    if (check_control_auth()){
        process_control_mode();
        process_control();
    } else {
        reset_yaw_sp();
        last_hover_count = -1;
    }

    commander_state_pub.publish(state);
}


void DroneCommander::try_arm(bool arm) {
    dji_sdk::DroneArmControl arm_srv;
    if (arm==state.is_armed )
        return;
    // if ((ros::Time::now() - last_try_arm_time).toSec() < MIN_TRY_ARM_DURATION) {
    //     ROS_INFO("Will try arm again later");
    //     return;
    // }
    if (fail_arm_times > MAX_TRY_ARM_TIMES) {
        ROS_INFO("Fail arm too much times, give up dear, Request IDLE!");
        request_ctrl_mode(DCMD::CTRL_MODE_IDLE);

        return;
    }
    if (!arm) {
        request_ctrl_mode(DCMD::CTRL_MODE_IDLE);
    }
    if (state.djisdk_valid && state.flight_status == DCMD::FLIGHT_STATUS_IDLE && arm) {
        // TODO:
        // rosservice call /dji_sdk_1/dji_sdk/drone_arm_control "arm: 0"
        arm_srv.request.arm = arm;
        ros::service::call("/dji_sdk_1/dji_sdk/drone_arm_control", arm_srv);
        ROS_INFO("Try arm success %d", arm_srv.response.result);
        // if *
        if (!arm_srv.response.result) {
            fail_arm_times ++;
        }
    }
    if (state.djisdk_valid && ! arm) {
        arm_srv.request.arm = arm;
        ros::service::call("/dji_sdk_1/dji_sdk/drone_arm_control", arm_srv);
        ROS_INFO("Try DIsarm success %d", arm_srv.response.result);
        // if *
        if (!arm_srv.response.result) {
            fail_arm_times ++;
        }
    }
    last_try_arm_time = ros::Time::now();
}

void DroneCommander::try_control_auth(bool auth) {
    dji_sdk::SDKControlAuthority srv;
    srv.request.control_enable = auth;
    if (control_auth_client.call(srv))
    {
        ROS_INFO("Require control auth %d, res %d", auth, srv.response.ack_data);
        state.control_auth = srv.response.ack_data;
    } else {
        ROS_ERROR("Failed to call service control auth");
    }
}

bool DroneCommander::check_control_auth() {
    bool require_auth_this = true;
    if (!state.djisdk_valid)
        return false;
    if (state.rc_valid) {
        if (this->rc_request_vo()){
            require_auth_this = true;
        } else {
            require_auth_this = false;
        }
    }

    //If rc still not available, will try to grab auth
    if (!state.rc_valid) {
        require_auth_this = true;
    }

    if (require_auth_this) {
        if (state.control_auth == DCMD::CTRL_AUTH_RC || 
            state.control_auth == DCMD::CTRL_AUTH_APP) {
            ROS_INFO("Require AUTH");
        }
    }
    else {
        if (state.control_auth == DCMD::CTRL_AUTH_THIS){
            ROS_INFO("Relase AUTH");
        }
    }

    if ((require_auth_this && state.control_auth != DCMD::CTRL_AUTH_THIS) ||
        (!require_auth_this && state.control_auth == DCMD::CTRL_AUTH_THIS)
        )
        try_control_auth(require_auth_this);

    return state.control_auth == DCMD::CTRL_AUTH_THIS;
}


void DroneCommander::vo_callback(const nav_msgs::Odometry & _odom) {
    bool vo_valid = is_odom_valid(_odom);
    //printf("VO valid %d", vo_valid);
    auto pose = _odom.pose.pose;
    Eigen::Quaterniond quat(pose.orientation.w, 
        pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d rpy = quat2eulers(quat);
    yaw_vo = - rpy.z();
    if (!state.vo_valid && vo_valid) {
        //Vo first time come
        //reset yaw sp use vo yaw
        reset_yaw_sp();
    }
    state.vo_valid = vo_valid;
    if (state.vo_valid) {
        odometry = _odom;
        //FIX vo lost due to time align
        last_vo_ts = ros::Time::now();
    }

    state.pos.x = pose.position.x;
    state.pos.y = pose.position.y;
    state.pos.z = pose.position.z;

    state.vel.x = _odom.twist.twist.linear.x;
    state.vel.y = _odom.twist.twist.linear.y;
    state.vel.z = _odom.twist.twist.linear.z;
    state.yaw = yaw_vo;
}

inline double lowpass_filter(double input, double fc, double outputlast, double dt) {
	double RC = 1.0 / (fc *2 * M_PI);
	double alpha = dt / (RC + dt);
	return outputlast + (alpha* (input - outputlast));
}

void DroneCommander::battery_callback(const sensor_msgs::BatteryState &_bat) {
    state.bat_vol = _bat.voltage;

    double battery_life_tmp = BATTERY_REMAIN_PARAM_A * state.bat_vol + BATTERY_REMAIN_PARAM_B;

    // if ((battery_life - battery_life_tmp) > 60)
    //     battery_life = battery_life - 60;
    // else if ((battery_life - battery_life_tmp) < -60)
    //     battery_life = battery_life + 60;
    // else
    //     battery_life = BATTERY_REMAIN_PARAM_A* state.bat_vol + BATTERY_REMAIN_PARAM_B;

    state.bat_remain = lowpass_filter(battery_life_tmp, 2 , state.bat_remain, 0.1);

    //ROS_INFO("Battery Level: %3.2f, Left Time: %3.2f", state.bat_vol, state.bat_remain);

    if (state.bat_remain <= BATTERY_REMAIN_CUTOFF &&
        state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
        //ROS_INFO("Battery Low, Landing");
        state.landing_mode = DCMD::LANDING_MODE_XYVEL;
        state.landing_velocity = LANDING_VEL_Z_BATTERY_LOW;
        request_ctrl_mode(DCMD::CTRL_MODE_LANDING);
        process_control_landing();
    }
    else  {
        //ROS_INFO("Battery Low");
    }
}

void DroneCommander::rc_callback(const sensor_msgs::Joy & _rc) {
    state.rc_valid = is_rc_valid(_rc);
    
    if (state.rc_valid) {
        rc = _rc;
        last_rc_ts = ros::Time::now();
    }

    state.djisdk_valid = true;
}

void DroneCommander::flight_status_callback(const std_msgs::UInt8 & _flight_status) {
    //TODO:
    uint8_t _status = _flight_status.data;
    if (_status == 0) {
        //IS on land not arm
        state.flight_status = DCMD::FLIGHT_STATUS_IDLE;
        state.is_armed = false;
    }

    if (_status == 1) {
        // Onland armed
        state.flight_status = DCMD::FLIGHT_STATUS_ARMED;
        state.is_armed = true;
    }

    if (_status == 2) {
        //In air
        state.flight_status = DCMD::FLIGHT_STATUS_IN_AIR;
        state.is_armed = true;
    }

    state.djisdk_valid = true;
    last_flight_status_ts = ros::Time::now();
}

void DroneCommander::fc_attitude_callback(const geometry_msgs::QuaternionStamped & _quat) {
    geometry_msgs::Quaternion quat = _quat.quaternion;


    Eigen::Matrix3d R_FLU2FRD; 
    R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    Eigen::Matrix3d R_ENU2NED;
    R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

    Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);

    Eigen::Matrix3d RFLU2ENU = q.toRotationMatrix();

    q = Eigen::Quaterniond(R_ENU2NED*RFLU2ENU*R_FLU2FRD.transpose());

    Eigen::Vector3d rpy = quat2eulers(q);

    //Original rpy is ENU, we need NED rpy
    yaw_fc = rpy.z();

}

void DroneCommander::on_imu_data(const sensor_msgs::Imu & _imu) {

    state.imu_data = _imu;
    // ROS_INFO("Imu data acc z: %f", state.imu_data.linear_acceleration.z);
    return;
    // Eigen::Vector3d acc(
    //     state.imu_data.linear_acceleration.x,
    //     state.imu_data.linear_acceleration.y,
    //     state.imu_data.linear_acceleration.z
    // );
}

void DroneCommander::set_att_setpoint(double roll, double pitch, double yaw, double z, bool z_use_vel, bool yaw_use_rate, bool use_fc_yaw) {

    ctrl_cmd->use_fc_yaw = use_fc_yaw;
    if (yaw_use_rate) {
        yaw = ctrl_cmd->yaw_sp = constrainAngle(ctrl_cmd->yaw_sp + yaw * LOOP_DURATION);
    } else {
        ctrl_cmd->yaw_sp = constrainAngle(yaw);
    }

    Quaterniond quat_sp = AngleAxisd(yaw, Vector3d::UnitZ()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(roll, Vector3d::UnitX());
    
    ctrl_cmd->att_sp.w = quat_sp.w();
    ctrl_cmd->att_sp.x = quat_sp.x();
    ctrl_cmd->att_sp.y = quat_sp.y();
    ctrl_cmd->att_sp.z = quat_sp.z();
    ctrl_cmd->z_sp = z;

    if (state.is_armed && state.control_auth == DCMD::CTRL_AUTH_THIS) {
        if (z_use_vel) {
            ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_ATT_VELZ_MODE;
        } else {
            ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_ATT_THRUST_MODE;
        }
    } else {
        ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    }
}

void DroneCommander::set_pos_setpoint(double x, double y, double z, double yaw, double vx_ff, double vy_ff, double vz_ff, double ax_ff, double ay_ff, double az_ff) {
    ctrl_cmd->pos_sp.x = x;
    ctrl_cmd->pos_sp.y = y;
    ctrl_cmd->pos_sp.z = z;
    ctrl_cmd->vel_sp.x = vx_ff;
    ctrl_cmd->vel_sp.y = vy_ff;
    ctrl_cmd->vel_sp.z = vz_ff;
    ctrl_cmd->acc_sp.x = ax_ff;
    ctrl_cmd->acc_sp.y = ay_ff;
    ctrl_cmd->acc_sp.z = az_ff;
    ctrl_cmd->use_fc_yaw = false;
    if (!std::isnan(yaw)) {
        ctrl_cmd->yaw_sp = constrainAngle(yaw);
    }

    if (state.is_armed && state.control_auth == DCMD::CTRL_AUTH_THIS) {
        ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_POS_MODE;
    } else {
        ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    }
}

void DroneCommander::set_vel_setpoint(double vx, double vy, double vz, double yaw, double ax_ff, double ay_ff, double az_ff) {
    ctrl_cmd->vel_sp.x = vx;
    ctrl_cmd->vel_sp.y = vy;
    ctrl_cmd->vel_sp.z = vz;
    ctrl_cmd->acc_sp.x = ax_ff;
    ctrl_cmd->acc_sp.y = ay_ff;
    ctrl_cmd->acc_sp.z = az_ff;
    if (!std::isnan(yaw)) {
        ctrl_cmd->yaw_sp = constrainAngle(yaw);
    }

    ctrl_cmd->use_fc_yaw = false;
    
    if (state.is_armed && state.control_auth == DCMD::CTRL_AUTH_THIS) {
        ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_VEL_MODE;
    } else {
        ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    }
}

void DroneCommander::onboard_cmd_callback(const drone_onboard_command & _cmd) {

    state.onboard_cmd_valid = true;
    last_onboard_cmd_ts = ros::Time::now();
    if (state.ctrl_input_state != DCMD::CTRL_INPUT_ONBOARD) {
        process_input_source();
    }

    if (state.ctrl_input_state == DCMD::CTRL_INPUT_ONBOARD) {
        switch (_cmd.command_type) {
            case OCMD::CTRL_POS_COMMAND: {
                request_ctrl_mode(DCMD::CTRL_MODE_POSVEL);
                double x = ((double)_cmd.param1) / 10000;
                double y = ((double)_cmd.param2) / 10000;
                double z = ((double)_cmd.param3) / 10000;
                double yaw = ((double) _cmd.param4) / 10000;
                double vx_ff = ((double)_cmd.param5) / 10000;
                double vy_ff = ((double)_cmd.param6) / 10000;
                double vz_ff = ((double)_cmd.param7) / 10000;
                double ax_ff = ((double)_cmd.param8) / 10000;
                double ay_ff = ((double)_cmd.param9) / 10000;
                double az_ff = ((double)_cmd.param10) / 10000;

                // ROS_INFO("Recv pos cmd, fly to %3.2 %3.2f %3.2f", x, y, z);
                if (_cmd.param4 == MAGIC_YAW_NAN) {
                    set_pos_setpoint(x, y, z, NAN, vx_ff, vy_ff, vz_ff, ax_ff, ay_ff, az_ff);
                } else {
                    set_pos_setpoint(x, y, z, yaw, vx_ff, vy_ff, vz_ff, ax_ff, ay_ff, az_ff);
                }

                break;
            }

            case OCMD::CTRL_VEL_COMMAND: {
                request_ctrl_mode(DCMD::CTRL_MODE_POSVEL);
                double x = ((double)_cmd.param1) / 10000;
                double y = ((double)_cmd.param2) / 10000;
                double z = ((double)_cmd.param3) / 10000;
                double yaw = ((double) _cmd.param4) / 10000;

                double ax_ff = ((double)_cmd.param5) / 10000;
                double ay_ff = ((double)_cmd.param6) / 10000;
                double az_ff = ((double)_cmd.param7) / 10000;

                if (_cmd.param4 == MAGIC_YAW_NAN) {
                    set_vel_setpoint(x, y, z, ax_ff, ay_ff, az_ff);
                }
                break;
            }
            case OCMD::CTRL_ATT_COMMAND: {
                request_ctrl_mode(DCMD::CTRL_MODE_ATT);

                double roll = ((double)_cmd.param1) / 10000;
                double pitch = ((double)_cmd.param2) / 10000;
                double yaw_rate = ((double)_cmd.param3) / 10000;
                double z = ((double)_cmd.param4) / 10000;
                set_att_setpoint(roll, pitch, yaw_rate, z, _cmd.param5 == 0, _cmd.param6 == 0);
                break;
            }

            case OCMD::CTRL_MISSION_LOAD_COMMAND: {
                
                request_ctrl_mode(DCMD::CTRL_MODE_MISSION);
                break;
            }

            case OCMD::CTRL_MISSION_END_COMMAND: {
                request_ctrl_mode(DCMD::CTRL_MODE_HOVER);
                break;
            }

            case OCMD::CTRL_TAKEOF_COMMAND: {
                // if (state.)
                fail_arm_times = 0;
                double h = ((double)_cmd.param1) / 10000;
                if (h < MIN_TAKEOFF_HEIGHT) {
                    h = MIN_TAKEOFF_HEIGHT;
                }
                ROS_INFO("Onboard trying to takeoff, will hover at %3.2f", h);

                request_ctrl_mode(DCMD::CTRL_MODE_TAKEOFF);
                state.takeoff_target_height = h;
                state.takeoff_velocity = ((double)_cmd.param2) / 10000.0;
                break;
            };

            case OCMD::CTRL_LANDING_COMMAND: {
                ROS_INFO("Onboard trying to Landing");
                if (_cmd.param1 < 0) {
                    state.landing_mode = DCMD::LANDING_MODE_ATT;
                    is_landing_tail = true;
                }
                else if (_cmd.param1 == 1) {
                    state.landing_mode = DCMD::LANDING_MODE_ATT;
                } else {
                    state.landing_mode = DCMD::LANDING_MODE_XYVEL;
                    is_landing_tail = false;
                    is_touch_ground = false;
                }

                state.landing_velocity = -((double)_cmd.param2) / 10000.0;

                request_ctrl_mode(DCMD::CTRL_MODE_LANDING);
                break;
            }


            case OCMD::CTRL_HOVER_COMMAND: {
                request_ctrl_mode(DCMD::CTRL_MODE_HOVER);
                break;
            }

            case OCMD::CTRL_ARM_COMMAND: {
                fail_arm_times = 0;
                ROS_INFO("Onboard command arm %d", _cmd.param1);
                try_arm(_cmd.param1 > 0);
                break;
            }
        }
    }
}


bool DroneCommander::rc_request_onboard() {
    return (rc.axes[4] > 8000 && rc.axes[5] < -8000);
}

bool DroneCommander::rc_request_vo() {
    // return (rc.axes[4] == 10000 && rc.axes[5] == -10000);
    return (rc.axes[4] > 8000);
}

bool DroneCommander::rc_moving_stick () {
    bool if_move =  fabs(rc.axes[0]) > RC_DEADZONE_RPY;
    if_move = if_move || fabs(rc.axes[1]) > RC_DEADZONE_RPY;
    if_move = if_move || fabs(rc.axes[2]) > RC_DEADZONE_RPY;
    if_move = if_move || fabs(rc.axes[3]) > RC_DEADZONE_THRUST;

    return if_move;
}

void DroneCommander::process_input_source () {
    if (state.ctrl_input_state == DCMD::CTRL_INPUT_NONE) {
        if (state.rc_valid) {
            state.ctrl_input_state = DCMD::CTRL_INPUT_RC;
            ROS_INFO("Change Source to RC");
        } else if (state.onboard_cmd_valid){
            state.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;
            ROS_INFO("Change Source to onboard because no RC and onboard vaild");
        }
    }

    if (state.ctrl_input_state == DCMD::CTRL_INPUT_RC) {
        if (!state.rc_valid){
            state.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
            ROS_INFO("Change Source to None because RC Failure");
            if (state.onboard_cmd_valid) {
                state.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;
                ROS_INFO("Change Source to CMD because RC Failure and cmd vaild");
            }
        } else if (this->rc_request_onboard() && state.onboard_cmd_valid) {
            state.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;
            ROS_INFO("Change Source to onboard because ctrl require onboard");
        }
    }

    if (state.ctrl_input_state == DCMD::CTRL_INPUT_ONBOARD) {
        if (!state.onboard_cmd_valid)
        {
            if (state.rc_valid) {
                state.ctrl_input_state = DCMD::CTRL_INPUT_RC;
                ROS_INFO("Onboard invaild. Change Source to RC");
            } else {
                state.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
                ROS_INFO("Onboard invail. Change Source to None");
            }
        }
    }

    // ROS_INFO("In state %d", state.ctrl_input_state);

    switch (state.ctrl_input_state) {
        case DCMD::CTRL_INPUT_RC:
            process_rc_input();
            break;
        case DCMD::CTRL_INPUT_ONBOARD:
            process_onboard_input();
            break;
        default:
        case DCMD::CTRL_INPUT_NONE:
            process_none_input();
            break;
    }
    // ROS_INFO("In2 state %d", state.ctrl_input_state);

}


void DroneCommander::process_rc_input () {
    if (state.control_auth != DCMD::CTRL_AUTH_THIS) {
        state.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
        return;
    }
    
    //Force RC control velocity
    if (rc_moving_stick()) {
        request_ctrl_mode(DCMD::CTRL_MODE_POSVEL);
    } else {
        if (state.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION && 
            state.commander_ctrl_mode != DCMD::CTRL_MODE_TAKEOFF &&
            state.commander_ctrl_mode != DCMD::CTRL_MODE_LANDING
            ) {
            // ROS_INFO("Stick not moving, using hOver mode");
            //When no input and not takeoff and not landing, turn to hover
            request_ctrl_mode(DCMD::CTRL_MODE_HOVER);
        } else {
            // ROS_INFO("Waiting for");
        }
    }

    //TODO: Generate command using rc
    double y = 0;
    double x = 0;
    double r = 0;
    double z = 0;

    if (state.rc_valid) {
        y = - superexpo(rc.axes[0]) ;
        x = superexpo(rc.axes[1]);
        r = superexpo(rc.axes[2]);
        z = superexpo(rc.axes[3]);
    }


    switch (state.commander_ctrl_mode) {
        case DCMD::CTRL_MODE_POSVEL: {
            ctrl_cmd->yaw_sp =  constrainAngle(ctrl_cmd->yaw_sp + r * RC_MAX_YAW_RATE * LOOP_DURATION);
            double vxd = x * RC_MAX_TILT_VEL;
            double vyd = y * RC_MAX_TILT_VEL;

            ctrl_cmd->vel_sp.x = vxd * cos(yaw_vo) + vyd*sin(yaw_vo);
            ctrl_cmd->vel_sp.y = -vxd * sin(yaw_vo) + vyd*cos(yaw_vo);
            ctrl_cmd->vel_sp.z = z * RC_MAX_Z_VEL;

            if (!pos_sp_inited) {
                ctrl_cmd->pos_sp.x = odometry.pose.pose.position.x;
                ctrl_cmd->pos_sp.y = odometry.pose.pose.position.y;
                ctrl_cmd->pos_sp.z = odometry.pose.pose.position.z;
                pos_sp_inited = true;
            }

            if (state.is_armed && state.control_auth == DCMD::CTRL_AUTH_THIS) { 
                if(state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
                    ctrl_cmd->pos_sp.x = ctrl_cmd->pos_sp.x + ctrl_cmd->vel_sp.x * LOOP_DURATION;
                    ctrl_cmd->pos_sp.y = ctrl_cmd->pos_sp.y + ctrl_cmd->vel_sp.y * LOOP_DURATION;
                    ctrl_cmd->pos_sp.z = ctrl_cmd->pos_sp.z + ctrl_cmd->vel_sp.z * LOOP_DURATION;
                    ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_POS_MODE;
                } else {
                    ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_VEL_MODE;
                    pos_sp_inited = false;
                }
            } else {
                ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
                pos_sp_inited = false;
            }


            break;
        }

        case DCMD::CTRL_MODE_TAKEOFF: 
        case DCMD::CTRL_MODE_LANDING:
        case DCMD::CTRL_MODE_MISSION:
            break;        
        case DCMD::CTRL_MODE_HOVER:
            prepare_control_hover();
            break;

        case DCMD::CTRL_MODE_IDLE:        
        case DCMD::CTRL_MODE_ATT:
        default: {
            reset_yaw_sp();
            set_att_setpoint(-y* RC_MAX_TILT_ANGLE, -x * RC_MAX_TILT_ANGLE,  r * RC_MAX_YAW_RATE, z, true, true, true);
            break;
        }
    }


}

void DroneCommander::process_none_input () {
    if (state.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION &&
        state.commander_ctrl_mode != DCMD::CTRL_MODE_TAKEOFF &&
        state.commander_ctrl_mode != DCMD::CTRL_MODE_LANDING) {
        //When no input and not takeoff and not landing, turn to hover
        request_ctrl_mode(DCMD::CTRL_MODE_HOVER);
    }
}

void DroneCommander::process_control_idle() {
    //Landing and disarm
    /*
    if (state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
        request_ctrl_mode(DCMD::CTRL_MODE_LANDING);
        process_control_landing();
    } else {
        if (state.is_armed) {
            this->try_arm(false);
        }
    }*/
}

void DroneCommander::process_onboard_input () {
    if (rc_moving_stick()) {
        state.onboard_cmd_valid = false;
        state.ctrl_input_state = DCMD::CTRL_INPUT_RC;
        ROS_INFO("Change Source to RC Due to RC moving stick");
    }
}

void DroneCommander::process_control() {
    //control_count ++;

    if (state.control_auth != DCMD::CTRL_AUTH_THIS) {
        state.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
        return;
    }

    
    switch (state.commander_ctrl_mode) {
        case DCMD::CTRL_MODE_HOVER:
            prepare_control_hover();
            process_control_posvel();
            break;
        case DCMD::CTRL_MODE_POSVEL:
            process_control_posvel();
            break;
        case DCMD::CTRL_MODE_ATT:
            process_control_att();
            break;
        case DCMD::CTRL_MODE_TAKEOFF:
            process_control_takeoff();
            break;
        case DCMD::CTRL_MODE_LANDING:
            process_control_landing();
            break;
        case DCMD::CTRL_MODE_MISSION:
            process_control_mission();
            break;
        
        case DCMD::CTRL_MODE_IDLE:
        default:
            process_control_idle();
            break;
    }
}
void DroneCommander::process_control_posvel () {
    // Check command first
    bool is_cmd_valid = true;
    if (is_cmd_valid)
    {
        send_ctrl_cmd();
    } else {
        ROS_ERROR("POSVEL Ctrl cmd invaild!");
        //TODO:
        // ctrl_cmd_pub.publish(*ctrl_cmd);
    }

}

void DroneCommander::process_control_att() {
    bool is_cmd_valid = true;
    if (is_cmd_valid)
    {
        ctrl_cmd_pub.publish(*ctrl_cmd);
    } else {
        ROS_ERROR("Att ctrl cmd invaild!");
        //TODO:
    }
}

void DroneCommander::process_control_takeoff() {
    //TODO: write takeoff scirpt
    bool is_in_air = state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR;
    bool is_takeoff_finish = false;
    auto pos = odometry.pose.pose.position;

    if (state.control_auth != DCMD::CTRL_AUTH_THIS) {
        //Abort takeoff
        takeoff_inited = false;
        request_ctrl_mode(DCMD::CTRL_MODE_IDLE);
        return;
    }

    if (!state.vo_valid) {
        takeoff_inited = false;
        request_ctrl_mode(DCMD::CTRL_MODE_LANDING);
        return;
    }

    if (!takeoff_inited) {
        if (state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR && pos.z > MIN_TAKEOFF_HEIGHT) {
            ROS_INFO("Already in air");
            is_takeoff_finish = true;
        }
        takeoff_inited = true;
        if (state.vo_valid) {
            takeoff_origin.x() = odometry.pose.pose.position.x;
            takeoff_origin.y() = odometry.pose.pose.position.y;
            takeoff_origin.z() = odometry.pose.pose.position.z;
            ROS_INFO("Initing takeoff, origin place is %3.2lf %3.2lf %3.2lf", takeoff_origin.x(), takeoff_origin.y(), takeoff_origin.z());
        } else {
            ROS_INFO("Initing takeoff, no vo");
        }
    }

    if (!state.is_armed) {
        ROS_INFO("Trying to takeoff but not armed. Try arm");
        try_arm(true);
    }

    if (state.vo_valid) {
        if (fabs(pos.x - takeoff_origin.x()) < MAX_AUTO_TILT_ERROR && 
            fabs(pos.y - takeoff_origin.y()) < MAX_AUTO_TILT_ERROR && 
            fabs(pos.z - (takeoff_origin.z() + state.takeoff_target_height)) < MAX_AUTO_Z_ERROR) {
            is_takeoff_finish = true;
            ROS_INFO("Takeoff finish");
        } else {
            // ROS_INFO("Takeoff error %3.2f %3.2f %3.2f", fabs(pos.x - takeoff_origin.x()), 
                // fabs(pos.y - takeoff_origin.y()), 
                // fabs(pos.z - (takeoff_origin.z() + state.takeoff_target_height)) < MAX_AUTO_Z_ERROR);
            is_takeoff_finish = false;
        }
    } else {
        is_takeoff_finish = is_in_air;
    }

    if (is_takeoff_finish) {
        ROS_INFO("Finish takeoff, turn to hover....");
        request_ctrl_mode(DCMD::CTRL_MODE_HOVER);
        if (state.commander_ctrl_mode == DCMD::CTRL_MODE_HOVER) {
            ROS_INFO("Takeof trans to hover mode, will hover at %3.2f %3.2f %3.2f....", takeoff_origin.x(), takeoff_origin.y(), state.takeoff_target_height + takeoff_origin.z());
            set_hover_target_position(
                takeoff_origin.x(), takeoff_origin.y(), state.takeoff_target_height + takeoff_origin.z()
            );
        }

        takeoff_inited = false;
        reset_ctrl_cmd_max_vel();
        return;
    }

    if (is_in_air && state.vo_valid) {
        //Already in air, process as a  posvel control
        ctrl_cmd->max_vel.z = state.takeoff_velocity;
        set_pos_setpoint(takeoff_origin.x(), takeoff_origin.y(), state.takeoff_target_height + takeoff_origin.z());
        
        // ROS_INFO("Already in air, fly to %3.2lf %3.2lf %3.2lf", ctrl_cmd->pos_sp.x, ctrl_cmd->pos_sp.y, ctrl_cmd->pos_sp.z);
    } else {
        if (state.vo_valid) {
            set_att_setpoint(0, 0, yaw_vo, state.takeoff_velocity, true, false);            
        }
    }

    send_ctrl_cmd();
}

bool DroneCommander::request_drone_landing() {
    dji_sdk::DroneTaskControl srv;
    srv.request.task = dji_sdk::DroneTaskControlRequest::TASK_LAND;
    if (drone_task_control.call(srv)) {
        if (srv.response.result) {
            ROS_INFO("Using DJI Landing success....");
            // request_ctrl_mode(DCMD::CTRL_MODE_IDLE);
            in_fc_landing = true;
            return true;
        }
    }
    return false;
}

void DroneCommander::process_control_landing() {
    //TODO: write better landing
    bool is_landing_finish = state.flight_status < DCMD::FLIGHT_STATUS_IN_AIR;

    // if acc.z > 12 (9.8 + 2.) stop

    if (is_landing_finish) {
        ROS_INFO("Landing finish; try disarm...");
        is_landing_tail = false;
        is_touch_ground = true;

        this->try_arm(false);
        if (!state.is_armed) {
            request_ctrl_mode(DCMD::CTRL_MODE_IDLE);
            ROS_INFO("Finsh landing, turn to IDLE");
        }
        return;
    }

    if (state.imu_data.linear_acceleration.z > 15.0) {
        ROS_INFO("Detect touching ground, is_touch_ground to TRUE");
        is_touch_ground = true;
    }

    if (is_landing_tail) {
        // ROS_INFO("Is landing tail....");
        if (is_touch_ground) {
            ROS_INFO("Touch ground, thrust set to zero");
            //Actually thrust protection will only keep thrust at a low value but not 0. 0.0 is just for convenience
            set_att_setpoint(0, 0, yaw_vo, 0.0, false, false);
        }
        else {
            set_att_setpoint(0, 0, yaw_vo, landing_thrust, false, false);
        }
        send_ctrl_cmd();
    } else {
        if (state.vo_valid && state.landing_mode == DCMD::LANDING_MODE_XYVEL) {
            if (state.pos.z > LANDING_ATT_MODE_HEIGHT) {
                set_vel_setpoint(0, 0, state.landing_velocity);
            } else {
                landing_thrust = state.bat_vol * BATTERY_THRUST_A + BATTERY_THRUST_B;
                landing_thrust = float_constrain(landing_thrust, landing_thrust_min, landing_thrust_max);
                ROS_INFO("battery is: %f", state.bat_vol);
                ROS_INFO("raw thrust is: %f", state.bat_vol * BATTERY_THRUST_A + BATTERY_THRUST_B);
                ROS_INFO("landing_thrust min is: %f", landing_thrust_min);
                ROS_INFO("landing_thrust max is: %f", landing_thrust_max);
                printf("landing_thrust is: %f", landing_thrust);
                is_landing_tail = true;
                //set_att_setpoint(0, 0, yaw_vo, LANDING_VEL_Z_EMERGENCY, true, false);

                /*
                bool res = request_drone_landing();
                if (!res) {
                    state.landing_velocity = LANDING_VEL_Z_EMERGENCY;
                    ROS_ERROR("Can't landing with dji; emergency landing instead");
                    set_att_setpoint(0, 0, yaw_vo, LANDING_VEL_Z_EMERGENCY, true, false);
                    state.landing_mode = DCMD::LANDING_MODE_ATT;
                }*/
            }

        } else {
            set_att_setpoint(0, 0, yaw_vo, state.landing_velocity, true, false);
        }
        send_ctrl_cmd();
    }


}

void DroneCommander::request_ctrl_mode(uint32_t req_ctrl_mode) {
    // ROS_INFO("Request %d", req_ctrl_mode);
    switch (req_ctrl_mode) {
        case DCMD::CTRL_MODE_LANDING: {
            if (state.flight_status < state.FLIGHT_STATUS_ARMED) {
                state.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
            } else {
                state.commander_ctrl_mode = req_ctrl_mode;
            }
            return;
            break;
        }

        case DCMD::CTRL_MODE_TAKEOFF: {
            if (state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR && state.commander_ctrl_mode != DCMD::CTRL_MODE_TAKEOFF && state.commander_ctrl_mode != DCMD::CTRL_MODE_LANDING) {
                ROS_INFO("Directly hover in take-off");
                request_ctrl_mode(DCMD::CTRL_MODE_HOVER);       
            } else {
                state.commander_ctrl_mode = req_ctrl_mode;
            }
            break;
        }
        case DCMD::CTRL_MODE_MISSION:
        case DCMD::CTRL_MODE_HOVER:
        case DCMD::CTRL_MODE_POSVEL:{
            if (state.flight_status < state.FLIGHT_STATUS_IN_AIR) {
                //Not in air; goto IDLE
                state.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
            }

            if (state.vo_valid) {
                state.commander_ctrl_mode = req_ctrl_mode;
            } else {
                ROS_WARN("VO failed on MISSION/HOVER/POSVEL Mode. Will emergency landing now");
                state.commander_ctrl_mode = DCMD::CTRL_MODE_LANDING;
                state.landing_velocity = LANDING_VEL_Z_EMERGENCY;
                return;
            }
            break;
        }
        case DCMD::CTRL_MODE_IDLE:
            state.commander_ctrl_mode = req_ctrl_mode;
            break;
        
        default:
        case DCMD::CTRL_MODE_ATT: {
            if (state.flight_status < state.FLIGHT_STATUS_IN_AIR) {
                //Not in air; goto IDLE
                state.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
            } else {
                state.commander_ctrl_mode = req_ctrl_mode;
            }
            
            break;
        }
    }

    if (state.ctrl_input_state == DCMD::CTRL_INPUT_NONE ) {
        if (state.commander_ctrl_mode !=DCMD::CTRL_MODE_LANDING && state.commander_ctrl_mode !=DCMD::CTRL_MODE_TAKEOFF && state.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION ) {
            //If no command come in, what to do
            //now is hover
            if (req_ctrl_mode != DCMD::CTRL_MODE_HOVER)
                request_ctrl_mode(DCMD::CTRL_MODE_HOVER);
        }
    }


}

void DroneCommander::process_control_mode() {
    //Request self ctrl mode can check vo vaild
    request_ctrl_mode(state.commander_ctrl_mode);

}

void DroneCommander::send_ctrl_cmd() {
    if (ctrl_cmd->ctrl_mode != DPCL::CTRL_CMD_POS_MODE) {
        pos_sp_inited = false;
    }

    if (!state.is_armed ||state.control_auth != DCMD::CTRL_AUTH_THIS ) {
        ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    }

    ctrl_cmd_pub.publish(*ctrl_cmd);
}


void DroneCommander::set_hover_target_position(double x, double y, double z) {
    if (state.is_armed && state.vo_valid && state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR && state.control_auth == DCMD::CTRL_AUTH_THIS) {
        hover_pos = Eigen::Vector3d(x, y, z);
        set_pos_setpoint(x, y, z);
        last_hover_count = control_count;
    }
}

void DroneCommander::prepare_control_hover() {
    bool fail_to_hover = false;
    if (last_hover_count < control_count - 1 && state.is_armed && state.vo_valid ) { // && this->state.control_auth == DCMD::CTRL_AUTH_THIS && state.is_armed && state.vo_valid ) {
        //Need to start new hover

        if (fabs(odometry.twist.twist.linear.x) > DANGER_SPEED_HOVER ||
            fabs(odometry.twist.twist.linear.y) > DANGER_SPEED_HOVER ||
            fabs(odometry.twist.twist.linear.z) > DANGER_SPEED_HOVER) {
            fail_to_hover = true;
        } else {
            set_hover_target_position(
                odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z);
            ROS_INFO("Entering hover mode, will hover at %3.2lf %3.2lf %3.2lf h %d c %d",
                hover_pos.x(),
                hover_pos.y(),
                hover_pos.z(),
                last_hover_count,
                control_count
            );
        }
    }

    if (fail_to_hover) {
        if (state.is_armed) {
            ROS_INFO("Onboard trying to Landing because try to hover failed");
            state.landing_mode = DCMD::LANDING_MODE_ATT;       
            state.landing_velocity = LANDING_VEL_Z_EMERGENCY;
            request_ctrl_mode(DCMD::CTRL_MODE_LANDING);
        }
    } else {
        last_hover_count = control_count;
    }
}


void DroneCommander::reset_ctrl_cmd() {
    pos_sp_inited = false;
    last_hover_count = 0;
    takeoff_inited = false;
    ctrl_cmd->ctrl_mode = DPCL::CTRL_CMD_IDLE_MODE;
    ctrl_cmd->pos_sp.x = 0;
    ctrl_cmd->pos_sp.y = 0;
    ctrl_cmd->pos_sp.z = 0;

    ctrl_cmd->vel_sp.x = 0;
    ctrl_cmd->vel_sp.y = 0;
    ctrl_cmd->vel_sp.z = 0;

    ctrl_cmd->att_sp.w = 0;
    ctrl_cmd->att_sp.x = 0;
    ctrl_cmd->att_sp.y = 0;
    ctrl_cmd->att_sp.z = 0;
    
    ctrl_cmd->z_sp = 0;

    reset_ctrl_cmd_max_vel();
    
}

void DroneCommander::reset_ctrl_cmd_max_vel() {
    ctrl_cmd->max_vel.x = DEFAULT_MAX_TITL_VEL;
    ctrl_cmd->max_vel.y = DEFAULT_MAX_TITL_VEL;
    ctrl_cmd->max_vel.z = DEFAULT_MAX_Z_VEL;    
}


bool DroneCommander::is_odom_valid(const nav_msgs::Odometry & _odom) {
    if ( fabs(_odom.twist.twist.linear.x) > MAX_ODOM_VELOCITY ||
        fabs(_odom.twist.twist.linear.y) > MAX_ODOM_VELOCITY ||
        fabs(_odom.twist.twist.linear.z) > MAX_ODOM_VELOCITY
    )
    {
        return false;
    }

    if ((ros::Time::now() - last_vo_image_ts).toSec() > MAX_VO_LATENCY ) {
        ROS_WARN_THROTTLE(10.0, "Latency on odom %3.1fms! VO is not valid now.", (ros::Time::now() - last_vo_image_ts.toSec())*1000);
        return false;
    }

    return true;
}

bool DroneCommander::is_rc_valid(const sensor_msgs::Joy & _rc) {
    //TODO: Test rc vaild function,
    // This only works for SBUS!!!!

    if (!rc_fail_detection) {
        return true;
    }
    if (
        _rc.axes[0] == 0 && 
        _rc.axes[1] == 0 && 
        _rc.axes[2] == 0 && 
        _rc.axes[3] == 0
    ) {
        return false;
    }
    return true;
}


void DroneCommander::ctrl_dev_callback(const dji_sdk::ControlDevice & _ctrl_dev) {
    //RC 0
    //App 1
    //SDK 2
    if (_ctrl_dev.controlDevice == 2) {
        state.control_auth = DCMD::CTRL_AUTH_THIS;
    }

    if (_ctrl_dev.controlDevice == 1) {
        state.control_auth = DCMD::CTRL_AUTH_APP;
    }

    if (_ctrl_dev.controlDevice == 0) {
        state.control_auth = DCMD::CTRL_AUTH_RC;
    }

}

void DroneCommander::reset_yaw_sp() {
    if (state.djisdk_valid) {
        if (state.vo_valid) {
            ctrl_cmd->yaw_sp = yaw_vo;
        } else {
            ctrl_cmd->yaw_sp = yaw_fc;
        }
        yaw_sp_inited = true;
    }
}

inline Eigen::Vector3d quat2eulers(Eigen::Quaterniond quat) {
    Eigen::Vector3d rpy;
    rpy.x() = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()),
                    1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()));
    rpy.y() = asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
    rpy.z() = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()),
                    1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z()));
    return rpy;
}

int main(int argc, char** argv)
{

    ROS_INFO("SWARM_COMMANDER_CONTROL_INIT\nIniting\n");

    ros::init(argc, argv, "drone_commander");

    ros::NodeHandle nh("drone_commander");

    DroneCommander swarm_commander(nh);

    ROS_INFO("Drone Commander is ONLINE! \n");
    ros::spin();

}
