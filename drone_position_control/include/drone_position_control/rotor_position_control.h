#pragma once

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <stdio.h>
#include "controllers.h"

using namespace Eigen;

#define GRAVITY (9.80f)
#define MAX_HORIZON_VEL 10.0
#define MAX_VERTICAL_VEL 3.0
#define MAX_VERTICAL_ACC 10.0
#define MAX_HORIZON_ACC 10.0
#define MAX_TILT_ANGLE 0.8
#define MIN_THRUST 0.0

enum CTRL_FRAME {
    VEL_WORLD_ACC_WORLD,
    VEL_WORLD_ACC_BODY,
    VEL_BODY_ACC_BODY
};

enum FRAME_COOR_SYS {
    NED,
    FLU
};

enum THRUST_SP_TYPE {
    THRUST_PASSTHROUGH,
    THRUST_ACC
};

enum YAW_MODE {
    YAW_MODE_LOCK,
    YAW_MODE_KEEP,
    YAW_MODE_RATE
};

struct YawCMD{
    float yaw_sp = 0;
    YAW_MODE yaw_mode = YAW_MODE_LOCK;
};

struct RotorThrustControlParam {
    PIDParam abx;
    double level_thrust = 0.5;
    FRAME_COOR_SYS coor_sys = NED;
};

struct RotorPosCtrlParam {
    PIDParam p_x, p_y, p_z;
    PIDParam v_y, v_z, v_x;
    CTRL_FRAME ctrl_frame = CTRL_FRAME::VEL_WORLD_ACC_WORLD;
    RotorThrustControlParam thrust_ctrl;
    FRAME_COOR_SYS coor_sys = NED;
};

struct AttiCtrlOut {
    enum {
        THRUST_MODE_THRUST,
        THRUST_MODE_VELZ
    };
    Eigen::Quaterniond atti_sp = Eigen::Quaterniond(1, 0, 0, 0);
    double roll_sp = 0, pitch_sp = 0, yaw_sp = 0;
    double thrust_sp = 0;
    double abx_sp = 0;
    int thrust_mode;
};

inline Eigen::Vector3d quat2eulers(const Eigen::Quaterniond & quat) {
    Eigen::Vector3d rpy;
    rpy.x() = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()),
                    1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()));
    rpy.y() = asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
    rpy.z() = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()),
                    1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z()));
    return rpy;
}


class RotorThrustControl {
    RotorThrustControlParam param;
    PIDController con;
public:
    double acc = 0;

    RotorThrustControl(RotorThrustControlParam _param):
        param(_param), con(_param.abx) {
        //TODO:
    }

    //We using NED in our control system
    void set_acc(double _acc) {
        acc = _acc;
    }

    void reset() {
        con.reset();
    }

    double get_level_thrust() {
        return param.level_thrust;
    }

    inline double control(const double & abx_sp, double dt) {
        //This is NED
        //Abx SP bigger, output thrust should be smaller(fly to ground)
        //Abx SP smaller, output thrust should be bigger
        //printf("In thrust control abx %3.2f acc %3.2f\n", abx_sp, acc);
        return -con.control(abx_sp - acc, dt, true) + param.level_thrust;
    }
};


//This position control output world accerelation
class RotorPositionControl {
    RotorPosCtrlParam param;
    PIDController px_con, py_con, pz_con;
    PIDController vx_con, vy_con, vz_con;


    Eigen::Quaterniond yaw_transverse;

    bool pos_inited = false;
    bool vel_inited = false;
    bool acc_inited = false;
    bool att_inited = false;

    double yaw;

    Eigen::Vector3d euler_rpy = Eigen::Vector3d(0, 0, 0);

public:
    RotorThrustControl thrust_ctrl;

    Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d acc = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel_body = Eigen::Vector3d(0, 0, 0);
    Eigen::Quaterniond quat = Eigen::Quaterniond(1, 0, 0, 0);


    RotorPositionControl(RotorPosCtrlParam _param):
        param(_param),
        px_con(_param.p_x), py_con(_param.p_y), pz_con(_param.p_z),
        vx_con(_param.v_x), vy_con(_param.v_y), vz_con(_param.v_z),
        thrust_ctrl(_param.thrust_ctrl) {

    }

    void position_controller_reset() {
        printf("Non position control mode, reset position controller\n");
        px_con.reset();
        py_con.reset();
        pz_con.reset();
    }

    virtual void set_pos(const Eigen::Vector3d & _pos) {
        pos = _pos;
        pos_inited = true;
    }
    
    virtual void set_global_vel(const Eigen::Vector3d & _vel) {
        vel = _vel;
        vel_inited = true;

        vel_body = quat.inverse() * vel;
    }

    virtual void set_body_acc(const Eigen::Vector3d & _acc) {
        acc = _acc;

        if(param.coor_sys == FRAME_COOR_SYS::FLU) {
            acc.z() = - acc.z();
            acc.y() = - acc.y();
        }
        
        thrust_ctrl.set_acc(acc.z());

    }

    virtual void set_attitude(const Eigen::Quaterniond & _quat) {
        quat = _quat;

        euler_rpy = quat2eulers(quat);

        yaw_transverse = Eigen::AngleAxisd(euler_rpy.z(), Vector3d::UnitZ());

        att_inited = true;

        yaw = euler_rpy.z();
    }

    virtual double control_pos_z(const double z_sp, double dt) {
            return float_constrain(pz_con.control2(z_sp, pos.z(), dt), -3,3);
    }
    
    virtual Eigen::Vector3d control_pos(const Eigen::Vector3d & pos_sp, double dt) {
        Eigen::Vector3d vel_sp(0, 0, 0);
        if (pos_inited) {
            Eigen::Vector3d pos_sp_body = yaw_transverse.inverse() * pos_sp;
            Eigen::Vector3d pos_body = yaw_transverse.inverse() * pos;

            // printf("CTRL POS %f %f SP %f %f POSBDY %f %f POSBDYSP %f %f YAW %f\n",
            //     pos.x(), pos.y(),
            //     pos_sp.x(), pos_sp.y(),
            //     pos_body.x(), pos_body.y(),
            //     pos_sp_body.x(), pos_sp_body.y(),
            //     yaw
            // );

            vel_sp.x() = float_constrain(px_con.control2(pos_sp_body.x(), pos_body.x(), dt), -MAX_HORIZON_VEL, MAX_HORIZON_VEL);
            vel_sp.y() = float_constrain(py_con.control2(pos_sp_body.y(), pos_body.y(), dt), -MAX_HORIZON_VEL, MAX_HORIZON_VEL);
            vel_sp.z() = float_constrain(control_pos_z(pos_sp_body.z(), dt), -MAX_VERTICAL_VEL, MAX_VERTICAL_VEL);

            if (param.ctrl_frame == VEL_BODY_ACC_BODY) {
            } else {
                vel_sp = yaw_transverse * vel_sp;
            }
        }

        return vel_sp;
    }

    //Issue on acc output!!!
    //Should not on *body*, should based on cooridnate transversed by yaw
    //Which means x and y is on planar

    virtual double control_vel_z(double vel_z_sp, double dt)
    {
        double one_g = GRAVITY;
        if (param.ctrl_frame == CTRL_FRAME::VEL_WORLD_ACC_WORLD)
        {
            one_g = 0;
        }
        double az = vz_con.control2(vel_z_sp, vel.z(), dt);
        if(param.coor_sys == FRAME_COOR_SYS::FLU) {
            return az + one_g;
        } else if(param.coor_sys == FRAME_COOR_SYS::NED) {
           return  az - one_g;
        }
        return 0;
    }

    virtual Eigen::Vector3d control_vel(const Eigen::Vector3d & vel_sp, double dt, bool input_body_frame=true, bool output_body_frame=true) {
        Eigen::Vector3d acc_sp(0, 0, 0);
        if (vel_inited) {

            Eigen::Vector3d vel_sp_body = vel_sp;
            Eigen::Vector3d vel_body = vel;
            if (param.ctrl_frame == CTRL_FRAME::VEL_WORLD_ACC_BODY || param.ctrl_frame == CTRL_FRAME::VEL_WORLD_ACC_WORLD) {
                vel_sp_body = yaw_transverse.inverse() * vel_sp;
                vel_body = yaw_transverse.inverse() * vel;
            }

            acc_sp.x() = float_constrain(vx_con.control2(vel_sp_body.x(), vel_body.x(), dt), -MAX_HORIZON_ACC, MAX_HORIZON_ACC);
            acc_sp.y() = float_constrain(vy_con.control2(vel_sp_body.y(), vel_body.y(), dt), -MAX_HORIZON_ACC, MAX_HORIZON_ACC);
            acc_sp.z() = float_constrain(control_vel_z(vel_sp.z(), dt), -MAX_VERTICAL_ACC, MAX_VERTICAL_ACC);

            if (param.ctrl_frame == CTRL_FRAME::VEL_WORLD_ACC_WORLD) {
                acc_sp = yaw_transverse * acc_sp;

            }
        }

        return acc_sp;    
    }
    virtual void reset() {
        vx_con.reset();
        vy_con.reset();
        vz_con.reset();

        px_con.reset();
        py_con.reset();
        pz_con.reset();

        thrust_ctrl.reset();
    }

    virtual AttiCtrlOut control_acc(Eigen::Vector3d acc_sp, YawCMD yaw_cmd, double dt) {
        AttiCtrlOut ret;
        
        double pitch_sp = 0;
        double roll_sp = 0;
        double yaw_sp = euler_rpy.z();
        
        if (param.ctrl_frame == CTRL_FRAME::VEL_WORLD_ACC_WORLD) {
            acc_sp = yaw_transverse.inverse() * acc_sp;
        }

        if(param.coor_sys == FRAME_COOR_SYS::FLU) {
            acc_sp.z() = - acc_sp.z();
            acc_sp.y() = - acc_sp.y();
        }
        
        //Norm vector
        Eigen::Vector3d Up = Eigen::Vector3d(0, 0, -1);
        Up = quat*Up; 
        //All controller is ON NED
        //So for hover, acc_sp and acc should be -9.8
        //Accerelation UP, acc should be smaller than -9.8
        acc_sp.z() = acc_sp.z() - 9.8;
        
        // TODO:
        // Do not care about aerodynamics drag
        // Only for hover
        //Note that abx sp should be negative here!
        ret.abx_sp = (Up.dot(acc_sp) * Up).z();
        
        //ret.thrust_sp = float_constrain(thrust_ctrl.control(ret.abx_sp, dt), MIN_THRUST, 1);
        //printf("AccZ %3.2f abx sp %3.2f body acc %3.2f thrustsp %3.2f\n", acc_sp.z(), ret.abx_sp, acc.z(), ret.thrust_sp);

        if (fabs(acc_sp.z()) > 0.1) {
            pitch_sp = float_constrain(- asin(acc_sp.x() / acc_sp.norm()), -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
            roll_sp = float_constrain(asin(acc_sp.y() /( acc_sp.norm() * cos(pitch_sp))), -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
        }

        if (yaw_cmd.yaw_mode == YAW_MODE::YAW_MODE_LOCK) {
            yaw_sp = yaw_cmd.yaw_sp;
        }

        if (yaw_cmd.yaw_mode == YAW_MODE::YAW_MODE_RATE) {
            yaw_sp = yaw_cmd.yaw_sp * dt + yaw_sp;
            printf("Yaw rate: dyaw %3.2f\n", yaw_cmd.yaw_sp*57.3);
        }

        printf("In CTRL accsp %3.2f %3.2f %3.2f\nsp p %3.2f r %3.2f y %3.2f\n",
            acc_sp.x(), acc_sp.y(), acc_sp.z(),
            pitch_sp, roll_sp, yaw_sp);
        ret.roll_sp = roll_sp;
        ret.pitch_sp = pitch_sp;
        ret.yaw_sp = yaw_sp;
        ret.thrust_mode = AttiCtrlOut::THRUST_MODE_THRUST;
        ret.atti_sp = Eigen::AngleAxisd(yaw_sp, Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch_sp, Vector3d::UnitY()) * Eigen::AngleAxisd(roll_sp, Vector3d::UnitX());

        return ret;
    }  
};
