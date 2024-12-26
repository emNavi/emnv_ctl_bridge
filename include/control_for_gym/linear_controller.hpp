#ifndef __LINEAR_CONTROL_HPP
#define __LINEAR_CONTROL_HPP

#include <geometry_msgs/Twist.h>
#include "control_for_gym/hover_thrust_ekf.hpp"
#include "control_for_gym/my_math.hpp"
#include <Eigen/Eigen>
#include "derivative.hpp"



//  在世界坐标系下完成
class LinearControl
// This class is a simple velocity controller for landing and hovering
{
private:
    Derivate velDerivateZ_;

    Eigen::Vector3d _gain_p,_gain_v,_gain_a;


    Eigen::Vector3d _pos_world;
    Eigen::Vector3d _vel_world;
    Eigen::Quaterniond _q_world;
    Eigen::Vector3d _angular_vel_world;


    Eigen::Vector3d _des_vel;
    Eigen::Vector3d _vel_error;
    Eigen::Vector3d _des_acc;
    Eigen::Vector3d _des_acc_int;


    Eigen::Vector3d _param_max_des_vel{5,5,5};

    double _hover_thrust;


    /* data */
public:
    LinearControl(/* args */);
    ~LinearControl();
    enum CTRL_MASK{
        POSI = 1,
        VEL = 2,
        ACC = 4
    };
    int ctrl_mask;
    
    // void update(geometry_msgs::Twist::ConstPtr  &des,double yaw_sp,double dt);
    void update(Eigen::Vector3d  &des_position,Eigen::Vector3d  &des_linear_vel,Eigen::Vector3d  &des_acc,double des_yaw,double dt);
    void update(Eigen::Vector3d  &des_linear_vel,double yaw_sp,double dt);
    
    void set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Vector4d q, double dt);
    void set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Quaterniond q);
    Eigen::Quaterniond q_exp;
    double thrust_exp;
    void set_hover_thrust(double t)
    {
        _hover_thrust = t;
    }

};

inline LinearControl::LinearControl()
{
    ctrl_mask=CTRL_MASK::POSI|CTRL_MASK::VEL|CTRL_MASK::ACC;
    _gain_p << 1.5,1.5,1.5;
    _gain_v << 1.5,1.5,1.5;
    _gain_a << 1.5,1.5,1.5;
    thrust_exp = 0.3;
}
inline void LinearControl::set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Vector4d q, double dt)
{
    _vel_world = vel;
    _pos_world = pos;
    _q_world.w() = q(0);
    _q_world.x() = q(1);
    _q_world.y() = q(2);
    _q_world.z() = q(3);
    _angular_vel_world = angular_velocity;


}
inline void LinearControl::set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Quaterniond q)
{
    _vel_world = vel;
    _pos_world = pos;
    _q_world = q;
    _angular_vel_world = angular_velocity;

 

}
// inline LinearControl::smooth_move_to(Eigen::Vector3d &des_pos,double des_yaw,double interval)
// {
//     Eigen::Vector3d des_vel = (des_pos - _pos_world) / interval;
//     Eigen::Vector3d des_acc = (des_vel - _vel_world) / interval;
//     Eigen::Vector3d des_jerk = (des_acc - _des_acc) / interval;
//     Eigen::Vector3d des_snap = (des_jerk - _des_jerk) / interval;
//     _des_acc = des_acc;
//     _des_jerk = des_jerk;
//     _des_snap = des_snap;
// }

/* 
* 由期望位姿通过微分平坦计算期望姿态
* @param des_position 期望位置(world frame)
* @param des_linear_vel 期望速度(world frame)
* @param des_acc 期望加速度(world frame)
* @param yaw_sp 期望偏航角(world frame)
* @param dt 时间间隔
*/
inline void LinearControl::update(Eigen::Vector3d  &des_position,Eigen::Vector3d  &des_linear_vel,Eigen::Vector3d  &des_acc,double des_yaw,double dt)
{
    // 应该用完整的PID控制器
    _des_acc = Eigen::Vector3d(0,0,0);
    if(ctrl_mask & CTRL_MASK::POSI)
    {
        _des_acc += _gain_p.asDiagonal()*(des_position - _pos_world);
    }
    if(ctrl_mask & CTRL_MASK::VEL)
    {
        _des_acc += _gain_v.asDiagonal()*(des_linear_vel - _vel_world);
        // _des_acc = _gain_v_p.asDiagonal()*(des_linear_vel - _vel_world) + _gain_v_i.asDiagonal() * _des_acc_int;
        // _vel_error = (des_linear_vel - _vel_world).cwiseMax(-_param_max_des_vel).cwiseMin(_param_max_des_vel);
        // _des_acc_int += _vel_error ; 



    }
    if(ctrl_mask & CTRL_MASK::ACC)
    {
        _des_acc += des_acc;
    }

    // _des_acc =        _gain_v_p.asDiagonal()*(des_linear_vel - _vel_world) + _gain_v_i.asDiagonal() * _des_acc_int;
    // _vel_error = (des_linear_vel - _vel_world).cwiseMax(-_param_max_des_vel).cwiseMin(_param_max_des_vel);
    // _des_acc_int += _vel_error ; 
    // _des_acc += Eigen::Vector3d(0,0,CONSTANTS_ONE_G);

    thrust_exp = _des_acc(2) * (_hover_thrust / CONSTANTS_ONE_G) + _hover_thrust;
    // ROS_INFO_STREAM("_des_acc"<< _des_acc << "thrust_exp" << thrust_exp << "_hover_thrust"<<_hover_thrust);
    double roll, pitch, yaw, yaw_imu;
    double yaw_odom = MyMath::fromQuaternion2yaw(_q_world);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    roll = (_des_acc(0) * sin - _des_acc(1) * cos) / CONSTANTS_ONE_G;
    pitch = (_des_acc(0) * cos + _des_acc(1) * sin) / CONSTANTS_ONE_G;
    q_exp = Eigen::AngleAxisd(des_yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}
/* 
* 由期望位姿通过微分平坦计算期望姿态
* @param des_linear_vel 期望速度(world frame)
* @param yaw_sp 期望偏航角(world frame)
*/
inline void LinearControl::update(Eigen::Vector3d  &des_linear_vel,double des_yaw,double dt)
{
    ctrl_mask = CTRL_MASK::VEL;
    Eigen::Vector3d empty_vec = Eigen::Vector3d(0,0,0);
    update(empty_vec,des_linear_vel,empty_vec,des_yaw,dt);
}


inline LinearControl::~LinearControl()
{
}
#endif