#ifndef __LINEAR_CONTROL_HPP
#define __LINEAR_CONTROL_HPP

#include <geometry_msgs/Twist.h>
#include "ctrl_bridge/hover_thrust_ekf.hpp"
#include "ctrl_bridge/my_math.hpp"
#include <Eigen/Eigen>
#include "derivative.hpp"

//  在世界坐标系下完成
class LinearControl
// This class is a simple velocity controller for landing and hovering
{
private:
    Derivate velDerivateZ_;

    Eigen::Vector3d _gain_v_p,_gain_v_i,_gain_v_d;

    Eigen::Vector3d _pos_world;


    Eigen::Vector3d _vel_world;
    Eigen::Vector3d _des_vel;
    Eigen::Vector3d _vel_error;


    Eigen::Quaterniond _q_world;



    Eigen::Vector3d _angular_vel_world;

    Eigen::Vector3d _des_acc;
    Eigen::Vector3d _des_acc_int;


    Eigen::Vector3d _param_max_des_vel{5,5,5};

    double _hover_thrust;


    /* data */
public:
    LinearControl(/* args */);
    ~LinearControl();
    void update(geometry_msgs::Twist::ConstPtr  &des,double yaw_sp,double dt);

    void set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Vector4d q, double dt);
    void set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Quaterniond q);
    Eigen::Quaterniond q_exp;
    double thrust_exp;
    void set_hover_thrust(double t)
    {
        _hover_thrust = t;
    }

};

inline LinearControl::LinearControl(/* args */)
{
    _gain_v_p << 1.5,1.5,1.5;
    _gain_v_i << 0.0,0.0,0.0;
    _gain_v_d << 0.0,0.0,0.0;

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



inline void LinearControl::update(geometry_msgs::Twist::ConstPtr  &des,double yaw_sp,double dt)
{
    _des_vel(0) = des->linear.x;
    _des_vel(1) = des->linear.y;
    _des_vel(2) = des->linear.z;

    _des_acc = _gain_v_p.asDiagonal()*(_des_vel - _vel_world) + _gain_v_i.asDiagonal() * _des_acc_int;
    _vel_error = (_des_vel - _vel_world).cwiseMax(-_param_max_des_vel).cwiseMin(_param_max_des_vel);


    _des_acc_int += _vel_error ; 
    // _des_acc += Eigen::Vector3d(0,0,CONSTANTS_ONE_G);


    thrust_exp = _des_acc(2) * (_hover_thrust / CONSTANTS_ONE_G) + _hover_thrust;
    ROS_INFO_STREAM("_des_acc"<< _des_acc << "thrust_exp" << thrust_exp << "_hover_thrust"<<_hover_thrust);

    double roll, pitch, yaw, yaw_imu;
    double yaw_odom = MyMath::fromQuaternion2yaw(_q_world);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    roll = (_des_acc(0) * sin - _des_acc(1) * cos) / CONSTANTS_ONE_G;
    pitch = (_des_acc(0) * cos + _des_acc(1) * sin) / CONSTANTS_ONE_G;

    q_exp = Eigen::AngleAxisd(yaw_sp, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());


}


inline LinearControl::~LinearControl()
{
}
#endif