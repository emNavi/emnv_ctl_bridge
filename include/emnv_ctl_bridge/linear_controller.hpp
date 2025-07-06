#ifndef __LINEAR_CONTROL_HPP
#define __LINEAR_CONTROL_HPP

#include <geometry_msgs/Twist.h>
#include "emnv_ctl_bridge/hover_thrust_ekf.hpp"
#include "emnv_ctl_bridge/my_math.hpp"
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


    double smooth_move_target_vel_;
    Eigen::Vector3d last_smooth_move_pos_;

    double _hover_thrust;
    double _max_tile_rad = 0.5; // 最大倾斜角度，单位弧度

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

    inline void set_gains(Eigen::Vector3d p_gain, Eigen::Vector3d v_gain, Eigen::Vector3d a_gain);

    void set_max_tile(double deg)
    {
        _max_tile_rad = deg * M_PI / 180.0; // 转换为弧度
    }

    
    void update(Eigen::Vector3d  &des_position,Eigen::Vector3d  &des_linear_vel,Eigen::Vector3d  &des_acc,double des_yaw,double dt);
    void smooth_move_init();
    void smooth_move(Eigen::Vector3d &des_position, double target_vel, double des_yaw, double dt);
    
    void set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Quaterniond q);
    Eigen::Quaterniond q_exp;
    double thrust_exp;
    void set_hover_thrust(double t)
    {
        _hover_thrust = t;
    }
    void setCtrlMask(int mask)
    {
        ctrl_mask = mask;
    }

};

inline LinearControl::LinearControl()
{
    ctrl_mask=CTRL_MASK::POSI|CTRL_MASK::VEL|CTRL_MASK::ACC;
    _gain_p << 2,2,2;
    _gain_v << 2,2,2;
    _gain_a << 1.5,1.5,1.5;
    thrust_exp = 0.3;
}
inline void LinearControl::set_gains(Eigen::Vector3d p_gain, Eigen::Vector3d v_gain, Eigen::Vector3d a_gain)
{
    _gain_p = p_gain;
    _gain_v = v_gain;
    _gain_a = a_gain;
    std::cout << "LinearControl gains set to: " << std::endl;
    std::cout << "P: " << _gain_p.transpose() << std::endl;
    std::cout << "V: " << _gain_v.transpose() << std::endl;
    std::cout << "A: " << _gain_a.transpose() << std::endl;
}

inline void LinearControl::set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Quaterniond q)
{
    _vel_world = vel;
    _pos_world = pos;
    _q_world = q;
    _angular_vel_world = angular_velocity;

 

}

/* 
* 更新姿态期望
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
    Eigen::Vector3d exp_lin_vel = Eigen::Vector3d(0,0,0);
    if(ctrl_mask & CTRL_MASK::POSI)
    {

        Eigen::Vector3d des_position_error = des_position - _pos_world;
        if( des_position_error.norm() > 5)
        {
            des_position_error = des_position_error.normalized() * 5; // 限制位置误差
        }
        exp_lin_vel += _gain_p.asDiagonal()*(des_position_error);
        if(exp_lin_vel.norm()  > 10)
        {
            exp_lin_vel = exp_lin_vel.normalized() *10; // 限制期望速度
        }
        Eigen::Vector3d diag = Eigen::Vector3d(0.2, 0.2, 0.6).cwiseProduct(_gain_v);
        des_acc += diag.asDiagonal()*(exp_lin_vel - _vel_world);
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
        _des_acc +=_gain_a.asDiagonal()*des_acc;
    }
    // _des_acc =        _gain_v_p.asDiagonal()*(des_linear_vel - _vel_world) + _gain_v_i.asDiagonal() * _des_acc_int;
    // _vel_error = (des_linear_vel - _vel_world).cwiseMax(-_param_max_des_vel).cwiseMin(_param_max_des_vel);
    // _des_acc_int += _vel_error ; 
    // _des_acc += Eigen::Vector3d(0,0,CONSTANTS_ONE_G);
    thrust_exp = _des_acc(2) * (_hover_thrust / CONSTANTS_ONE_G) + _hover_thrust;
    double roll, pitch, yaw, yaw_imu;
    double yaw_odom = MyMath::fromQuaternion2yaw(_q_world);
    

    double sin = std::sin(des_yaw);
    double cos = std::cos(des_yaw); // 不是 当前里程计的值，与px4的逻辑有关
    roll = (_des_acc(0) * sin - _des_acc(1) * cos) / CONSTANTS_ONE_G;
    pitch = (_des_acc(0) * cos + _des_acc(1) * sin) / CONSTANTS_ONE_G;
    if (std::abs(roll) > _max_tile_rad) {
        roll = (roll > 0 ? 1 : -1) * _max_tile_rad;
    }
    if (std::abs(pitch) > _max_tile_rad) {
        pitch = (pitch > 0 ? 1 : -1) * _max_tile_rad;
    }
    q_exp = Eigen::AngleAxisd(des_yaw, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}
inline void LinearControl::smooth_move_init()
{
    last_smooth_move_pos_ = _pos_world;
} 

inline void LinearControl::smooth_move(Eigen::Vector3d &des_position, double target_vel, double des_yaw, double dt)
{
    smooth_move_target_vel_ = target_vel;
    Eigen::Vector3d direction;
    Eigen::Vector3d des_pos;
    Eigen::Vector3d des_linear_vel = Eigen::Vector3d::Zero();
    if ((des_position - last_smooth_move_pos_).norm() > 1e-2) {
        direction = (des_position - last_smooth_move_pos_).normalized();
        last_smooth_move_pos_ = last_smooth_move_pos_ + direction * std::abs(smooth_move_target_vel_) * dt;
        des_linear_vel = std::abs(smooth_move_target_vel_) * direction;
    } else {
        des_linear_vel = Eigen::Vector3d::Zero();
    }
    des_pos = last_smooth_move_pos_;
    Eigen::Vector3d des_acc = Eigen::Vector3d::Zero(); // 平滑移动时加速度
    update(des_pos, des_linear_vel, des_acc, des_yaw, dt);
}


inline LinearControl::~LinearControl()
{
}
#endif