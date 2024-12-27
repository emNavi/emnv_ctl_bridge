#ifndef __MAVROS_UTILS_HPP
#define __MAVROS_UTILS_HPP

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <strings.h>
#include <Eigen/Eigen>
#include <std_msgs/String.h> 

#include "control_for_gym/linear_controller.hpp"
#include "control_for_gym/my_math.hpp"
#include "control_for_gym/Px4AttitudeController.hpp"
#include "control_for_gym/params_parse.hpp"
#include "control_for_gym/FSM.hpp"

extern ParamsParse params_parse;
class MavContext
{
private:
    /* data */
public:
    MavContext(/* args */)
    {
        last_recv_odom_time = ros::Time::now();
        landing_touchdown_start_time = ros::Time::now();

    };
    ~MavContext(){

    };
    ros::Time last_recv_odom_time;
    bool is_offboard=false; 
    bool connected = false;
    bool armed = false;
    std::string mode="";
    bool f_recv_takeoff_cmd = false;
    bool f_recv_land_cmd = false;

    // landing context
    ros::Time landing_touchdown_start_time;

    // last state context
    Eigen::Vector3d last_state_position;
    // Eigen::Vector3d last_state_velocity;
    Eigen::Quaterniond last_state_attitude;
};

class MavrosUtils
{
public:
    enum CTRL_OUTPUT_LVEVL {
        POSI = 0,
        RATE = 1,
        ATTI = 2
    };
    struct CtrlCommand
    {
        CTRL_OUTPUT_LVEVL ctrl_level;
        // position control
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        double yaw;
        // and rate control
        Eigen::Quaterniond attitude;
        Eigen::Vector3d rate;
        double thrust; //(-1,1)
    };
    struct State
    {
        bool connected = false;
        bool armed = false;
        std::string mode;
    };

    struct Odometry
    {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond attitude;
        Eigen::Vector3d rate;
        Eigen::Vector3d acc;
    };


private:
    // ==================  Node  ==================
    // Subscribe Mavros Msg
    ros::Subscriber state_sub_,current_odom_sub_,imu_data_sub_,atti_target_sub_;
    // Subscribe Ctrl Command
    ros::Subscriber pva_yaw_sub,local_linear_vel_sub,atti_sp_sub,rate_sp_sub;
    // Subscribe external information 
    ros::Subscriber vision_pose_sub,vrpn_pose_sub;
    // Subscribe takeoff and land command
    ros::Subscriber takeoff_sub,land_sub;

    // Publish Mavros State Msg
    ros::Publisher vision_pose_pub;
    // Publish Mavros Ctrl Msg
    ros::Publisher local_raw_pub,ctrl_pub_;
    // Publish MavUtils State
    ros::Publisher hover_thrust_pub_;

    ros::ServiceClient set_mode_client_, arming_client_;
    // ==================  Params  ==================

    HoverThrustEkf *hover_thrust_ekf_;
    double _hover_thrust=0.3;
    Px4AttitudeController atti_controller_;
    CTRL_OUTPUT_LVEVL ctrl_level = POSI;

public:
    MavrosUtils(ros::NodeHandle &_nh, CTRL_OUTPUT_LVEVL _ctrl_output_level);
    ~MavrosUtils();

    MavContext context_;
    Odometry odometry_;
    CtrlCommand ctrl_cmd_;
    CtrlFSM fsm;

    LinearControl lin_controller;
    // ==================  Callback  ==================
    void mavStateCallback(const mavros_msgs::State::ConstPtr &msg);
    void mavLocalOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void mavImuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void mavAttiTargetCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);

    void mavTakeoffCallback(const std_msgs::String::ConstPtr& msg, std::string name);
    void mavLandCallback(const std_msgs::String::ConstPtr& msg, std::string name);

    void mavVisionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mavVrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void mavPosCtrlSpCallback(const mavros_msgs::PositionTarget::ConstPtr &msg);
    void mavLocalLinearVelCallback(const mavros_msgs::PositionTarget::ConstPtr &msg);
    void mavAttiSpCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
    void mavRateSpCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);


    void waitConnected();
    bool requestArm();
    bool requestOffboard();
    bool requestDisarm();

    void sentCtrlCmd();
    void setMotorsIdling();
    void setThrustZero();

    double get_hover_thrust()
    {
        return _hover_thrust;
    }

    bool isOffboardMode()
    {
        return (context_.mode == "OFFBOARD");
    }
    bool isArmed()
    {
        return context_.armed;
    }
    
    /**
     * @brief 位置控制更新，输入期望位置，速度，加速度，yaw角
     * @param des_pos 期望位置
     * @param des_vel 期望速度
     * @param des_acc 期望加速度
     * @param des_yaw 期望yaw角
     * @return 返回初始化成功与否
     */
    void ctrlUpdate(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, Eigen::Vector3d des_acc, double des_yaw);
    // void ctrlUpdate(Eigen::Quaterniond des_atti, double des_thrust);
    // void ctrlUpdate(Eigen::MatrixX3d, double des_thrust);
    // void ctrlUpdate(Eigen::Vector3d des_rate, double des_thrust);
    void ctrlUpdate(Eigen::Vector3d des_vel,double des_yaw, double dt);

    void ctrl_loop();

};

#endif