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
#include "ctrl_bridge/linear_controller.hpp"
#include "ctrl_bridge/my_math.hpp"
#include "ctrl_bridge/Px4AttitudeController.hpp"
#include "ctrl_bridge/params_parse.hpp"
#include "ctrl_bridge/FSM.hpp"


enum class CtrlMode {
    QUAD_T,
    RATE_T,
    PVA_Ys,
    Unknown // 处理无效输入
};


enum class CmdPubType {
    ATTI,
    RATE,
    POSY,
    Unknown // 处理无效输入
};
extern std::map<std::string, CmdPubType> cmdPubMap;
extern std::map<std::string, CtrlMode> ctrlModeMap;


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
    // ==================  Params  ==================
    CmdPubType ctrl_level;

    // 所有可能用到的控制变量
    struct CtrlCommand
    {
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
    ros::NodeHandle nh;
    // ==================  Node  ==================
    // Subscribe Mavros Msg
    ros::Subscriber state_sub_,current_odom_sub_,imu_data_sub_,atti_target_sub_;
    // Subscribe Ctrl Command
    ros::Subscriber pva_yaw_sub,atti_sp_sub,rate_sp_sub;
    // ros::Subscriber local_linear_vel_sub;
    // Subscribe external information 
    ros::Subscriber vision_pose_sub,vrpn_pose_sub;
    // Subscribe takeoff and land command
    ros::Subscriber takeoff_sub,land_sub;

    // Publish Mavros State Msg
    ros::Publisher vision_pose_pub;
    // Publish Mavros Ctrl Msg
    ros::Publisher local_pvay_pub,ctrl_atti_pub_;
    // Publish MavUtils State
    ros::Publisher hover_thrust_pub_;

    ros::ServiceClient set_mode_client_, arming_client_;
    // ==================  Params  ==================

    HoverThrustEkf *hover_thrust_ekf_;
    double _hover_thrust=0.3;
    Px4AttitudeController atti_controller_;

public:
    MavrosUtils(ros::NodeHandle &_nh);
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
    /**
     * @brief 向Mosvos请求解锁
     * 
     * @return true 
     * @return false 
     */
    bool requestArm();
    /**
     * @brief 向Mosvos请求切换到OFFBOARD模式。
     * @return 成功(true)
     * @return 失败(false)
     */
    bool requestOffboard();
    /**
     * @brief 向Mosvos请求电机上锁。
     * @return 成功(true)或失败(false)
     * @return 成功(true)
     * @return 失败(false)
     */   
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

    int set_bridge_mode(std::string ctrl_mode_str, std::string cmd_pub_type_str);


};

#endif