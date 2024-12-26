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
#include "control_for_gym/linear_controller.hpp"
#include "control_for_gym/my_math.hpp"

#include "control_for_gym/Px4AttitudeController.hpp"





struct mav_state
{
    bool connected = false;
    bool armed = false;
    std::string mode;
};
struct mav_odom
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond attitude;
    Eigen::Vector3d rate;
    Eigen::Vector3d acc;
};

struct mav_low_command
{
    Eigen::Quaterniond attitude;
    Eigen::Vector3d rate;
    double thrust; //(-1,1)
};

class MavrosUtils
{
public:
    enum CTRL_OUTPUT_LVEVL {
        POSI = 0,
        RATE = 1,
        ATTI = 2
    };
private:
    HoverThrustEkf *hoverThrustEkf;
    mav_state _mav_state;
    ros::Subscriber mav_state_sub;
    ros::Subscriber mav_current_odom_sub;
    ros::Subscriber mav_vel_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber mav_atti_target_sub;

    ros::Publisher mav_low_ctrl_pub;
    ros::Publisher hover_thrust_pub;

    ros::ServiceClient set_mode_client, arming_client;

    ros::Time last_recv_odom_time;
    bool is_offboard = false;
    double _hover_thrust=0.3;

    Px4AttitudeController atti_controller;
    CTRL_OUTPUT_LVEVL ctrl_level = POSI;

public:
    MavrosUtils(ros::NodeHandle &_nh, CTRL_OUTPUT_LVEVL _ctrl_output_level);
    ~MavrosUtils();


    mav_odom _mav_odom;
    mav_low_command _mav_low_cmd;

    LinearControl lin_controller;
    void connect();
    void mav_state_cb(const mavros_msgs::State::ConstPtr &msg);
    void mav_local_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void mav_imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
    void mav_atti_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg);

    bool request_arm();
    bool request_offboard();
    bool request_disarm();

    void send_low_cmd();
    void set_motors_idling();

    // quick function
    double get_hover_thrust()
    {
        return _hover_thrust;
    }

    bool isOffboardMode()
    {
        // return is_offboard;
        return (_mav_state.mode == "OFFBOARD");
    }
    bool isArmed()
    {
        return _mav_state.armed;
    }
    void update(geometry_msgs::Twist::ConstPtr cmd);
    void hover_update(geometry_msgs::Twist::ConstPtr cmd);
    
    void ctrl_update(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, Eigen::Vector3d des_acc, double des_yaw);
    // void ctrl_update(Eigen::Quaterniond des_atti, double des_thrust);
    // void ctrl_update(Eigen::MatrixX3d, double des_thrust);
    // void ctrl_update(Eigen::Vector3d des_rate, double des_thrust);
    void ctrl_update(Eigen::Vector3d des_vel,double des_yaw, double dt);

};

#endif