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

struct mav_atti_command
{
    Eigen::Quaterniond attitude;
    double thrust; //(-1,1)
    double target_thrust;
};

class MavrosUtils
{
private:
    HoverThrustEkf *hoverThrustEkf;
    mav_state _mav_state;
    mav_atti_command _mav_atti_cmd;
    ros::Subscriber mav_state_sub;
    ros::Subscriber mav_current_odom_sub;
    ros::Subscriber mav_vel_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber mav_atti_target_sub;

    ros::Publisher mav_atti_ctrl_pub;

    ros::ServiceClient set_mode_client, arming_client;

    ros::Time last_recv_odom_time;
    bool is_offboard = false;
    double _hover_thrust=0.3;

public:
    MavrosUtils(ros::NodeHandle &_nh);
    ~MavrosUtils();
    mav_odom _mav_odom;

    LinearControl controller;
    void connect();
    void mav_state_cb(const mavros_msgs::State::ConstPtr &msg);
    void mav_local_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void mav_imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
    void mav_atti_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg);

    bool request_arm();
    bool request_offboard();
    bool request_disarm();
    void send_atti_cmd();
    void set_motors_idling();
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
};

#endif