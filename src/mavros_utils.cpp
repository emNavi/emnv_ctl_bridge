#include "control_for_gym/mavros_utils.hpp"
#include <std_msgs/Float64.h>


MavrosUtils::MavrosUtils(ros::NodeHandle &_nh,CTRL_OUTPUT_LVEVL _ctrl_output_level)
{
    ctrl_level = _ctrl_output_level;
    hoverThrustEkf = new HoverThrustEkf(0.4f, 0.1f, 0.036f);
    // hoverThrustEkf = new HoverThrustEkf(0.4f, 0.1f, 0.0);


    last_recv_odom_time = ros::Time::now();
    // get state
    mav_state_sub = _nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosUtils::mav_state_cb, this);
    mav_current_odom_sub = _nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MavrosUtils::mav_local_odom_cb, this);
    imu_sub = _nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &MavrosUtils::mav_imu_cb, this);

    // Note: do NOT change it to /mavros/imu/data_raw !!!
    mav_low_ctrl_pub = _nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10); // give atti cmd

    // 注意不要用 target_attitude ,里面的油门可能不正确
    // mav_atti_target_sub = _nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10, &MavrosUtils::mav_atti_target_cb, this);
    mav_atti_target_sub = _nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10, &MavrosUtils::mav_atti_target_cb, this);

    arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // pub hover thrust
    hover_thrust_pub = _nh.advertise<std_msgs::Float64>("/hover_thrust", 10);

    
}
MavrosUtils::~MavrosUtils()
{
}
// void MavrosUtils::ctrl_update(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, Eigen::Vector3d des_acc, double des_yaw)
// {
//     lin_controller.update(cmd, 0.0, 0.01); // dt 暂时无用
// }
void MavrosUtils::ctrl_update(Eigen::Vector3d des_vel,double yaw, double dt)
{
    lin_controller.update(des_vel, 0.0, 0.01); // dt 暂时无用
    _mav_low_cmd.thrust = lin_controller.thrust_exp;
    if(ctrl_level == CTRL_OUTPUT_LVEVL::RATE)
        _mav_low_cmd.rate = atti_controller.update(_mav_odom.attitude,lin_controller.q_exp);
    else if(ctrl_level == CTRL_OUTPUT_LVEVL::ATTI)
        _mav_low_cmd.attitude = lin_controller.q_exp;
}
void MavrosUtils::set_motors_idling()
{
    if(ctrl_level == CTRL_OUTPUT_LVEVL::RATE)
    {
        _mav_low_cmd.rate = Eigen::Vector3d::Zero();
    }
    else if(ctrl_level == CTRL_OUTPUT_LVEVL::ATTI)
    {
        _mav_low_cmd.attitude =  _mav_odom.attitude;
    }
    _mav_low_cmd.thrust = 0.04;
}

void MavrosUtils::send_low_cmd()
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::string("FCU");
    msg.thrust = _mav_low_cmd.thrust;    // body z axis
    if(ctrl_level == CTRL_OUTPUT_LVEVL::RATE)
    {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        msg.body_rate.x = _mav_low_cmd.rate(0);
        msg.body_rate.y = _mav_low_cmd.rate(1);
        msg.body_rate.z = _mav_low_cmd.rate(2);
    }
    else if(ctrl_level == CTRL_OUTPUT_LVEVL::ATTI)
    {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        msg.orientation.x = _mav_low_cmd.attitude.x();
        msg.orientation.y = _mav_low_cmd.attitude.y();
        msg.orientation.z = _mav_low_cmd.attitude.z();
        msg.orientation.w = _mav_low_cmd.attitude.w();
    }

    mav_low_ctrl_pub.publish(msg);


}

void MavrosUtils::connect()
{
    // wait for FCU connection
    ros::Rate rate(2);
    ROS_INFO("FCU connecting");
    while (ros::ok() && !_mav_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        std::cout << ">";
    }
    std::cout << std::endl;
    ROS_INFO("FCU connected");
    // TODO:send a few setpoints before starting
    // 对于 /setpoint_raw/attitude 似乎不需要也可以
}



// ///////////////// Checkout Status

bool MavrosUtils::request_offboard()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
        return true;
    }
    return false;
}

bool MavrosUtils::request_arm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}

bool MavrosUtils::request_disarm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;

    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}


/////////////////////////// callback
void MavrosUtils::mav_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    _mav_state.armed = msg->armed;
    _mav_state.connected = msg->connected;
    _mav_state.mode = msg->mode;
}
void MavrosUtils::mav_imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    _mav_odom.acc(0) = msg->linear_acceleration.x;
    _mav_odom.acc(1) = msg->linear_acceleration.y;
    _mav_odom.acc(2) = msg->linear_acceleration.z;
        
    double dt = (msg->header.stamp - last_recv_odom_time).toSec();
    last_recv_odom_time = msg->header.stamp;
    if(dt > 0.1)
    {
        ROS_WARN("IMU dt is too large");
        return;
    }
    // ROS_INFO_STREAM(" _mav_low_cmd.thrust" <<  _mav_low_cmd.thrust << "dt" << dt);
    if(_mav_low_cmd.thrust > 0.1)
    {
        hoverThrustEkf->predict(dt); // dt
        hoverThrustEkf->fuseAccZ(_mav_odom.acc(2)-CONSTANTS_ONE_G, _mav_low_cmd.thrust);
        _hover_thrust = hoverThrustEkf->getHoverThrust();
        hoverThrustEkf->fuseAccZ(_mav_odom.acc(2)-CONSTANTS_ONE_G, _mav_low_cmd.thrust);
        lin_controller.set_hover_thrust(_hover_thrust);

    }
    
    // hoverThrustEkf->printLog();
    // _hover_thrust = 0.30;
    std_msgs::Float64 hover_thrust_msg;
    hover_thrust_msg.data = _hover_thrust;
    hover_thrust_pub.publish(hover_thrust_msg);
    lin_controller.set_hover_thrust(_hover_thrust);
    
}
void MavrosUtils::mav_local_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    _mav_odom.position(0) = msg->pose.pose.position.x;
    _mav_odom.position(1) = msg->pose.pose.position.y;
    _mav_odom.position(2) = msg->pose.pose.position.z;

    _mav_odom.velocity(0) = msg->twist.twist.linear.x;
    _mav_odom.velocity(1) = msg->twist.twist.linear.y;
    _mav_odom.velocity(2) = msg->twist.twist.linear.z;

    _mav_odom.attitude.x() = msg->pose.pose.orientation.x;
    _mav_odom.attitude.y() = msg->pose.pose.orientation.y;
    _mav_odom.attitude.z() = msg->pose.pose.orientation.z;
    _mav_odom.attitude.w() = msg->pose.pose.orientation.w;

    _mav_odom.rate(0) = msg->twist.twist.angular.x;
    _mav_odom.rate(1) = msg->twist.twist.angular.y;
    _mav_odom.rate(2) = msg->twist.twist.angular.z;
    lin_controller.set_status(_mav_odom.position, _mav_odom.velocity, _mav_odom.rate, _mav_odom.attitude);

}

void MavrosUtils::mav_atti_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    // ROS_INFO_STREAM("_mav_odom.acc(2)" << _mav_odom.acc(2) << "_mav_low_cmd.target_thrust"<< _mav_low_cmd.target_thrust);


    // TODO: need to check the thrust is in world frame or body frame
    // barometer cannot be turned off?
    // In /mavros/imu/data msg ,acceleration is from HIGHRES_IMU, is barometer will calibrate the acceleration?
    // _mav_odom.acc need transform to world frame
    // _mav_low_cmd.target_thrust need transform to world frame
    
    // _mav_odom.attitude world -> body， 
    //  油门 是 body 下 [0,0,_mav_low_cmd.target_thrust]
    // 转换 油门到 world frame下

    Eigen::Vector3d thrust_world = _mav_odom.attitude*Eigen::Vector3d(0,0,_mav_low_cmd.thrust);
    Eigen::Vector3d acc_world = _mav_odom.attitude*_mav_odom.acc;
    // hoverThrustEkf->fuseAccZ(acc_world(2)-CONSTANTS_ONE_G, thrust_world(2));
    // hoverThrustEkf->printLog();


    
}