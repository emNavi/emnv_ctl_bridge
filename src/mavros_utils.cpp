#include "control_for_gym/mavros_utils.hpp"

MavrosUtils::MavrosUtils(ros::NodeHandle &_nh)
{
    hoverThrustEkf = new HoverThrustEkf(0.4f, 0.1f, 0.036f);

    last_recv_odom_time = ros::Time::now();
    // get state
    mav_state_sub = _nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosUtils::mav_state_cb, this);
    mav_current_odom_sub = _nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MavrosUtils::mav_local_odom_cb, this);
    imu_sub = _nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &MavrosUtils::mav_imu_cb, this);

    // Note: do NOT change it to /mavros/imu/data_raw !!!
    mav_atti_ctrl_pub = _nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    // mav_atti_target_sub = _nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10, &MavrosUtils::mav_atti_target_cb, this);
    mav_atti_target_sub = _nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10, &MavrosUtils::mav_atti_target_cb, this);

    arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}
MavrosUtils::~MavrosUtils()
{
}

void MavrosUtils::update(geometry_msgs::Twist::ConstPtr cmd)
{

    controller.update(cmd, 0.0, 0.01); // dt 暂时无用
    _mav_atti_cmd.attitude = controller.q_exp;
    _mav_atti_cmd.thrust = controller.thrust_exp;
    ROS_INFO_STREAM(" cmd->linear.z " << cmd->linear.z<<" thrust_exp "<<controller.thrust_exp << "hover t" << _hover_thrust);

}
void MavrosUtils::set_motors_idling()
{
    _mav_atti_cmd.attitude = _mav_odom.attitude;
    _mav_atti_cmd.thrust = 0.04;
}
void MavrosUtils::send_atti_cmd()
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::string("FCU");

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    msg.orientation.x = _mav_atti_cmd.attitude.x();
    msg.orientation.y = _mav_atti_cmd.attitude.y();
    msg.orientation.z = _mav_atti_cmd.attitude.z();
    msg.orientation.w = _mav_atti_cmd.attitude.w();

    msg.thrust = _mav_atti_cmd.thrust;    // enu z axis

    mav_atti_ctrl_pub.publish(msg);
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
    
    // Eigen::Vector3d thrust_z = _mav_odom.attitude.toRotationMatrix() * Eigen::Vector3d(0, 0, _mav_atti_cmd.thrust);
    
    double dt = (msg->header.stamp - last_recv_odom_time).toSec();
    // ROS_INFO_STREAM(" _mav_atti_cmd.thrust" <<  _mav_atti_cmd.target_thrust << "dt" << dt);
    hoverThrustEkf->predict(dt); // dt
    _hover_thrust = hoverThrustEkf->getHoverThrust();
    // _hover_thrust = 0.30;

    controller.set_hover_thrust(_hover_thrust);
    last_recv_odom_time = msg->header.stamp;
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
    controller.set_status(_mav_odom.position, _mav_odom.velocity, _mav_odom.rate, _mav_odom.attitude);

}

void MavrosUtils::mav_atti_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    _mav_atti_cmd.target_thrust = msg->thrust;
    ROS_INFO_STREAM("_mav_odom.acc(2)" << _mav_odom.acc(2) << "_mav_atti_cmd.target_thrust"<< _mav_atti_cmd.target_thrust);
    hoverThrustEkf->fuseAccZ(_mav_odom.acc(2)-CONSTANTS_ONE_G, _mav_atti_cmd.target_thrust);
    hoverThrustEkf->printLog();
}