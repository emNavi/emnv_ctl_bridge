#include "control_for_gym/mavros_utils.hpp"
#include <std_msgs/Float64.h>


MavrosUtils::MavrosUtils(ros::NodeHandle &_nh,CTRL_OUTPUT_LVEVL _ctrl_output_level)
{
    ctrl_level = _ctrl_output_level;
    hover_thrust_ekf_ = new HoverThrustEkf(0.4f, 0.1f, 0.036f);

    // sub mavros states
    state_sub_ = _nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosUtils::mavStateCallback, this);
    current_odom_sub_ = _nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MavrosUtils::mavLocalOdomCallback, this);
    imu_data_sub_ = _nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &MavrosUtils::mavImuDataCallback, this);
    // 注意不要用 target_attitude ,里面的油门可能不正确
    atti_target_sub_ = _nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10, &MavrosUtils::mavAttiTargetCallback, this);

    // Note: do NOT change it to /mavros/imu/data_raw !!!
    ctrl_pub_ = _nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10); // give atti cmd

    arming_client_ = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    takeoff_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/takeoff", 1000, boost::bind(&MavrosUtils::mavTakeoffCallback,this, _1, params_parse.name));
    land_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/land", 1000, boost::bind(&MavrosUtils::mavLandCallback,this, _1, params_parse.name));


    // direct command
    // 检查 控制指令合法性 ，ctrl_output_level 为控制指令的级别
    pva_yaw_sub= _nh.subscribe<mavros_msgs::PositionTarget>("pos_sp_cmd", 10,&MavrosUtils::mavPosCtrlSpCallback, this);
    local_linear_vel_sub = _nh.subscribe<mavros_msgs::PositionTarget>("vel_sp_sub", 10, &MavrosUtils::mavLocalLinearVelCallback, this);
    atti_sp_sub = _nh.subscribe<mavros_msgs::AttitudeTarget>("atti_sp_sub", 10, &MavrosUtils::mavAttiSpCallback, this);
    rate_sp_sub = _nh.subscribe<mavros_msgs::AttitudeTarget>("rate_sp_sub", 10, &MavrosUtils::mavRateSpCallback, this);


    vision_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    // pub hover thrust
    hover_thrust_pub_ = _nh.advertise<std_msgs::Float64>("/hover_thrust", 10);

    // vrpn - vision_pose
    ros::Subscriber vrpn_pose = _nh.subscribe<geometry_msgs::PoseStamped>("vrpn_pose", 10, &MavrosUtils::mavVrpnPoseCallback,this);




    fsm.Init_FSM();
}
MavrosUtils::~MavrosUtils()
{
}
void MavrosUtils::ctrlUpdate(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, Eigen::Vector3d des_acc, double des_yaw)
{
    lin_controller.update(des_pos, des_vel, des_acc, des_yaw, 0.01);
    ctrl_cmd_.thrust = lin_controller.thrust_exp;
    if(ctrl_level == CTRL_OUTPUT_LVEVL::RATE)
        ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude,lin_controller.q_exp);
    else if(ctrl_level == CTRL_OUTPUT_LVEVL::ATTI)
        ctrl_cmd_.attitude = lin_controller.q_exp;

}
void MavrosUtils::ctrlUpdate(Eigen::Vector3d des_vel,double yaw, double dt)
{
    lin_controller.update(des_vel, 0.0, 0.01); // dt 暂时无用
    ctrl_cmd_.thrust = lin_controller.thrust_exp;
    if(ctrl_level == CTRL_OUTPUT_LVEVL::RATE)
        ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude,lin_controller.q_exp);
    else if(ctrl_level == CTRL_OUTPUT_LVEVL::ATTI)
        ctrl_cmd_.attitude = lin_controller.q_exp;
}
void MavrosUtils::setMotorsIdling()
{
    if(ctrl_level == CTRL_OUTPUT_LVEVL::RATE)
    {
        ctrl_cmd_.rate = Eigen::Vector3d::Zero();
    }
    else if(ctrl_level == CTRL_OUTPUT_LVEVL::ATTI)
    {
        ctrl_cmd_.attitude =  odometry_.attitude;
    }
    ctrl_cmd_.thrust = 0.04;
}
void MavrosUtils::setThrustZero()
{
    if(ctrl_level == CTRL_OUTPUT_LVEVL::RATE)
    {
        ctrl_cmd_.rate = Eigen::Vector3d::Zero();
    }
    else if(ctrl_level == CTRL_OUTPUT_LVEVL::ATTI)
    {
        ctrl_cmd_.attitude =  odometry_.attitude;
    }
    ctrl_cmd_.thrust = 0;
}

void MavrosUtils::sentCtrlCmd()
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::string("FCU");
    msg.thrust = ctrl_cmd_.thrust;    // body z axis
    if(ctrl_level == CTRL_OUTPUT_LVEVL::RATE)
    {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        msg.body_rate.x = ctrl_cmd_.rate(0);
        msg.body_rate.y = ctrl_cmd_.rate(1);
        msg.body_rate.z = ctrl_cmd_.rate(2);
    }
    else if(ctrl_level == CTRL_OUTPUT_LVEVL::ATTI)
    {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        msg.orientation.x = ctrl_cmd_.attitude.x();
        msg.orientation.y = ctrl_cmd_.attitude.y();
        msg.orientation.z = ctrl_cmd_.attitude.z();
        msg.orientation.w = ctrl_cmd_.attitude.w();
    }

    ctrl_pub_.publish(msg);


}

void MavrosUtils::waitConnected()
{
    // wait for FCU connection
    ros::Rate rate(2);
    ROS_INFO("FCU connecting");
    while (ros::ok() && !context_.connected)
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

bool MavrosUtils::requestOffboard()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client_.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
        return true;
    }
    return false;
}

bool MavrosUtils::requestArm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client_.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}

bool MavrosUtils::requestDisarm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;

    if (arming_client_.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}


void MavrosUtils::ctrl_loop()
{
    ros::Rate rate(params_parse.loop_rate);
    while (ros::ok())
    {
        fsm.process();
        if (fsm.now_state == CtrlFSM::INITIAL)
        {
            if (!context_.f_recv_takeoff_cmd)
            {
                ros::spinOnce();
                ros::Rate(10).sleep();
                continue;
            }
            // Request offboard and Arm
            if (!isOffboardMode() &&
                (ros::Time::now() - fsm.last_try_offboard_time > ros::Duration(5.0)))
            {
                if (!requestOffboard())
                    ROS_WARN("offboard failed,try again after 5.0 second");
                fsm.last_try_offboard_time = ros::Time::now();
            }
            else if (isOffboardMode() && !isArmed() &&
                     (ros::Time::now() - fsm.last_try_arm_time > ros::Duration(5.0)))
            {
                if (!requestArm())
                    ROS_WARN("arm failed,try again after 5.0 second");
                fsm.last_try_arm_time = ros::Time::now();
            }

            // set flag
            if (isOffboardMode())
            {
                fsm.setOffboardFlag(true);
            }
            if (isOffboardMode() && isArmed())
            {
                fsm.setArmFlag(true);
            }
            setMotorsIdling(); //设置怠速
        }
        else if (fsm.now_state == CtrlFSM::TAKEOFF)
        {
            if (fsm.last_state != CtrlFSM::TAKEOFF)
            {
                ROS_INFO("MODE: TAKEOFF");
                // odometry_ comes from /mavros/local_position/odom
                context_.last_state_position = odometry_.position;
                context_.last_state_position(2) = params_parse.takeoff_height;
                context_.last_state_attitude = odometry_.attitude;
            }
            
            Eigen::Vector3d des_takeoff_vel;
            double takeoff_ctl_gain = 3;
            des_takeoff_vel = -(odometry_.position - context_.last_state_position)*takeoff_ctl_gain; 
            des_takeoff_vel.cwiseMin(-1).cwiseMax(1);
            ctrlUpdate(des_takeoff_vel,0.0,0.01);    
            ROS_INFO_STREAM("des_takeoff_vel: " << des_takeoff_vel.transpose());

            // set auto_takeoff_height
            if (abs(odometry_.position(2) - params_parse.takeoff_height) < 0.1)
            {
                fsm.setTakeoffOverFlag(true);
                ROS_INFO("Take off done");
            }
        }
        else if (fsm.now_state == CtrlFSM::HOVER)
        {
            if (fsm.last_state != CtrlFSM::HOVER)
            {
                ROS_INFO("MODE: HOVER");
                context_.last_state_position = odometry_.position;
                context_.last_state_attitude = odometry_.attitude;
                }
            Eigen::Vector3d hover_vel;
            hover_vel = -(odometry_.position - context_.last_state_position)*3;
            hover_vel.cwiseMin(-1).cwiseMax(1);
            ctrlUpdate(hover_vel,0.0,0.01);        
        }
        else if (fsm.now_state == CtrlFSM::RUNNING)
        {
            if (fsm.last_state != CtrlFSM::RUNNING)
            {
                ROS_INFO_STREAM("MODE: RUNNING");
            }
        }

        else if (fsm.now_state == CtrlFSM::LANDING)
        {
            if (fsm.last_state != CtrlFSM::LANDING)
            {
                ROS_INFO("MODE: LAND");
                context_.last_state_attitude = odometry_.attitude;
                context_.last_state_position = odometry_.position;
                context_.landing_touchdown_start_time = ros::Time::now();
            }
            Eigen::Vector3d land_vel;
            land_vel = -(odometry_.position - context_.last_state_position)*3;
            land_vel(2) = -0.3;
            land_vel.cwiseMin(-1).cwiseMax(1);
            ctrlUpdate(land_vel,0.0,0.01);

            if(get_hover_thrust() > 0.11)
            {
                context_.landing_touchdown_start_time = ros::Time::now();
            }
            if(ros::Time::now() - context_.landing_touchdown_start_time > ros::Duration(2))
            {
                ROS_INFO("Land done");
                setThrustZero();
                requestDisarm();
            }

            // if (current_state.mode != "AUTO.LAND" &&
            //     (ros::Time::now() - fsm.last_try_offboard_time > ros::Duration(5.0)))
            // {
            //     if (!request_land())
            //         ROS_WARN("Try land cmd failed, pls try again in 5 seconds");
            //     fsm.last_try_offboard_time = ros::Time::now();
            // }
        }

        // set flag
        if (context_.f_recv_land_cmd)
        {
            fsm.setLandFlag(true);
        }
        sentCtrlCmd();

        ros::spinOnce();
        rate.sleep();
    }
}

/////////////////////////// callback
void MavrosUtils::mavPosCtrlSpCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    fsm.updateCtrlCmdTimestamp(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        local_raw_pub.publish(msg);
    }
}
void MavrosUtils::mavLocalLinearVelCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    fsm.updateCtrlCmdTimestamp(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        Eigen::Vector3d vel = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        ctrlUpdate(vel,0.0,0.01);    
    }
}

void MavrosUtils::mavRateSpCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    fsm.updateCtrlCmdTimestamp(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        ctrl_cmd_.rate(0) = msg->body_rate.x;
        ctrl_cmd_.rate(1) = msg->body_rate.y;
        ctrl_cmd_.rate(2) = msg->body_rate.z;
        ctrl_cmd_.thrust = msg->thrust;
    }
}

void MavrosUtils::mavAttiSpCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    fsm.updateCtrlCmdTimestamp(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        ctrl_cmd_.attitude.x() = msg->orientation.x;
        ctrl_cmd_.attitude.y() = msg->orientation.y;
        ctrl_cmd_.attitude.z() = msg->orientation.z;
        ctrl_cmd_.attitude.w() = msg->orientation.w;
        ctrl_cmd_.thrust = msg->thrust;
    }
}

void MavrosUtils::mavVrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{    // 创建一个新的 PoseStamped 消息
    geometry_msgs::PoseStamped modified_msg;
    modified_msg.header.stamp = ros::Time::now();
    modified_msg.header.frame_id = msg->header.frame_id;  // 保留原来的 frame_id

    // 对位置进行变换（例如，添加一个偏移量）
    modified_msg.pose.position.x = msg->pose.position.x/1000.0;  // 偏移量为 1.0 米
    modified_msg.pose.position.y = msg->pose.position.y/1000.0;
    modified_msg.pose.position.z = msg->pose.position.z/1000.0;

    // 对方向（四元数）进行变换（这里保持不变，仅作为示例）
    modified_msg.pose.orientation = msg->pose.orientation;
    vision_pose_pub.publish(modified_msg);
}




void MavrosUtils::mavTakeoffCallback(const std_msgs::String::ConstPtr& msg, std::string name)
{
    std::string received_string = msg->data;

    if (received_string.find(name) != std::string::npos) {
        context_.f_recv_takeoff_cmd = true;
    }
}

void MavrosUtils::mavLandCallback(const std_msgs::String::ConstPtr& msg, std::string name)
{
    std::string received_string = msg->data;
    if (received_string.find(name) != std::string::npos) {
        context_.f_recv_land_cmd = true;
    }
}
void MavrosUtils::mavStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    context_.armed = msg->armed;
    context_.connected = msg->connected;
    context_.mode = msg->mode;
}
void MavrosUtils::mavImuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    odometry_.acc(0) = msg->linear_acceleration.x;
    odometry_.acc(1) = msg->linear_acceleration.y;
    odometry_.acc(2) = msg->linear_acceleration.z;
        
    double dt = (msg->header.stamp - context_.last_recv_odom_time).toSec();
    context_.last_recv_odom_time = msg->header.stamp;
    if(dt > 0.1)
    {
        ROS_WARN("IMU dt is too large");
        return;
    }
    // ROS_INFO_STREAM(" ctrl_cmd_.thrust" <<  ctrl_cmd_.thrust << "dt" << dt);
    if(ctrl_cmd_.thrust > 0.1)
    {
        hover_thrust_ekf_->predict(dt); // dt
        hover_thrust_ekf_->fuseAccZ(odometry_.acc(2)-CONSTANTS_ONE_G, ctrl_cmd_.thrust);
        _hover_thrust = hover_thrust_ekf_->getHoverThrust();
        hover_thrust_ekf_->fuseAccZ(odometry_.acc(2)-CONSTANTS_ONE_G, ctrl_cmd_.thrust);
        lin_controller.set_hover_thrust(_hover_thrust);

    }
    
    // hover_thrust_ekf_->printLog();
    // _hover_thrust = 0.30;
    std_msgs::Float64 hover_thrust_msg;
    hover_thrust_msg.data = _hover_thrust;
    hover_thrust_pub_.publish(hover_thrust_msg);
    lin_controller.set_hover_thrust(_hover_thrust);
    
}
void MavrosUtils::mavLocalOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odometry_.position(0) = msg->pose.pose.position.x;
    odometry_.position(1) = msg->pose.pose.position.y;
    odometry_.position(2) = msg->pose.pose.position.z;

    odometry_.velocity(0) = msg->twist.twist.linear.x;
    odometry_.velocity(1) = msg->twist.twist.linear.y;
    odometry_.velocity(2) = msg->twist.twist.linear.z;

    odometry_.attitude.x() = msg->pose.pose.orientation.x;
    odometry_.attitude.y() = msg->pose.pose.orientation.y;
    odometry_.attitude.z() = msg->pose.pose.orientation.z;
    odometry_.attitude.w() = msg->pose.pose.orientation.w;

    odometry_.rate(0) = msg->twist.twist.angular.x;
    odometry_.rate(1) = msg->twist.twist.angular.y;
    odometry_.rate(2) = msg->twist.twist.angular.z;
    lin_controller.set_status(odometry_.position, odometry_.velocity, odometry_.rate, odometry_.attitude);

}

void MavrosUtils::mavAttiTargetCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    // ROS_INFO_STREAM("odometry_.acc(2)" << odometry_.acc(2) << "ctrl_cmd_.target_thrust"<< ctrl_cmd_.target_thrust);


    // TODO: need to check the thrust is in world frame or body frame
    // barometer cannot be turned off?
    // In /mavros/imu/data msg ,acceleration is from HIGHRES_IMU, is barometer will calibrate the acceleration?
    // odometry_.acc need transform to world frame
    // ctrl_cmd_.target_thrust need transform to world frame
    
    // odometry_.attitude world -> body， 
    //  油门 是 body 下 [0,0,ctrl_cmd_.target_thrust]
    // 转换 油门到 world frame下

    Eigen::Vector3d thrust_world = odometry_.attitude*Eigen::Vector3d(0,0,ctrl_cmd_.thrust);
    Eigen::Vector3d acc_world = odometry_.attitude*odometry_.acc;
    // hover_thrust_ekf_->fuseAccZ(acc_world(2)-CONSTANTS_ONE_G, thrust_world(2));
    // hover_thrust_ekf_->printLog();


    
}