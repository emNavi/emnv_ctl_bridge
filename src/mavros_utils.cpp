#include "ctrl_bridge/mavros_utils.hpp"
#include <std_msgs/Float64.h>
#include <iostream>
// #include "quadrotor_msgs/PositionCommand.h"

// 创建一个映射关系
std::map<std::string, CmdPubType> cmdPubMap = {
    {"ATTI", CmdPubType::ATTI},
    {"RATE", CmdPubType::RATE},
    {"POSY", CmdPubType::POSY}
};
// 创建一个映射关系
std::map<std::string, CtrlMode> ctrlModeMap = {
    {"ATTI", CtrlMode::QUAD_T},
    {"RATE", CtrlMode::RATE_T},
    {"POSY", CtrlMode::PVA_Ys}
};


MavrosUtils::MavrosUtils(ros::NodeHandle &_nh)
{
    nh = _nh;
    hover_thrust_ekf_ = new HoverThrustEkf(0.4f, 0.1f, 0.036f);

    // sub mavros states
    state_sub_ = _nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosUtils::mavStateCallback, this);
    current_odom_sub_ = _nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MavrosUtils::mavLocalOdomCallback, this);
    imu_data_sub_ = _nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &MavrosUtils::mavImuDataCallback, this);
    // 注意不要用 target_attitude ,里面的油门可能不正确
    atti_target_sub_ = _nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10, &MavrosUtils::mavAttiTargetCallback, this);
    super_target_sub = _nh.subscribe<quadrotor_msgs::PositionCommand>("/mavros/super_attitude", 100, &MavrosUtils::mavSupergetCallback, this);

    // Note: do NOT change it to /mavros/imu/data_raw !!!
    ctrl_atti_pub_ = _nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10); // give atti cmd
    ctrl_posy_pub_ = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10); // give posy cmd

    arming_client_ = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    takeoff_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/takeoff", 1000, boost::bind(&MavrosUtils::mavTakeoffCallback,this, _1, params_parse.name));
    land_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/land", 1000, boost::bind(&MavrosUtils::mavLandCallback,this, _1, params_parse.name));
    cmd_vaild_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/cmd_vaild", 1000, boost::bind(&MavrosUtils::mavCmd_vaildCallback,this, _1, params_parse.name));



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
    if(ctrl_level == CmdPubType::RATE)
        ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude,lin_controller.q_exp);
    else if(ctrl_level == CmdPubType::ATTI)
        ctrl_cmd_.attitude = lin_controller.q_exp;
    else if(ctrl_level == CmdPubType::POSY)
    {
        ctrl_cmd_.position(0) = des_pos(0);
        ctrl_cmd_.position(1) = des_pos(1);
        ctrl_cmd_.position(2) = des_pos(2);
        ctrl_cmd_.velocity(0) = des_vel(0);
        ctrl_cmd_.velocity(1) = des_vel(1);
        ctrl_cmd_.velocity(2) = des_vel(2);
        ctrl_cmd_.acceleration(0) = des_acc(0);
        ctrl_cmd_.acceleration(1) = des_acc(1);
        ctrl_cmd_.acceleration(2)= des_acc(2);
        ctrl_cmd_.yaw = des_yaw;
    }

}
void MavrosUtils::ctrlUpdate(Eigen::Vector3d des_vel,double yaw, double dt)
{
    lin_controller.update(des_vel, 0.0, 0.01); // dt 暂时无用
    ctrl_cmd_.thrust = lin_controller.thrust_exp;
    if(ctrl_level == CmdPubType::RATE)
        ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude,lin_controller.q_exp);
    else if(ctrl_level == CmdPubType::ATTI)
        ctrl_cmd_.attitude = lin_controller.q_exp;
}

void MavrosUtils::setMotorsIdling()
{
    if(ctrl_level == CmdPubType::RATE)
    {
        ctrl_cmd_.rate = Eigen::Vector3d::Zero();
    }
    else if(ctrl_level == CmdPubType::ATTI)
    {
        ctrl_cmd_.attitude =  odometry_.attitude;
    }
    ctrl_cmd_.thrust = 0.04;

    if(ctrl_level == CmdPubType::POSY)
    {
        ctrl_cmd_.position =  Eigen::Vector3d::Zero();
        ctrl_cmd_.velocity(0) =  0;
        ctrl_cmd_.velocity(1) =  0;
        ctrl_cmd_.velocity(2) =  0.1;
        ctrl_cmd_.acceleration =  Eigen::Vector3d::Zero();
        ctrl_cmd_.yaw = 0;
    }
}
void MavrosUtils::setThrustZero()
{
    if(ctrl_level == CmdPubType::RATE)
    {
        ctrl_cmd_.rate = Eigen::Vector3d::Zero();
        ctrl_cmd_.thrust = 0;

    }
    else if(ctrl_level == CmdPubType::ATTI)
    {
        ctrl_cmd_.attitude =  odometry_.attitude;
        ctrl_cmd_.thrust = 0;
    }
    else if(ctrl_level == CmdPubType::POSY)
    {
        // TODO  这里应该用当前的位置
        ctrl_cmd_.position = odometry_.position;
        ctrl_cmd_.velocity = Eigen::Vector3d::Zero();
        ctrl_cmd_.acceleration = Eigen::Vector3d::Zero();
        ctrl_cmd_.yaw = 0;
    }
}

Eigen::Vector3d super_posm, super_vel, super_acc;
double super_yaw;
int mav_now_state = 0;
void MavrosUtils::mavSupergetCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{ 
    // ctrlUpdate(msg->position)
    // ctrlUpdate(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, Eigen::Vector3d des_acc, double des_yaw);
    if(fsm.now_state == CtrlFSM::RUNNING)
    {   
        // if(msg->position == NULL)
            // ROS_INFO("空数据");
            // fsm.setFlag("takeoff_done",true);
            // ROS_INFO("Take off done");
        super_posm(0) = msg->position.x;
        super_posm(1) = msg->position.y;
        super_posm(2) = msg->position.z;
        super_vel(0) = msg->velocity.x;
        super_vel(1) = msg->velocity.y;
        super_vel(2) = msg->velocity.z;
        super_acc(0) = msg->acceleration.x;
        super_acc(1) = msg->acceleration.y;
        super_acc(2) = msg->acceleration.z;
        super_yaw = msg->yaw;
    }
}

void MavrosUtils::sentCtrlCmd()
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::string("FCU");

    mavros_msgs::PositionTarget msg_pos;
    msg_pos.header.stamp = ros::Time::now();
    // msg_pos.header.frame_id = std::string("FCU");
    msg_pos.coordinate_frame = msg_pos.FRAME_LOCAL_NED;
    mav_now_state =  fsm.now_state;//查看状态机使用

    if(ctrl_level == CmdPubType::RATE)
    {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        msg.body_rate.x = ctrl_cmd_.rate(0);
        msg.body_rate.y = ctrl_cmd_.rate(1);
        msg.body_rate.z = ctrl_cmd_.rate(2);
        msg.thrust = ctrl_cmd_.thrust;    // body z axis
        ctrl_atti_pub_.publish(msg);


    }
    else if(ctrl_level == CmdPubType::ATTI)
    {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        msg.orientation.x = ctrl_cmd_.attitude.x();
        msg.orientation.y = ctrl_cmd_.attitude.y();
        msg.orientation.z = ctrl_cmd_.attitude.z();
        msg.orientation.w = ctrl_cmd_.attitude.w();
        msg.thrust = ctrl_cmd_.thrust;    // body z axis
        ctrl_atti_pub_.publish(msg);

    }
    
    // else if((ctrl_level == CmdPubType::POSY) && (ctrl_level == CmdPubType::POSY))  
    else if(ctrl_level == CmdPubType::POSY)  
    {
        // msg_pos.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ
        //                     | mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ 
        //                     | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFX 
        //                     | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        /*重置初始位置 +  悬停*/
        // fsm.setFlag("takeoff_done",true);
        // ROS_INFO("Take off done");
        msg_pos.type_mask =  mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        msg_pos.position.x = ctrl_cmd_.position(0);
        msg_pos.position.y = ctrl_cmd_.position(1);
        msg_pos.position.z = ctrl_cmd_.position(2);
        msg_pos.velocity.x = ctrl_cmd_.velocity(0);
        msg_pos.velocity.y = ctrl_cmd_.velocity(1);
        msg_pos.velocity.z = ctrl_cmd_.velocity(2);
        msg_pos.acceleration_or_force.x = ctrl_cmd_.acceleration(0);
        msg_pos.acceleration_or_force.y = ctrl_cmd_.acceleration(1);
        msg_pos.acceleration_or_force.z = ctrl_cmd_.acceleration(2);
        // msg_pos.acceleration_or_force = Eigen::Vector3d::Zero();
        // yaw_rate；
        msg_pos.yaw = ctrl_cmd_.yaw;
        ctrl_posy_pub_.publish(msg_pos);

        // ROS_INFO("ctrl_cmd_.z = %f %f",ctrl_cmd_.position(2),msg_pos.position.z);
    }
    // ROS_INFO("ctrl_cmd_.thrust: %f", ctrl_cmd_.thrust);
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
        std::cout << ">"<< std::flush;
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

int runnig_false=0,time_flage=0;
int FSM_RESET_flage = 0;
void MavrosUtils::ctrl_loop()
{
    ros::Rate rate(params_parse.loop_rate);
    while (ros::ok())
    {
        /*简单重置状态机*/
        ros::param::get ("FSM_RESET_flage",FSM_RESET_flage);
        if(FSM_RESET_flage == 1)
        {   
            FSM_RESET_flage = 0;
            ros::param::set("FSM_RESET_flage", 0 );
            fsm.Init_FSM();
            ROS_INFO("FSM reset!!!");
        }
        fsm.process();
        if (fsm.now_state == CtrlFSM::IDLE)
        {
            if (fsm.last_state != CtrlFSM::IDLE)
            {
            }
            // ros::spinOnce();
            // continue; // 不发布任何东西
            // just wait for takeoff cmd
        }
        else if (fsm.now_state == CtrlFSM::INITIAL)
        {
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
                fsm.setFlag("offboard_done", true);
            }
            if (isOffboardMode() && isArmed())
            {
                fsm.setFlag("arm_done", true);
            }
            setMotorsIdling(); //设置怠速，important
        }
        else if (fsm.now_state == CtrlFSM::TAKEOFF)
        {
            if (fsm.last_state != CtrlFSM::TAKEOFF)
            {
                // odometry_ comes from /mavros/local_position/odom
                context_.last_state_position = odometry_.position;
                context_.last_state_position(2) = params_parse.takeoff_height;
                context_.last_state_attitude = odometry_.attitude;
            }
            Eigen::Vector3d des_takeoff_pos;
            des_takeoff_pos = context_.last_state_position;
            des_takeoff_pos(2) = params_parse.takeoff_height;
            double takeoff_ctl_gain = 3;
            Eigen::Vector3d des_takeoff_vel;

            if(ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                takeoff_ctl_gain = 3;
                des_takeoff_vel = (des_takeoff_pos-odometry_.position)*takeoff_ctl_gain; 
                des_takeoff_vel.cwiseMin(-1).cwiseMax(1);
                ctrlUpdate(des_takeoff_vel,0.0,0.01);  
                ROS_INFO_STREAM("des_takeoff_vel: " << des_takeoff_vel.transpose());
            }
            else if(ctrl_level == CmdPubType::POSY)
            {
                takeoff_ctl_gain = 3;
                des_takeoff_vel = (des_takeoff_pos-odometry_.position)*takeoff_ctl_gain; 
                des_takeoff_vel.cwiseMin(-1).cwiseMax(1);
                ctrlUpdate(des_takeoff_pos, des_takeoff_vel, Eigen::Vector3d::Zero(), 0.0);
            }
            // set auto_takeoff_height
            if (abs(odometry_.position(2) - params_parse.takeoff_height) < 0.1)
            {
                fsm.setFlag("takeoff_done",true);
                ROS_INFO("Take off done");
            }
        }
        else if (fsm.now_state == CtrlFSM::HOVER)
        {
            if (fsm.last_state != CtrlFSM::HOVER)
            {
                context_.last_state_position = odometry_.position;
                context_.last_state_attitude = odometry_.attitude;
            }
            double hover_vel_ctrl_gain;
            if(ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                hover_vel_ctrl_gain = 3;
                Eigen::Vector3d hover_vel;
                hover_vel = (context_.last_state_position-odometry_.position)*hover_vel_ctrl_gain;
                hover_vel.cwiseMin(-1).cwiseMax(1);
                // 这里需要设置为上一次的yaw 值
                ctrlUpdate(hover_vel,0.0,0.01);        
            }
            else if(ctrl_level == CmdPubType::POSY)
            {
                hover_vel_ctrl_gain = 3;
                Eigen::Vector3d hover_pos;
                hover_pos = context_.last_state_position;
                Eigen::Vector3d hover_vel;
                hover_vel = -(odometry_.position - context_.last_state_position)*3;
                hover_vel.cwiseMin(-1).cwiseMax(1);
                ctrlUpdate(hover_pos, hover_vel, Eigen::Vector3d::Zero(), 0.0);
            }

        }
        else if (fsm.now_state == CtrlFSM::RUNNING)
        {
            // exec user commnad
            if (fsm.last_state != CtrlFSM::RUNNING)
            {
                ROS_INFO("MODE: RUNNING   ctrl mode == %d ",(int8_t)ctrl_level);
                // std::cout << "ctrl mode is %d" << ctrl_level << std::endl;
            }

            if(ctrl_level == CmdPubType::POSY)
            {
                time_flage++;
                ros::param::get("runnig_false",runnig_false);//节点内param ，选择0 1 :悬停初始点, 飞

                if( runnig_false == 0 )
                {
                    Eigen::Vector3d hover_pos,hover_vel;
                    hover_pos(0) = 0.0;
                    hover_pos(1) = 0.0;
                    hover_pos(2) = 1.5;
                    ctrlUpdate(hover_pos, hover_vel, Eigen::Vector3d::Zero(), 0.0);
                    if(time_flage%200==0)
                        ROS_INFO("runnig_false==%d",runnig_false);
                }
                else if( runnig_false == 1 )
                {
                    ctrlUpdate(super_posm, super_vel, super_acc, super_yaw);
                    if(time_flage%200==0)
                        ROS_INFO("runnig_false==%d",runnig_false);
                }
                // else if( runnig_false == 2 )//故障悬停
                // {
                //     hover_vel_ctrl_gain = 3;
                //     Eigen::Vector3d hover_pos;
                //     hover_pos = context_.last_state_position;
                //     Eigen::Vector3d hover_vel;
                //     hover_vel = -(odometry_.position - context_.last_state_position)*3;
                //     hover_vel.cwiseMin(-1).cwiseMax(1);
                //     ctrlUpdate(hover_pos, hover_vel, Eigen::Vector3d::Zero(), 0.0);
                // }
            }
            
        }

        else if (fsm.now_state == CtrlFSM::LANDING)
        {
            if (fsm.last_state != CtrlFSM::LANDING)
            {
                context_.last_state_attitude = odometry_.attitude;
                context_.last_state_position = odometry_.position;
                context_.landing_touchdown_start_time = ros::Time::now();
                ROS_INFO("MODE: LAND");
            }

            if(ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                Eigen::Vector3d land_vel;
                land_vel = (context_.last_state_position-odometry_.position)*3;
                land_vel(2) = -0.3;
                land_vel.cwiseMin(-1).cwiseMax(1);
                ctrlUpdate(land_vel,0.0,0.01);
                if(get_hover_thrust() > 0.11)
                {
                    ROS_INFO("Land thrust: %f", get_hover_thrust());
                    context_.landing_touchdown_start_time = ros::Time::now();
                }
                if(ros::Time::now() - context_.landing_touchdown_start_time > ros::Duration(2))
                {
                    ROS_INFO("Land done");
                    setThrustZero();
                    requestDisarm();
                    fsm.setFlag("land_done",true);
                }
            }
            // else if(ctrl_level == CmdPubType::POSY)
            // {
            //     setThrustZero();
            //     if (context_.mode != "AUTO.LAND" &&
            //         (ros::Time::now() - fsm.last_try_offboard_time > ros::Duration(5.0)))
            //     {
            //         if (!request_land())
            //             ROS_WARN("Try land cmd failed, pls try again in 5 seconds");
            //         fsm.last_try_offboard_time = ros::Time::now();
            //     }
            // }
            // bool request_land()
            // {
            //     mavros_msgs::SetMode auto_land_mode;
            //     arm_cmd.request.value = true;
            //     auto_land_mode.request.custom_mode = "AUTO.LAND";

            //     if (set_mode_client.call(auto_land_mode) &&
            //         auto_land_mode.response.mode_sent)
            //     {
            //         ROS_INFO("Landing");
            //         return true;
            //     }
            //     return false;
            // }
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
        
        local_pvay_pub.publish(msg);
    }
}

// void MavrosUtils::mavLocalLinearVelCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
// {
//     fsm.updateCtrlCmdTimestamp(ros::Time::now());
//     if (fsm.now_state == CtrlFSM::RUNNING)
//     {
//         Eigen::Vector3d vel = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
//         ctrlUpdate(vel,0.0,0.01);    
//     }
// }

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
        ROS_INFO("Received takeoff command");
        fsm.setFlag("recv_takeoff_cmd",true);
    }
}

void MavrosUtils::mavLandCallback(const std_msgs::String::ConstPtr& msg, std::string name)
{
    std::string received_string = msg->data;
    if (received_string.find(name) != std::string::npos) {
        ROS_INFO("Received land command");
        fsm.setFlag("recv_land_cmd",true);
    }
}

void MavrosUtils::mavCmd_vaildCallback(const std_msgs::String::ConstPtr& msg, std::string name)
{
    std::string received_string = msg->data;
    if (received_string.find(name) != std::string::npos) {
        ROS_INFO("Received cmd command");
        fsm.setFlag("cmd_vaild",true);
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

int MavrosUtils::set_bridge_mode(std::string ctrl_mode_str, std::string ctrl_level_str)
{
    CmdPubType ctrl_level_enum = cmdPubMap[ctrl_level_str];
    CtrlMode ctrl_mode_enum = ctrlModeMap[ctrl_mode_str];
    if(ctrl_level_enum == CmdPubType::Unknown || ctrl_mode_enum == CtrlMode::Unknown)
    {
        // 不一定管用
        ROS_ERROR("Invalid input");
        return -1;
    }

    if(ctrl_level_enum == CmdPubType::ATTI)
    {
        if(ctrl_mode_enum == CtrlMode::QUAD_T)
        {
        }
        else if (ctrl_mode_enum == CtrlMode::RATE_T)
        {
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"ctrl_level\"");
            return -1;
        }
    }
    else if(ctrl_level_enum == CmdPubType::RATE)
    {
        if (ctrl_mode_enum == CtrlMode::RATE_T || ctrl_mode_enum == CtrlMode::QUAD_T || ctrl_mode_enum == CtrlMode::PVA_Ys)
        {
        }
        else{
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"ctrl_level\"");
            return -1;
        }
    }
    else if(ctrl_level_enum == CmdPubType::POSY)
    {
        if (ctrl_mode_enum == CtrlMode::RATE_T || ctrl_mode_enum == CtrlMode::QUAD_T)
        {
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"ctrl_level\"");
            return -1;
        }
    }
    // std::ostringstream ctrl_mode_str_scan,cmd_pub_str_scan;
    // ctrl_mode_str_scan << ctrl_mode_str;
    // cmd_pub_str_scan << ctrl_level_str;
    ROS_INFO("ctrl_mode: %s, ctrl_level: %s", ctrl_mode_str.c_str(), ctrl_level_str.c_str());
    // ROS_INFO("ctrl_mode: %s, ctrl_level: %s", ctrl_mode_str_scan.str(), cmd_pub_str_scan.str());
    // ROS_INFO("ctrl_mode==%d, ctrl_level==%d", ctrl_mode_enum, ctrl_level_enum);
    ctrl_level = ctrl_level_enum;


    // direct command
    // 检查 控制指令合法性 ，ctrl_output_level 为控制指令的级别
    if(ctrl_mode_enum == CtrlMode::QUAD_T)
    {
        user_cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("atti_sp_cmd", 10, &MavrosUtils::mavAttiSpCallback, this);

    }
    else if(ctrl_mode_enum == CtrlMode::RATE_T)
    {
        user_cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("rate_sp_sub", 10, &MavrosUtils::mavRateSpCallback, this);

    }
    else if(ctrl_mode_enum == CtrlMode::PVA_Ys)
    {
        user_cmd_sub = nh.subscribe<mavros_msgs::PositionTarget>("pos_sp_cmd", 10,&MavrosUtils::mavPosCtrlSpCallback, this);

    }
    // user_cmd_sub 

    return 0;
}
