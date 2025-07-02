#include "emnv_ctl_bridge/mavros_utils.hpp"
#include <std_msgs/Float64.h>
#include <iostream>
#include <yaml-cpp/yaml.h>

// #include "quadrotor_msgs/PositionCommand.h"

// 创建一个映射关系
std::map<std::string, CmdPubType> cmdPubMap = {
    {"ATTI", CmdPubType::ATTI},
    {"RATE", CmdPubType::RATE},
    {"POSY", CmdPubType::POSY}};
// 创建一个映射关系
std::map<std::string, CtrlMode> ctrlModeMap = {
    {"ATTI", CtrlMode::QUAD_T},
    {"RATE", CtrlMode::RATE_T},
    {"POSY", CtrlMode::PVA_Ys}};

MavrosUtils::MavrosUtils(ros::NodeHandle &_nh, ParamsParse params_parse)
{
    nh = _nh;

    // 设置模式，订阅对应控制指令
    if (set_bridge_mode(params_parse.ctrl_mode, params_parse.ctrl_out_level) < 0)
    {
        throw std::runtime_error("Failed to set bridge mode");
    }
    std::cout << "set_bridge_mode success" << std::endl;
    // 解析yaml文件
    try
    {
        YAML::Node config = YAML::LoadFile(params_parse.drone_config_path);
        Eigen::Vector3d p_gain, v_gain, a_gain;
        // linear_controller gains
        auto gain = config["linear_controller"]["gain"];
        p_gain << gain["Kpxy"].as<double>(), gain["Kpxy"].as<double>(), gain["Kpz"].as<double>();
        v_gain << gain["Kvxy"].as<double>(), gain["Kvxy"].as<double>(), gain["Kvz"].as<double>();
        a_gain << gain["Kaxy"].as<double>(), gain["Kaxy"].as<double>(), gain["Kaz"].as<double>();
        lin_controller.set_gains(p_gain, v_gain, a_gain);
        lin_controller.set_max_tile(config["linear_controller"]["max_tile_deg"].as<double>());
        auto ekf = config["hover_thrust_ekf"];
        hover_thrust_ekf_ = new HoverThrustEkf(ekf["init_hover_thrust"].as<double>(), ekf["hover_thrust_noise"].as<double>(), ekf["process_noise"].as<double>(), ekf["hover_thrust_max"].as<double>());
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR_STREAM("YAML error: " << e.what() <<" "<< params_parse.drone_config_path);
        throw std::runtime_error("Failed to load drone configuration");
    }

    // vrpn - vision_pose 动捕消息
    ros::Subscriber vrpn_pose = _nh.subscribe<geometry_msgs::PoseStamped>("vrpn_pose", 10, &MavrosUtils::mavVrpnPoseCallback, this);
    vision_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>(params_parse.ros_namespace + "/mavros/vision_pose/pose", 10);
    // local_position 消息
    if (params_parse.enable_vel_transpose_b2w)
    {
        current_odom_sub_ = _nh.subscribe<nav_msgs::Odometry>("ref_odom", 10, &MavrosUtils::mavLocalOdomCallback, this);
    }
    else
    {
        current_odom_sub_ = _nh.subscribe<nav_msgs::Odometry>("ref_odom", 10, &MavrosUtils::mavRefOdomCallback, this);
    }

    // sub mavros states
    state_sub_ = _nh.subscribe<mavros_msgs::State>(params_parse.ros_namespace + "/mavros/state", 10, &MavrosUtils::mavStateCallback, this);
    // Note: do NOT change it to /mavros/imu/data_raw !!!
    imu_data_sub_ = _nh.subscribe<sensor_msgs::Imu>(params_parse.ros_namespace + "/mavros/imu/data", 10, &MavrosUtils::mavImuDataCallback, this);
    // 注意不要用 target_attitude ,里面的油门可能不正确
    atti_target_sub_ = _nh.subscribe<mavros_msgs::AttitudeTarget>(params_parse.ros_namespace + "/mavros/setpoint_raw/attitude", 10, &MavrosUtils::mavAttiTargetCallback, this);

    ctrl_atti_pub_ = _nh.advertise<mavros_msgs::AttitudeTarget>(params_parse.ros_namespace + "/mavros/setpoint_raw/attitude", 10);
    ctrl_posy_pub_ = _nh.advertise<mavros_msgs::PositionTarget>(params_parse.ros_namespace + "/mavros/setpoint_raw/local", 10);

    arming_client_ = _nh.serviceClient<mavros_msgs::CommandBool>(params_parse.ros_namespace + "/mavros/cmd/arming");
    set_mode_client_ = _nh.serviceClient<mavros_msgs::SetMode>(params_parse.ros_namespace + "/mavros/set_mode");

    takeoff_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/takeoff", 1000, boost::bind(&MavrosUtils::mavTakeoffCallback, this, _1, params_parse.name));
    land_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/land", 1000, boost::bind(&MavrosUtils::mavLandCallback, this, _1, params_parse.name));
    cmd_vaild_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/cmd_vaild", 1000, boost::bind(&MavrosUtils::mavCmd_vaildCallback, this, _1, params_parse.name));

    bridge_status_pub = _nh.advertise<std_msgs::String>("bridge_status", 10);
    // pub hover thrust
    hover_thrust_pub_ = _nh.advertise<std_msgs::Float64>("hover_thrust", 10);

    fsm.Init_FSM();
}
MavrosUtils::~MavrosUtils()
{
}

int MavrosUtils::set_bridge_mode(std::string ctrl_mode_str, std::string ctrl_level_str)
{
    CmdPubType ctrl_level_enum = cmdPubMap[ctrl_level_str];
    CtrlMode ctrl_mode_enum = ctrlModeMap[ctrl_mode_str];
    if (ctrl_level_enum == CmdPubType::Unknown || ctrl_mode_enum == CtrlMode::Unknown)
    {
        ROS_ERROR("Invalid input");
        return -1;
    }

    if (ctrl_level_enum == CmdPubType::ATTI)
    {
        if (ctrl_mode_enum == CtrlMode::QUAD_T)
        {
        }
        else if (ctrl_mode_enum == CtrlMode::RATE_T)
        {
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"ctrl_level\"");
            return -1;
        }
    }
    else if (ctrl_level_enum == CmdPubType::RATE)
    {
        if (ctrl_mode_enum == CtrlMode::RATE_T || ctrl_mode_enum == CtrlMode::QUAD_T || ctrl_mode_enum == CtrlMode::PVA_Ys)
        {
        }
        else
        {
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"ctrl_level\"");
            return -1;
        }
    }
    else if (ctrl_level_enum == CmdPubType::POSY)
    {
        if (ctrl_mode_enum == CtrlMode::RATE_T || ctrl_mode_enum == CtrlMode::QUAD_T)
        {
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"ctrl_level\"");
            return -1;
        }
    }
    ROS_INFO("ctrl_mode: %s, ctrl_level: %s", ctrl_mode_str.c_str(), ctrl_level_str.c_str());
    ctrl_level = ctrl_level_enum;

    // direct command
    // 检查 控制指令合法性 ，ctrl_output_level 为控制指令的级别
    if (ctrl_mode_enum == CtrlMode::QUAD_T)
    {
        user_cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("atti_sp_cmd", 10, &MavrosUtils::mavAttiSpCallback, this);
    }
    else if (ctrl_mode_enum == CtrlMode::RATE_T)
    {
        user_cmd_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("rate_sp_cmd", 10, &MavrosUtils::mavRateSpCallback, this);
    }
    else if (ctrl_mode_enum == CtrlMode::PVA_Ys)
    {
        user_cmd_sub = nh.subscribe<emnv_ctl_bridge::PvayCommand>("pos_sp_cmd", 10, &MavrosUtils::mavPosCtrlSpCallback, this);
        // target_cmd
    }
    // user_cmd_sub

    return 0;
}

void MavrosUtils::ctrlUpdate(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, Eigen::Vector3d des_acc, double des_yaw)
{

    lin_controller.update(des_pos, des_vel, des_acc, des_yaw, 0.01);
    ctrl_cmd_.thrust = lin_controller.thrust_exp;
    if (ctrl_level == CmdPubType::RATE)
        ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude, lin_controller.q_exp);
    else if (ctrl_level == CmdPubType::ATTI)
        ctrl_cmd_.attitude = lin_controller.q_exp;
    else if (ctrl_level == CmdPubType::POSY)
    {
        ctrl_cmd_.position(0) = des_pos(0);
        ctrl_cmd_.position(1) = des_pos(1);
        ctrl_cmd_.position(2) = des_pos(2);
        ctrl_cmd_.velocity(0) = des_vel(0);
        ctrl_cmd_.velocity(1) = des_vel(1);
        ctrl_cmd_.velocity(2) = des_vel(2);
        ctrl_cmd_.acceleration(0) = des_acc(0);
        ctrl_cmd_.acceleration(1) = des_acc(1);
        ctrl_cmd_.acceleration(2) = des_acc(2);
        ctrl_cmd_.yaw = des_yaw;
    }
}
void MavrosUtils::ctrlUpdate(Eigen::Vector3d des_vel, double des_yaw, double dt)
{
    Eigen::Vector3d des_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d des_acc = Eigen::Vector3d::Zero();
    lin_controller.update(des_pos, des_vel, des_acc, des_yaw, 0.01);
    ctrl_cmd_.thrust = lin_controller.thrust_exp;
    if (ctrl_level == CmdPubType::RATE)
        ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude, lin_controller.q_exp);
    else if (ctrl_level == CmdPubType::ATTI)
        ctrl_cmd_.attitude = lin_controller.q_exp;
}

void MavrosUtils::setMotorsIdling()
{
    if (ctrl_level == CmdPubType::RATE)
    {
        ctrl_cmd_.rate = Eigen::Vector3d::Zero();
    }
    else if (ctrl_level == CmdPubType::ATTI)
    {
        ctrl_cmd_.attitude = odometry_.attitude;
    }
    ctrl_cmd_.thrust = 0.04;
    // 发布位置指令无需 设置怠速
}
void MavrosUtils::setThrustZero()
{
    if (ctrl_level == CmdPubType::RATE)
    {
        ctrl_cmd_.rate = Eigen::Vector3d::Zero();
        ctrl_cmd_.thrust = 0;
    }
    else if (ctrl_level == CmdPubType::ATTI)
    {
        ctrl_cmd_.attitude = odometry_.attitude;
        ctrl_cmd_.thrust = 0;
    }
}

int type_mask_flage = 0;
void MavrosUtils::sentCtrlCmd()
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::string("FCU");

    mavros_msgs::PositionTarget msg_pos;
    msg_pos.header.stamp = ros::Time::now();
    // msg_pos.header.frame_id = std::string("FCU");
    msg_pos.coordinate_frame = msg_pos.FRAME_LOCAL_NED;

    if (ctrl_level == CmdPubType::RATE)
    {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        msg.body_rate.x = ctrl_cmd_.rate(0);
        msg.body_rate.y = ctrl_cmd_.rate(1);
        msg.body_rate.z = ctrl_cmd_.rate(2);
        msg.thrust = ctrl_cmd_.thrust; // body z axis
        ctrl_atti_pub_.publish(msg);
    }
    else if (ctrl_level == CmdPubType::ATTI)
    {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        msg.orientation.x = ctrl_cmd_.attitude.x();
        msg.orientation.y = ctrl_cmd_.attitude.y();
        msg.orientation.z = ctrl_cmd_.attitude.z();
        msg.orientation.w = ctrl_cmd_.attitude.w();
        msg.thrust = ctrl_cmd_.thrust; // body z axis
        ctrl_atti_pub_.publish(msg);
    }
    else if (ctrl_level == CmdPubType::POSY)
    {
        // TODO Mask
        msg_pos.position.x = ctrl_cmd_.position(0);
        msg_pos.position.y = ctrl_cmd_.position(1);
        msg_pos.position.z = ctrl_cmd_.position(2);
        msg_pos.velocity.x = ctrl_cmd_.velocity(0);
        msg_pos.velocity.y = ctrl_cmd_.velocity(1);
        msg_pos.velocity.z = ctrl_cmd_.velocity(2);
        msg_pos.acceleration_or_force.x = ctrl_cmd_.acceleration(0);
        msg_pos.acceleration_or_force.y = ctrl_cmd_.acceleration(1);
        msg_pos.acceleration_or_force.z = ctrl_cmd_.acceleration(2);

        msg_pos.yaw = ctrl_cmd_.yaw;
        ctrl_posy_pub_.publish(msg_pos);
    }
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
        std::cout << ">" << std::flush;
    }
    std::cout << std::endl;
    ROS_INFO("FCU connected");

    // TODO:send a few setpoints before starting
    // 对于 /setpoint_raw/attitude 不需要也可以
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
    // ros::Rate rate( (double)params_parse.loop_rate );
    ros::Rate rate((double)params_parse.loop_rate);
    ros::Time last_fsm_status_pub_time = ros::Time::now();
    while (ros::ok())
    {
        fsm.process();
        if (last_fsm_status_pub_time + ros::Duration(1.0) < ros::Time::now())
        {
            last_fsm_status_pub_time = ros::Time::now();
            std_msgs::String bridge_status_msg;
            bridge_status_msg.data = fsm.getStatusMsg();
            bridge_status_pub.publish(bridge_status_msg);
        }

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
            setMotorsIdling(); // 设置怠速，important
        }
        else if (fsm.now_state == CtrlFSM::TAKEOFF)
        {
            if (fsm.last_state != CtrlFSM::TAKEOFF)
            {
                lin_controller.setCtrlMask(LinearControl::CTRL_MASK::VEL);
                lin_controller.smooth_move_init();
                // odometry_ comes from /mavros/local_position/odom
                context_.last_state_position = odometry_.position;
                context_.last_state_position(2) = params_parse.takeoff_height;
                context_.last_state_attitude = odometry_.attitude;
                context_.last_state_yaw = MyMath::fromQuaternion2yaw(odometry_.attitude); // 获取当前的yaw角
            }
            Eigen::Vector3d des_takeoff_pos;
            des_takeoff_pos = context_.last_state_position;
            des_takeoff_pos(2) = params_parse.takeoff_height;

            if (ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                lin_controller.smooth_move(des_takeoff_pos, 0.5, context_.last_state_yaw, 0.01);
                ctrl_cmd_.thrust = lin_controller.thrust_exp;
                if (ctrl_level == CmdPubType::RATE)
                    ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude, lin_controller.q_exp);
                else if (ctrl_level == CmdPubType::ATTI)
                    ctrl_cmd_.attitude = lin_controller.q_exp;
            }
            // else if(ctrl_level == CmdPubType::POSY)
            // {
            //     takeoff_ctl_gain = 3;
            //     des_takeoff_vel(0) = 0;
            //     des_takeoff_vel(1) = 0;
            //     des_takeoff_vel(2) = (des_takeoff_pos(2)-odometry_.position(2))*takeoff_ctl_gain;
            //     des_takeoff_vel.cwiseMin(-1).cwiseMax(1);

            //     ctrlUpdate(des_takeoff_pos, des_takeoff_vel, Eigen::Vector3d::Zero(), 0.0);
            // }
            // set auto_takeoff_height
            if (abs(odometry_.position(2) - params_parse.takeoff_height) < 0.1)
            {
                fsm.setFlag("takeoff_done", true);
                ROS_INFO("Take off done");
            }
        }
        else if (fsm.now_state == CtrlFSM::HOVER)
        {
            if (fsm.last_state != CtrlFSM::HOVER)
            {
                lin_controller.setCtrlMask(LinearControl::CTRL_MASK::POSI | LinearControl::CTRL_MASK::VEL | LinearControl::CTRL_MASK::ACC);
                context_.last_state_position = odometry_.position;
                context_.last_state_attitude = odometry_.attitude;
                context_.last_state_yaw = MyMath::fromQuaternion2yaw(odometry_.attitude); // 获取当前的yaw角
            }
            double hover_vel_ctrl_gain;
            if (ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                ctrlUpdate(context_.last_state_position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), context_.last_state_yaw);
            }
            else if (ctrl_level == CmdPubType::POSY)
            {
            }
        }
        else if (fsm.now_state == CtrlFSM::RUNNING)
        {
            // exec user commnad
            if (fsm.last_state != CtrlFSM::RUNNING)
            {
                lin_controller.setCtrlMask(LinearControl::CTRL_MASK::POSI | LinearControl::CTRL_MASK::VEL | LinearControl::CTRL_MASK::ACC);
                ROS_INFO("MODE: RUNNING   ctrl mode == %d ", (int8_t)ctrl_level);
                // std::cout << "ctrl mode is %d" << ctrl_level << std::endl;
                // super_posm(0) = 0;
                // super_posm(1) = 0;
                // super_posm(2) = params_parse.takeoff_height;
            }
            if (ctrl_level == CmdPubType::POSY)
            {
            }
            else if (ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                ctrlUpdate(ctrl_cmd_.position, ctrl_cmd_.velocity, ctrl_cmd_.acceleration, ctrl_cmd_.yaw); // dt
            }
        }

        else if (fsm.now_state == CtrlFSM::LANDING)
        {
            if (fsm.last_state != CtrlFSM::LANDING)
            {
                context_.last_state_attitude = odometry_.attitude;
                context_.last_state_position = odometry_.position;
                context_.last_state_yaw = MyMath::fromQuaternion2yaw(odometry_.attitude); // 获取当前的yaw角
                lin_controller.setCtrlMask(LinearControl::CTRL_MASK::VEL);
                // lin_controller.smooth_move_init();

                context_.landing_touchdown_start_time = ros::Time::now();
                ROS_INFO("MODE: LAND");
            }

            if (ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                Eigen::Vector3d land_vel;
                land_vel = (context_.last_state_position - odometry_.position) * 3;
                land_vel(2) = -0.5;
                land_vel.cwiseMin(-1).cwiseMax(1);
                ctrlUpdate(land_vel, context_.last_state_yaw, 0.01);
                if (get_hover_thrust() > 0.11 && odometry_.velocity(2) < -0.1)
                {
                    // ROS_INFO("Land thrust: %f", get_hover_thrust());
                    context_.landing_touchdown_start_time = ros::Time::now();
                }
                
                if (ros::Time::now() - context_.landing_touchdown_start_time > ros::Duration(2))
                {
                    ROS_INFO("Land done");
                    setThrustZero();
                    requestDisarm();
                    fsm.setFlag("land_done", true);
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
void MavrosUtils::mavPosCtrlSpCallback(const emnv_ctl_bridge::PvayCommand::ConstPtr &msg)
{
    fsm.updateCtrlCmdTimestamp(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        ctrl_cmd_.position(0) = msg->position.x;
        ctrl_cmd_.position(1) = msg->position.y;
        ctrl_cmd_.position(2) = msg->position.z;
        ctrl_cmd_.velocity(0) = msg->velocity.x;
        ctrl_cmd_.velocity(1) = msg->velocity.y;
        ctrl_cmd_.velocity(2) = msg->velocity.z;
        ctrl_cmd_.acceleration(0) = msg->acceleration.x;
        ctrl_cmd_.acceleration(1) = msg->acceleration.y;
        ctrl_cmd_.acceleration(2) = msg->acceleration.z;
        ctrl_cmd_.yaw = msg->yaw;
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
{ // 创建一个新的 PoseStamped 消息
    geometry_msgs::PoseStamped modified_msg;
    modified_msg.header.stamp = ros::Time::now();
    modified_msg.header.frame_id = msg->header.frame_id; // 保留原来的 frame_id

    // 对位置进行变换（例如，添加一个偏移量）
    modified_msg.pose.position.x = msg->pose.position.x / 1000.0; // 偏移量为 1.0 米
    modified_msg.pose.position.y = msg->pose.position.y / 1000.0;
    modified_msg.pose.position.z = msg->pose.position.z / 1000.0;

    // 对方向（四元数）进行变换（这里保持不变，仅作为示例）
    modified_msg.pose.orientation = msg->pose.orientation;
    vision_pose_pub.publish(modified_msg);
}

void MavrosUtils::mavTakeoffCallback(const std_msgs::String::ConstPtr &msg, std::string name)
{
    std::string received_string = msg->data;
    if (received_string.find(name) != std::string::npos)
    {
        ROS_INFO("Received takeoff command");
        fsm.setFlag("recv_takeoff_cmd", true);
        fsm.setFlag("recv_land_cmd", false); // 清除 land cmd
    }
}

void MavrosUtils::mavLandCallback(const std_msgs::String::ConstPtr &msg, std::string name)
{
    std::string received_string = msg->data;
    if (received_string.find(name) != std::string::npos)
    {
        ROS_INFO("Received land command");

        fsm.setFlag("recv_land_cmd", true);
    }
}

void MavrosUtils::mavCmd_vaildCallback(const std_msgs::String::ConstPtr &msg, std::string name)
{
    std::string received_string = msg->data;
    if (received_string.find(name) != std::string::npos)
    {
        ROS_INFO("Received cmd command");
        fsm.setFlag("cmd_vaild", true);
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
    Eigen::Vector3d base_link_acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    odometry_.acc = odometry_.attitude * base_link_acc; //

    double dt = (msg->header.stamp - context_.last_recv_odom_time).toSec();
    context_.last_recv_odom_time = msg->header.stamp;
    if (dt > 0.1)
    {
        ROS_WARN("IMU dt is too large");
        return;
    }
    if (ctrl_cmd_.thrust > 0.1)
    {
        hover_thrust_ekf_->predict(dt); // dt
        hover_thrust_ekf_->fuseAccZ(odometry_.acc(2) - CONSTANTS_ONE_G, ctrl_cmd_.thrust);
        _hover_thrust = hover_thrust_ekf_->getHoverThrust();
    }
    // hover_thrust_ekf_->printLog();


    std_msgs::Float64 hover_thrust_msg;
    hover_thrust_msg.data = _hover_thrust;
    hover_thrust_pub_.publish(hover_thrust_msg);
    lin_controller.set_hover_thrust(_hover_thrust);
}

void MavrosUtils::mavRefOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    fsm.updateOdomTimestamp(msg->header.stamp);
    odometry_.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odometry_.rate = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    odometry_.attitude.x() = msg->pose.pose.orientation.x;
    odometry_.attitude.y() = msg->pose.pose.orientation.y;
    odometry_.attitude.z() = msg->pose.pose.orientation.z;
    odometry_.attitude.w() = msg->pose.pose.orientation.w;

    odometry_.velocity = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    lin_controller.set_status(odometry_.position, odometry_.velocity, odometry_.rate, odometry_.attitude);
}
void MavrosUtils::mavLocalOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    fsm.updateOdomTimestamp(msg->header.stamp);
    odometry_.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odometry_.rate = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    odometry_.attitude.x() = msg->pose.pose.orientation.x;
    odometry_.attitude.y() = msg->pose.pose.orientation.y;
    odometry_.attitude.z() = msg->pose.pose.orientation.z;
    odometry_.attitude.w() = msg->pose.pose.orientation.w;

    Eigen::Vector3d base_link_vel = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    odometry_.velocity = odometry_.attitude * base_link_vel; // body frame to world frame
    lin_controller.set_status(odometry_.position, odometry_.velocity, odometry_.rate, odometry_.attitude);
}

void MavrosUtils::mavAttiTargetCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    Eigen::Vector3d thrust_world = odometry_.attitude * Eigen::Vector3d(0, 0, ctrl_cmd_.thrust);
    Eigen::Vector3d acc_world = odometry_.attitude * odometry_.acc;
}
