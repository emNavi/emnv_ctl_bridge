#include "emnv_ctl_bridge/mavros_utils.hpp"
#include <std_msgs/Float64.h>
#include <iostream>
#include <yaml-cpp/yaml.h>

// #include "quadrotor_msgs/PositionCommand.h"

// 创建一个映射关系
CmdPubType getCmdPubType(const std::string& key) {
    static const std::map<std::string, CmdPubType> cmdPubMap = {
        {"ATTI", CmdPubType::ATTI},
        {"RATE", CmdPubType::RATE},
        {"POSY", CmdPubType::POSY}
    };
    auto it = cmdPubMap.find(key);
    return (it != cmdPubMap.end()) ? it->second : CmdPubType::Unknown;
}

CtrlMode getCtrlMode(const std::string& key) {
    static const std::map<std::string, CtrlMode> ctrlModeMap = {
        {"ATTI", CtrlMode::QUAD_T},
        {"RATE", CtrlMode::RATE_T},
        {"POSY", CtrlMode::PVA_Ys}
    };
    auto it = ctrlModeMap.find(key);
    return (it != ctrlModeMap.end()) ? it->second : CtrlMode::Unknown;
}


MavrosUtils::MavrosUtils(ros::NodeHandle &_nh, ParamsParse params_parse)
{
    nh = _nh;
    params_parse_ = params_parse;
    enable_imu_dt_check_f = params_parse_.enable_imu_dt_check;
    // 设置模式，订阅对应控制指令
    if (set_bridge_mode(params_parse_.ctrl_mode, params_parse_.ctrl_pub_level) < 0)
    {
        throw std::runtime_error("Failed to set bridge mode");
    }
    std::cout << "set_bridge_mode success" << std::endl;
    if(!updateCtrlParams(false))
        throw std::runtime_error("Failed to load drone configuration");
    if(params_parse_.use_vrpn_convert)
    {
        ROS_INFO("Using vrpn_convert for motion capture data");
        // vrpn - vision_pose 动捕消息
        vrpn_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>("vrpn_pose", 10, &MavrosUtils::mavVrpnPoseCallback, this);
        vision_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("vrpn_convert_pose", 10);
    }

    acc_fusion_ = new AccelerationFusion(50.0, 3.0, 10.0, 1.0, 0.7, 0.3);
    // params_parse.ros_namespace + "/mavros/vision_pose/pose"
    // local_position 消息
    if (params_parse_.ref_odom_topic.find("mavros/local_position") != std::string::npos) {
        current_odom_sub_ = _nh.subscribe<nav_msgs::Odometry>(params_parse.ros_namespace + params_parse_.ref_odom_topic, 10, &MavrosUtils::mavLocalOdomCallback, this);
    }
    else
    {
        current_odom_sub_ = _nh.subscribe<nav_msgs::Odometry>(params_parse.ros_namespace + params_parse_.ref_odom_topic, 10, &MavrosUtils::mavRefOdomCallback, this);
    }

    world_odom_pub_ = _nh.advertise<nav_msgs::Odometry>("world_odom", 10);

    // sub mavros states
    state_sub_ = _nh.subscribe<mavros_msgs::State>(params_parse.ros_namespace + "/mavros/state", 10, &MavrosUtils::mavStateCallback, this);
    extended_state_sub_ = _nh.subscribe<mavros_msgs::ExtendedState>(params_parse.ros_namespace + "/mavros/extended_state", 10, &MavrosUtils::mavExtendedStateCallback, this);
    // Note: do NOT change it to /mavros/imu/data_raw !!!
    imu_data_sub_ = _nh.subscribe<sensor_msgs::Imu>(params_parse.ros_namespace + "/mavros/imu/data", 10, &MavrosUtils::mavImuDataCallback, this);

    ctrl_atti_pub_ = _nh.advertise<mavros_msgs::AttitudeTarget>(params_parse_.ros_namespace + "/mavros/setpoint_raw/attitude", 10);
    ctrl_posy_pub_ = _nh.advertise<mavros_msgs::PositionTarget>(params_parse_.ros_namespace + "/mavros/setpoint_raw/local", 10);

    arming_client_ = _nh.serviceClient<mavros_msgs::CommandBool>(params_parse_.ros_namespace + "/mavros/cmd/arming");
    set_mode_client_ = _nh.serviceClient<mavros_msgs::SetMode>(params_parse_.ros_namespace + "/mavros/set_mode");

    takeoff_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/takeoff", 1000, boost::bind(&MavrosUtils::mavTakeoffCallback, this, _1, params_parse_.drone_id));
    land_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/land", 1000, boost::bind(&MavrosUtils::mavLandCallback, this, _1, params_parse_.drone_id));
    cmd_vaild_sub = _nh.subscribe<std_msgs::String>("/emnavi_cmd/cmd_vaild", 1000, boost::bind(&MavrosUtils::mavCmd_vaildCallback, this, _1, params_parse_.drone_id));
    update_ctrl_params_sub = _nh.subscribe<std_msgs::Empty>("/emnavi_cmd/update_ctrl_params", 1, &MavrosUtils::mavUpdateCtrlParamsCallback, this);

    bridge_status_pub = _nh.advertise<std_msgs::String>("bridge_status", 10);
    // pub hover thrust
    hover_thrust_pub_ = _nh.advertise<std_msgs::Float64>("hover_thrust", 10);

    fsm.Init_FSM(params_parse_.enable_odom_timeout_check);
}
MavrosUtils::~MavrosUtils()
{
}
bool MavrosUtils::updateCtrlParams(bool is_reload_yaml)
{
    try
    {
        YAML::Node config = YAML::LoadFile(params_parse_.drone_config_path);
        Eigen::Vector3d p_gain, v_gain, a_gain;
        // linear_controller gains
        auto lin_gain = config["linear_controller"]["gain"];
        p_gain << lin_gain["Kpxy"].as<double>(), lin_gain["Kpxy"].as<double>(), lin_gain["Kpz"].as<double>();
        v_gain << lin_gain["Kvxy"].as<double>(), lin_gain["Kvxy"].as<double>(), lin_gain["Kvz"].as<double>();
        a_gain << lin_gain["Kaxy"].as<double>(), lin_gain["Kaxy"].as<double>(), lin_gain["Kaz"].as<double>();
        lin_controller.set_gains(p_gain, v_gain, a_gain);
        lin_controller.set_max_tile(config["linear_controller"]["max_tile_deg"].as<double>());


        Eigen::Vector3d extra_v_gain, extra_a_gain;
        auto extra_gain = config["px4_internal_ctrl_extra_params"]["gain"];
        extra_v_gain << extra_gain["kvxy_extra"].as<double>(), extra_gain["kvxy_extra"].as<double>(), extra_gain["kvz_extra"].as<double>();
        extra_a_gain << extra_gain["kaxy_extra"].as<double>(), extra_gain["kaxy_extra"].as<double>(), extra_gain["kaz_extra"].as<double>();
        lin_controller.setExtraGain(extra_v_gain, extra_a_gain);
        auto atti_gain = config["atti_controller"]["gain"];
        atti_controller_.set_pid_params(Eigen::Vector3d(atti_gain["Kx"].as<double>(), atti_gain["Ky"].as<double>(), atti_gain["Kz"].as<double>()));
        if(!is_reload_yaml)
        {
            // just init once
            auto ekf = config["hover_thrust_ekf"];
            hover_thrust_ekf_ = new HoverThrustEkf(ekf["init_hover_thrust"].as<double>(), ekf["hover_thrust_noise"].as<double>(), ekf["process_noise"].as<double>(), ekf["hover_thrust_max"].as<double>());
        }
        return true;
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR_STREAM("YAML error: " << e.what() <<" "<< params_parse_.drone_config_path);
        return false;
    }
}


void MavrosUtils::mavUpdateCtrlParamsCallback(const std_msgs::Empty::ConstPtr &msg)
{
    // 更新控制参数
    ROS_INFO("Updating control parameters...");
    // TODO: 实现具体的更新逻辑
    if(updateCtrlParams(true))
        ROS_INFO("Control parameters updated successfully.");
    else
        ROS_WARN("Failed to update control parameters.");
}

int MavrosUtils::set_bridge_mode(std::string ctrl_mode_str, std::string ctrl_level_str)
{
    // 检查 控制指令合法性 ，ctrl_output_level 为控制指令的级别

    CmdPubType ctrl_level_enum = getCmdPubType(ctrl_level_str);
    CtrlMode ctrl_mode_enum = getCtrlMode(ctrl_mode_str);

    if (ctrl_level_enum == CmdPubType::Unknown || ctrl_mode_enum == CtrlMode::Unknown) {
        ROS_ERROR("Invalid ctrl_level or ctrl_mode input");
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
    ctrl_mode = ctrl_mode_enum;

    // 为了防止混淆，使用三个不同的topic接收控制指令
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

void MavrosUtils::ctrlUpdate(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, Eigen::Vector3d des_acc, double des_yaw, double dt)
{
    if(ctrl_mode == CtrlMode::PVA_Ys)
    {
        lin_controller.update(des_pos, des_vel, des_acc, des_yaw, 0.01);
        ctrl_cmd_.thrust = lin_controller.thrust_exp;
        if (ctrl_level == CmdPubType::RATE)
            ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude, lin_controller.q_exp);
        else if (ctrl_level == CmdPubType::ATTI)
            ctrl_cmd_.attitude = lin_controller.q_exp;
    }
    else if (ctrl_mode == CtrlMode::QUAD_T)
    {
        if (ctrl_level == CmdPubType::RATE)
            ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude, lin_controller.q_exp);
    }
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
    ros::Rate rate((double)params_parse_.loop_rate);
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
                lin_controller.setCtrlMask(LinearControl::CTRL_MASK::POSI|LinearControl::CTRL_MASK::VEL| LinearControl::CTRL_MASK::ACC);
                lin_controller.smooth_move_init();
                // odometry_ comes from /mavros/local_position/odom
                context_.last_state_position = odometry_.position;
                context_.last_state_position(2) = params_parse_.takeoff_height;
                context_.last_state_attitude = odometry_.attitude;
                context_.last_state_yaw = MyMath::fromQuaternion2yaw(odometry_.attitude); // 获取当前的yaw角
            }
            Eigen::Vector3d des_takeoff_pos;
            des_takeoff_pos = context_.last_state_position;
            des_takeoff_pos(2) = params_parse_.takeoff_height;

            if (ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                lin_controller.smooth_move(des_takeoff_pos, 1.0, context_.last_state_yaw, 0.01);
                ctrl_cmd_.thrust = lin_controller.thrust_exp;
                if (ctrl_level == CmdPubType::RATE)
                    ctrl_cmd_.rate = atti_controller_.update(odometry_.attitude, lin_controller.q_exp);
                else if (ctrl_level == CmdPubType::ATTI)
                    ctrl_cmd_.attitude = lin_controller.q_exp;
            }
            else if(ctrl_level == CmdPubType::POSY)
            {
                // TODO 平滑
                double takeoff_vel = 0.5; // m/s

                Eigen::Vector3d direction = des_takeoff_pos - ctrl_cmd_.position;
                double dist = direction.norm();
                if (dist > 1e-6) {
                    direction.normalize();
                    double step = takeoff_vel * 0.01; // 0.01为dt
                    if (dist > step) {
                        ctrl_cmd_.position += direction * step;
                    } else {
                        ctrl_cmd_.position = des_takeoff_pos;
                    }
                }
                ctrl_cmd_.velocity = Eigen::Vector3d::Zero();
                ctrl_cmd_.acceleration = Eigen::Vector3d::Zero();
                ctrl_cmd_.yaw = context_.last_state_yaw;
            }

            // set auto_takeoff_height
            if (abs(odometry_.position(2) - params_parse_.takeoff_height) < 0.1)
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
                // context_.last_state_yaw = MyMath::fromQuaternion2yaw(odometry_.attitude); // 获取当前的yaw角
                context_.last_state_yaw = 0 ;
            }
            double hover_vel_ctrl_gain;
            if (ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                ctrlUpdate(context_.last_state_position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), context_.last_state_yaw,0.01);
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
                ROS_INFO("MODE: RUNNING ctrl mode == %d ", (int8_t)ctrl_level);
            }
            if (ctrl_level == CmdPubType::POSY)
            {
                // 如果是位置控制
                // position指令不变
                // velocity
                Eigen::Vector3d pos_err = ctrl_cmd_.position - odometry_.position;

                Eigen::Vector3d extra_vel_gain;
                Eigen::Vector3d extra_acc_gain;
                ctrl_cmd_.velocity = ctrl_cmd_.feedforward_vel; // 位置环 P 控制
                ctrl_cmd_.acceleration = ctrl_cmd_.feedforward_acc; // 速度环 P 控制
                // lin_controller.getExtraGain(extra_vel_gain, extra_acc_gain);
                
                // ctrl_cmd_.velocity = extra_vel_gain.asDiagonal() * pos_err + ctrl_cmd_.feedforward_vel; // 位置环 P 控制
                // Eigen::Vector3d vel_err = ctrl_cmd_.velocity - odometry_.velocity;
                // ctrl_cmd_.acceleration = extra_acc_gain.asDiagonal() * vel_err + ctrl_cmd_.feedforward_acc; // 速度环 P 控制
                // ctrl_cmd_.velocity = Eigen::Vector3d::Zero();
                // ctrl_cmd_.acceleration = Eigen::Vector3d::Zero();
                // Eigen::Vector3d world_vel = ctrl_cmd_.feedforward_vel;
                // ctrl_cmd_.velocity  = odometry_.attitude.inverse() * world_vel; // body frame to world frame
                // ctrl_cmd_.acceleration = Eigen::Vector3d::Zero();

            }
            else if (ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                ctrlUpdate(ctrl_cmd_.position, ctrl_cmd_.feedforward_vel, ctrl_cmd_.feedforward_acc, ctrl_cmd_.yaw, 0.01); // dt
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
            double target_land_vel = 0.5; // m/s

            if (ctrl_level == CmdPubType::RATE || ctrl_level == CmdPubType::ATTI)
            {
                Eigen::Vector3d land_vel;
                land_vel = (context_.last_state_position - odometry_.position) * 3;
                land_vel(2) = -target_land_vel;
                land_vel.cwiseMin(1).cwiseMax(-1);
                Eigen::Vector3d des_pos = Eigen::Vector3d::Zero();
                Eigen::Vector3d des_acc = Eigen::Vector3d::Zero();
                ctrlUpdate(des_pos,land_vel,des_acc, context_.last_state_yaw, 0.01);
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
            else if(ctrl_level == CmdPubType::POSY)
            {
                ctrl_cmd_.position(0) = context_.last_state_position(0);
                ctrl_cmd_.position(1) = context_.last_state_position(1);
                ctrl_cmd_.position(2) = ctrl_cmd_.position(2) - (1/(double)params_parse_.loop_rate)*target_land_vel;
                
                ctrl_cmd_.velocity = Eigen::Vector3d::Zero();
                ctrl_cmd_.acceleration = Eigen::Vector3d::Zero();
                ctrl_cmd_.yaw = context_.last_state_yaw;
                ctrl_cmd_.feedforward_acc = Eigen::Vector3d::Zero();
                ctrl_cmd_.feedforward_vel = Eigen::Vector3d::Zero();
                                
                if (context_.landed_state== false && context_.check_vel_landed(odometry_.position(2),ros::Time::now())== false )
                {
                    // 不满足着陆条件就重置计时
                    // ROS_INFO("Land thrust: %f", get_hover_thrust());
                    context_.landing_touchdown_start_time = ros::Time::now();
                }
                if (ros::Time::now() - context_.landing_touchdown_start_time > ros::Duration(2))
                {
                    double wait_arm_time = 5.0; // s
                    ctrl_level = CmdPubType::RATE; // 切换到姿态控制
                    while (ros::ok() && isArmed() && wait_arm_time > 0)
                    {
                        setThrustZero();
                        sentCtrlCmd();
                        // requestDisarm();
                        ros::Duration(0.05).sleep();
                        ros::spinOnce();
                        wait_arm_time -= 0.05;
                    }           
                    ROS_INFO("Land done; disarm");
                    while ( isArmed() )
                    {
                        requestDisarm();
                        ROS_WARN("Disarm failed, try again");
                        ros::Duration(1).sleep();
                        ros::spinOnce();
                    }
                    ctrl_level = CmdPubType::POSY; // 切换回位置控制
                    // check to stabilize
                    fsm.setFlag("land_done", true);
                }
            }
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
    if (fsm.now_state == CtrlFSM::RUNNING | fsm.now_state == CtrlFSM::HOVER)
    {
        ctrl_cmd_.position(0) = msg->position.x;
        ctrl_cmd_.position(1) = msg->position.y;
        ctrl_cmd_.position(2) = msg->position.z;

        ctrl_cmd_.feedforward_vel(0) = msg->velocity.x;
        ctrl_cmd_.feedforward_vel(1) = msg->velocity.y;
        ctrl_cmd_.feedforward_vel(2) = msg->velocity.z;

        ctrl_cmd_.feedforward_acc(0) = msg->acceleration.x;
        ctrl_cmd_.feedforward_acc(1) = msg->acceleration.y;
        ctrl_cmd_.feedforward_acc(2) = msg->acceleration.z;
        ctrl_cmd_.yaw = msg->yaw;
        // set zero
        ctrl_cmd_.velocity = Eigen::Vector3d::Zero();
        ctrl_cmd_.acceleration = Eigen::Vector3d::Zero();

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
{ 
    // std::cout << "vrpn callback" << std::endl;
    geometry_msgs::PoseStamped modified_msg;
    modified_msg.header.stamp = ros::Time::now();
    modified_msg.header.frame_id = msg->header.frame_id; // keep the same frame id

    // Motion capture system uint is usually in millimeters, convert to meters
    modified_msg.pose.position.x = msg->pose.position.x / 1000.0; 
    modified_msg.pose.position.y = msg->pose.position.y / 1000.0;
    modified_msg.pose.position.z = msg->pose.position.z / 1000.0;
    modified_msg.pose.orientation = msg->pose.orientation;
    vision_pose_pub.publish(modified_msg);
}

void MavrosUtils::mavTakeoffCallback(const std_msgs::String::ConstPtr &msg, int drone_id)
{
    std::string received_string = msg->data;
    // std::cout << "Takeoff cmd received: " << received_string << std::endl;
    // std::cout << "Vehicle name: " << name << std::endl;
    if (received_string.find(std::to_string(drone_id)) != std::string::npos || 
        received_string.find("all") != std::string::npos)
    {
        ROS_INFO("%s: Received takeoff command", std::to_string(drone_id).c_str());
        fsm.setFlag("recv_takeoff_cmd", true);
        fsm.setFlag("recv_land_cmd", false);
    }
}

void MavrosUtils::mavLandCallback(const std_msgs::String::ConstPtr &msg, int drone_id)
{
    std::string received_string = msg->data;
    if (received_string.find(std::to_string(drone_id)) != std::string::npos || 
        received_string.find("all") != std::string::npos)
    {
        ROS_INFO("%s: Received land command", std::to_string(drone_id).c_str());
        fsm.setFlag("recv_takeoff_cmd", false);
        fsm.setFlag("recv_land_cmd", true);
    }
}

void MavrosUtils::mavCmd_vaildCallback(const std_msgs::String::ConstPtr &msg, int drone_id)
{
    std::string received_string = msg->data;
    if (received_string.find(std::to_string(drone_id)) != std::string::npos || 
        received_string.find("all") != std::string::npos)
    {
        ROS_INFO("%s: Received cmd command", std::to_string(drone_id).c_str());
        fsm.setFlag("cmd_vaild", true);
    }
}

void MavrosUtils::mavStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    context_.armed = msg->armed;
    context_.connected = msg->connected;
    context_.mode = msg->mode;
}
void MavrosUtils::mavExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    // 1 is landed
    // 2 is in air
    if(msg->landed_state == 1 && context_.landed_state != 1)
    {
        // ROS_INFO("Landed");
        context_.landed_state = true;
    }
    else if(msg->landed_state != 1 && context_.landed_state == 1)
    {
         context_.landed_state = false;
    }
    
}
void MavrosUtils::mavImuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    Eigen::Vector3d baselink_acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    odometry_.imu_attitude.x() = msg->orientation.x;
    odometry_.imu_attitude.y() = msg->orientation.y;
    odometry_.imu_attitude.z() = msg->orientation.z;
    odometry_.imu_attitude.w() = msg->orientation.w;
    
    odometry_.acc = odometry_.imu_attitude * baselink_acc; //
    
    double dt = (msg->header.stamp - context_.last_recv_imu_data_time).toSec();
    context_.last_recv_imu_data_time = msg->header.stamp;
    if (dt > 0.1 && enable_imu_dt_check_f)
    {
        ROS_WARN("IMU dt is too large");
        return;
    }

    #define ACC_MODE
    #ifdef ACC_MODE
    // std::cout << "gogog"<<std::endl;
    if (ctrl_cmd_.thrust > 0.1)
    {
        hover_thrust_ekf_->predict(dt); // dt
        hover_thrust_ekf_->fuseAccZ(odometry_.acc(2) - CONSTANTS_ONE_G, ctrl_cmd_.thrust);
        _hover_thrust = hover_thrust_ekf_->getHoverThrust();
    }
    // 记录最近的odometry_.pos(2) 
    // hover_thrust_ekf_->printLog();
    std_msgs::Float64 hover_thrust_msg;
    hover_thrust_msg.data = _hover_thrust;
    hover_thrust_pub_.publish(hover_thrust_msg);
    lin_controller.set_hover_thrust(_hover_thrust);
    #endif

    // #ifndef ACC_MODE
    // // // 收集最近2s的imu数据
    // acc_fusion_->addDataPoint(msg->header.stamp.toSec(),odometry_.position(2),odometry_.acc(2) - CONSTANTS_ONE_G);
    // static double last_fusion_time = 0;
    // if(acc_fusion_->getDataDuration() > 3.0 && (msg->header.stamp.toSec() - last_fusion_time) > 3.0)
    // {
    //     std::vector<double> time_vec, acc_vec;
    //     acc_fusion_->update(time_vec, acc_vec);
    //     last_fusion_time = msg->header.stamp.toSec();
    //     for (int i = 0; i < time_vec.size(); i++)
    //     {
    //         // std::cout << "time: " << time_vec[i] << " acc: " << acc_vec[i] << std::endl;
    //         hover_thrust_ekf_->predict(time_vec[i]);
    //         hover_thrust_ekf_->fuseAccZ(acc_vec[i], ctrl_cmd_.thrust);

    //     }
    //     _hover_thrust = hover_thrust_ekf_->getHoverThrust();
    //     std_msgs::Float64 hover_thrust_msg;
    //     hover_thrust_msg.data = _hover_thrust;
    //     hover_thrust_pub_.publish(hover_thrust_msg);
    //     lin_controller.set_hover_thrust(_hover_thrust);
    // }
    // #endif
}

void MavrosUtils::mavRefOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO("%s mavRefOdomCallback Current State: %s", params_parse.name.c_str(), fsm.getStatusMsg().c_str());
    
    fsm.updateOdomTimestamp(msg->header.stamp);
    odometry_.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odometry_.rate = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    odometry_.attitude.x() = msg->pose.pose.orientation.x;
    odometry_.attitude.y() = msg->pose.pose.orientation.y;
    odometry_.attitude.z() = msg->pose.pose.orientation.z;
    odometry_.attitude.w() = msg->pose.pose.orientation.w;

    odometry_.velocity = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    lin_controller.set_status(odometry_.position, odometry_.velocity, odometry_.rate, odometry_.attitude,odometry_.imu_attitude);
}
void MavrosUtils::mavLocalOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO("%s Current State: %s", params_parse.name.c_str(), fsm.getStatusMsg().c_str());
    fsm.updateOdomTimestamp(msg->header.stamp);
    odometry_.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odometry_.rate = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    odometry_.attitude.x() = msg->pose.pose.orientation.x;
    odometry_.attitude.y() = msg->pose.pose.orientation.y;
    odometry_.attitude.z() = msg->pose.pose.orientation.z;
    odometry_.attitude.w() = msg->pose.pose.orientation.w;

    Eigen::Vector3d baselink_vel = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    odometry_.velocity = odometry_.attitude * baselink_vel; // body frame to world frame
    lin_controller.set_status(odometry_.position, odometry_.velocity, odometry_.rate, odometry_.attitude,odometry_.imu_attitude);

    nav_msgs::Odometry world_odom;
    world_odom.header.stamp = msg->header.stamp;
    world_odom.header.frame_id = "uav_world"; // 世界坐标系
    world_odom.child_frame_id = "base_link"; // 子坐标系
    world_odom.pose.pose.position.x = odometry_.position(0);
    world_odom.pose.pose.position.y = odometry_.position(1);
    world_odom.pose.pose.position.z = odometry_.position(2);
    world_odom.pose.pose.orientation.x = odometry_.attitude.x();
    world_odom.pose.pose.orientation.y = odometry_.attitude.y();
    world_odom.pose.pose.orientation.z = odometry_.attitude.z();
    world_odom.pose.pose.orientation.w = odometry_.attitude.w();
    world_odom.twist.twist.linear.x = odometry_.velocity(0);
    world_odom.twist.twist.linear.y = odometry_.velocity(1);
    world_odom.twist.twist.linear.z = odometry_.velocity(2);
    world_odom.twist.twist.angular.x = odometry_.rate(0);
    world_odom.twist.twist.angular.y = odometry_.rate(1);
    world_odom.twist.twist.angular.z = odometry_.rate(2);

    world_odom_pub_.publish(world_odom); // 发布世界坐标系下的odom
}