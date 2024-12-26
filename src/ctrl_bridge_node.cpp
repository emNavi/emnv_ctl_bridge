#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Float32MultiArray.h>
#include <boost/shared_ptr.hpp>

#include "control_for_gym/FSM.hpp"
#include "control_for_gym/mavros_utils.hpp"
#include <std_msgs/String.h> 

mavros_msgs::CommandBool arm_cmd;
ros::Publisher local_raw_pub, local_linear_vel_pub,atti_ctrl_pub;
ros::Publisher err_pub;
ros::Publisher vision_pose_pub;

std_msgs::Float32MultiArray takeoff_cmd;
std_msgs::Float32MultiArray land_cmd;


CtrlFSM fsm;

MavrosUtils* mavros_utils_ptr = nullptr;


bool f_recv_takeoff_cmd = false;
bool land_cmd_flag = false;


// callback begin************************

void pva_yaw_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    fsm.update_cmd_update_time(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        local_raw_pub.publish(msg);
    }
}
void local_linear_vel_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    fsm.update_cmd_update_time(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        Eigen::Vector3d vel = Eigen::Vector3d(msg->linear.x, msg->linear.y, msg->linear.z);
        mavros_utils_ptr->ctrl_update(vel,0.0,0.01);    
    }
}

void des_body_rate_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    fsm.update_cmd_update_time(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        mavros_utils_ptr->_mav_low_cmd.rate(0) = msg->body_rate.x;
        mavros_utils_ptr->_mav_low_cmd.rate(1) = msg->body_rate.y;
        mavros_utils_ptr->_mav_low_cmd.rate(2) = msg->body_rate.z;
        mavros_utils_ptr->_mav_low_cmd.thrust = msg->thrust;
    }
}

void des_atti_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    fsm.update_cmd_update_time(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        mavros_utils_ptr->_mav_low_cmd.attitude.x() = msg->orientation.x;
        mavros_utils_ptr->_mav_low_cmd.attitude.y() = msg->orientation.y;
        mavros_utils_ptr->_mav_low_cmd.attitude.z() = msg->orientation.z;
        mavros_utils_ptr->_mav_low_cmd.attitude.w() = msg->orientation.w;
        mavros_utils_ptr->_mav_low_cmd.thrust = msg->thrust;
    }
}

void vrpn_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
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

void takeoff_cb(const std_msgs::String::ConstPtr& msg, std::string name)
{
    std::string received_string = msg->data;

    if (received_string.find(name) != std::string::npos) {
        f_recv_takeoff_cmd = true;
    }
}

void land_cb(const std_msgs::String::ConstPtr& msg, std::string name)
{
    std::string received_string = msg->data;
    if (received_string.find(name) != std::string::npos) {
        land_cmd_flag = true;
    }
}


struct node_param
{
    std::string ctrl_out_level;
    double takeoff_height;
    int drone_id;
    double loop_rate;
    std::string name;
    

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ctrl_bridge");
    ros::NodeHandle nh("~");

    node_param param;

    nh.param<int>("drone_id", param.drone_id, 99);
    nh.param<double>("takeoff_height", param.takeoff_height, 0.3);
    nh.param<std::string>("ctrl_out_level", param.ctrl_out_level, "ATTI");
    nh.param<double>("loop_rate", param.loop_rate, 100.0);
    nh.param<std::string>("name", param.name, "drone");

    MavrosUtils::CTRL_OUTPUT_LVEVL ctrl_out_level = MavrosUtils::ATTI;
    if (param.ctrl_out_level == "ATTI")
    {
        ctrl_out_level = MavrosUtils::ATTI;
    }
    else if (param.ctrl_out_level == "RATE")
    {
        ctrl_out_level = MavrosUtils::RATE;
    }

    std::cout << "ctrl_out_level " << param.ctrl_out_level << std::endl;
    std::cout << "drone id " << param.drone_id << std::endl;
    std::cout << "takeoff_height" << param.takeoff_height << std::endl;

    ros::Time hover_above_land_start_time = ros::Time::now();

    mavros_utils_ptr = new MavrosUtils(nh,ctrl_out_level);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    fsm.Init_FSM();

    // vrpn - vision_pose
    ros::Subscriber vrpn_pose = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_pose", 10, vrpn_cb);

    // direct command
    ros::Subscriber  pva_yaw_sub= nh.subscribe<mavros_msgs::PositionTarget>("pos_cmd", 10, pva_yaw_cb);
    ros::Subscriber local_linear_vel_sub = nh.subscribe<geometry_msgs::Twist>("vel_sp_sub", 10, local_linear_vel_cb);
    ros::Subscriber atti_sp_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("atti_sp_sub", 10, des_atti_cb);
    ros::Subscriber rate_sp_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("rate_sp_sub", 10, des_body_rate_cb);

    ros::Subscriber takeoff_sub = nh.subscribe<std_msgs::String>("/emnavi_cmd/takeoff", 1000, boost::bind(&takeoff_cb, _1, param.name));
    ros::Subscriber land_sub = nh.subscribe<std_msgs::String>("/emnavi_cmd/land", 1000, boost::bind(&land_cb, _1, param.name));
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);


    mavros_utils_ptr->connect();
    ros::Rate rate(param.loop_rate);
    /* ------------------init params --------------------------------------------------*/
    geometry_msgs::PoseStamped start_pose;
    Eigen::Vector3d start_pos;
    Eigen::Quaterniond start_q;
    /* ------------------init params done --------------------------------------------*/
    while (ros::ok())
    {
        fsm.process();
        if (fsm.now_state == CtrlFSM::INITIAL)
        {
            if (!f_recv_takeoff_cmd)
            {
                ros::spinOnce();
                ros::Rate(10).sleep();
                continue;
            }
            // Request offboard and Arm
            if (!mavros_utils_ptr->isOffboardMode() &&
                (ros::Time::now() - fsm.last_try_offboard_time > ros::Duration(5.0)))
            {
                if (!mavros_utils_ptr->request_offboard())
                    ROS_WARN("offboard failed,try again after 5.0 second");
                fsm.last_try_offboard_time = ros::Time::now();
            }
            else if (mavros_utils_ptr->isOffboardMode() && !mavros_utils_ptr->isArmed() &&
                     (ros::Time::now() - fsm.last_try_arm_time > ros::Duration(5.0)))
            {
                if (!mavros_utils_ptr->request_arm())
                    ROS_WARN("arm failed,try again after 5.0 second");
                fsm.last_try_arm_time = ros::Time::now();
            }

            // set flag
            if (mavros_utils_ptr->isOffboardMode())
            {
                fsm.set_offboard_flag(true);
            }
            if (mavros_utils_ptr->isOffboardMode() && mavros_utils_ptr->isArmed())
            {
                fsm.set_arm_flag(true);
            }
            mavros_utils_ptr->set_motors_idling(); //设置怠速
        }
        else if (fsm.now_state == CtrlFSM::TAKEOFF)
        {
            if (fsm.last_state != CtrlFSM::TAKEOFF)
            {
                ROS_INFO("MODE: TAKEOFF");
                // mavros_utils_ptr->_mav_odom comes from /mavros/local_position/odom
                start_pos(0) = mavros_utils_ptr->_mav_odom.position(0);
                start_pos(1) = mavros_utils_ptr->_mav_odom.position(1);
                start_pos(2) = param.takeoff_height;
            }
            
            Eigen::Vector3d des_takeoff_vel;
            double takeoff_ctl_gain = 3;
            des_takeoff_vel = -(mavros_utils_ptr->_mav_odom.position - start_pos)*takeoff_ctl_gain; 
            des_takeoff_vel.cwiseMin(-1).cwiseMax(1);
            mavros_utils_ptr->ctrl_update(des_takeoff_vel,0.0,0.01);    
            ROS_INFO_STREAM("des_takeoff_vel: " << des_takeoff_vel.transpose());

            // set auto_takeoff_height
            if (abs(mavros_utils_ptr->_mav_odom.position(2) - param.takeoff_height) < 0.1)
            {
                fsm.set_takeoff_over_flag(true);
                ROS_INFO("Take off done");
            }
        }
        else if (fsm.now_state == CtrlFSM::HOVER)
        {
            if (fsm.last_state != CtrlFSM::HOVER)
            {
                ROS_INFO("MODE: HOVER");
                start_pos = mavros_utils_ptr->_mav_odom.position;
                start_q = mavros_utils_ptr->_mav_odom.attitude;
            }
            Eigen::Vector3d hover_vel;
            hover_vel = -(mavros_utils_ptr->_mav_odom.position - start_pos)*3;
            hover_vel.cwiseMin(-1).cwiseMax(1);
            ROS_INFO_STREAM("hover_vel: " << hover_vel.transpose());
            mavros_utils_ptr->ctrl_update(hover_vel,0.0,0.01);        
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
                start_pos = mavros_utils_ptr->_mav_odom.position;
                start_q = mavros_utils_ptr->_mav_odom.attitude;
                ROS_INFO("MODE: LAND");
                ROS_INFO_STREAM("start_pos: " << start_pos.transpose());
                hover_above_land_start_time = ros::Time::now();
            }
            Eigen::Vector3d land_vel;
            land_vel = -(mavros_utils_ptr->_mav_odom.position - start_pos)*3;
            land_vel(2) = -0.3;
            land_vel.cwiseMin(-1).cwiseMax(1);
            mavros_utils_ptr->ctrl_update(land_vel,0.0,0.01);

            if(mavros_utils_ptr->get_hover_thrust() > 0.11)
            {
                hover_above_land_start_time = ros::Time::now();
            }
            if(ros::Time::now() -  hover_above_land_start_time > ros::Duration(2))
            {
                ROS_INFO("Land done");
                mavros_utils_ptr->request_disarm();
            }

            // if (current_state.mode != "AUTO.LAND" &&
            //     (ros::Time::now() - fsm.last_try_offboard_time > ros::Duration(5.0)))
            // {
            //     if (!request_land())
            //         ROS_WARN("Try land cmd failed, pls try again in 5 seconds");
            //     fsm.last_try_offboard_time = ros::Time::now();
            // }
        }
        if (land_cmd_flag)
        {
            fsm.set_land_flag(true);
        }
        mavros_utils_ptr->send_low_cmd();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
