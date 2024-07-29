/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */
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

#include "control_for_gym/FSM.hpp"
#include "control_for_gym/regular_motion.hpp"
#include "control_for_gym/mavros_utils.hpp"
#include <boost/shared_ptr.hpp>

#define ROS_RATE 50.0
#define TAKEOFF_HEIGHT 1.2

mavros_msgs::CommandBool arm_cmd;
ros::Publisher local_raw_pub, local_position_pub, local_linear_vel_pub,atti_ctrl_pub;
ros::Publisher err_pub;
ros::Publisher vision_pose_pub;


mavros_msgs::State current_state;
geometry_msgs::PoseStamped cur_pos;
geometry_msgs::TwistStamped cur_vel;
std_msgs::Float32MultiArray takeoff_cmd;
std_msgs::Float32MultiArray land_cmd;
CtrlFSM fsm;

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
        local_linear_vel_pub.publish(msg);
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

bool f_recv_takeoff_cmd = false;
void takeoff_cmd_cb(const std_msgs::Float32MultiArray::ConstPtr &msg, int drone_id)
{

    takeoff_cmd = *msg;
    // ROS_INFO_STREAM(takeoff_cmd.data[0] << takeoff_cmd.data[1] << drone_id);

    for (int i = 0; i < takeoff_cmd.data[0] + 1e-2; i++)
    {
        if (abs(takeoff_cmd.data[i + 1] - drone_id) < 1e-3)
        {
            f_recv_takeoff_cmd = true;
        }
    }
}

bool land_cmd_flag = false;
void land_cmd_cb(const std_msgs::Float32MultiArray::ConstPtr &msg, int drone_id)
{
    land_cmd = *msg;
    for (int i = 0; i < land_cmd.data[0]; i++)
    {
        if (abs(land_cmd.data[i + 1] - drone_id) < 1e-3)
        {
            land_cmd_flag = true;
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_ctrl_node");
    ros::NodeHandle nh("~");

    std::string ctrl_mode;
    int drone_id;
    nh.param<int>("drone_id", drone_id, 99);

    nh.param<std::string>("ctrl_mode", ctrl_mode, "vel");
    std::cout << "ctrl_mode " << ctrl_mode << std::endl;
    std::cout << "drone id " << drone_id << std::endl;

    //////////////////////////////////////////////////////
    fsm.Init_FSM();


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    MavrosUtils mavros_utils(nh);

    // vrpn - vision_pose
    ros::Subscriber vrpn_pose = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/drone_7/pose", 10, vrpn_cb);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    // direct command
    ros::Subscriber  pva_yaw_sub= nh.subscribe<mavros_msgs::PositionTarget>("pos_cmd", 10, pva_yaw_cb);
    ros::Subscriber local_linear_vel_sub = nh.subscribe<geometry_msgs::Twist>("vel_cmd", 10, local_linear_vel_cb);

    ros::Subscriber takeoff_cmd_sub = nh.subscribe<std_msgs::Float32MultiArray>("/swarm_takeoff", 10, boost::bind(takeoff_cmd_cb, _1, drone_id));
    ros::Subscriber land_cmd_sub = nh.subscribe<std_msgs::Float32MultiArray>("/swarm_land", 10, boost::bind(land_cmd_cb, _1, drone_id));
    
    local_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    mavros_utils.connect();
    // ros::Rate(1).sleep();
    ros::Rate rate(50);
    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = TAKEOFF_HEIGHT;
    // for (int i = 100; ros::ok() && i > 0; --i)
    // {
    //     local_position_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    // send a few setpoints before starting
    // for (int i = 100; ros::ok() && i > 0; --i)
    // {
    //     // local_position_pub.publish(pose);
    //     mavros_utils.set_motors_idling();
    //     mavros_utils.send_atti_cmd();
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    /* ------------------init params --------------------------------------------------*/
    geometry_msgs::PoseStamped start_pose;
    /* ------------------init params done --------------------------------------------*/
    while (ros::ok())
    {
        fsm.process();
        if (fsm.now_state == CtrlFSM::INIT_PARAM)
        {
            if (!f_recv_takeoff_cmd)
            {
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // Request offboard and Arm
            if (!mavros_utils.isOffboardMode() &&
                (ros::Time::now() - fsm.last_try_offboard_time > ros::Duration(5.0)))
            {
                if (!mavros_utils.request_offboard())
                    ROS_WARN("offboard failed,try again after 5.0 second");
                fsm.last_try_offboard_time = ros::Time::now();
            }
            else if (mavros_utils.isOffboardMode() && !mavros_utils.isArmed() &&
                     (ros::Time::now() - fsm.last_try_arm_time > ros::Duration(5.0)))
            {
                if (!mavros_utils.request_arm())
                    ROS_WARN("arm failed,try again after 5.0 second");
                fsm.last_try_arm_time = ros::Time::now();
            }

            // set flag
            if (mavros_utils.isOffboardMode())
            {
                fsm.set_offboard_flag(true);
            }
            if (mavros_utils.isOffboardMode() && mavros_utils.isArmed())
            {
                fsm.set_arm_flag(true);
            }
            // local_position_pub.publish(pose);
            mavros_utils.set_motors_idling();

            // local_linear_vel_pub.publish(linear_vel_msg);

            // local_position_pub.publish(pose);
        }
        else if (fsm.now_state == CtrlFSM::TAKEOFF)
        {
            if (fsm.last_state != CtrlFSM::TAKEOFF)
            {
                ROS_INFO("MODE: TAKEOFF");
                start_pose.pose.position.x = cur_pos.pose.position.x;
                start_pose.pose.position.y = cur_pos.pose.position.y;
                start_pose.pose.position.z = TAKEOFF_HEIGHT;
            }
            // pose.pose = start_pose.pose;
            
            // local_position_pub.publish(pose);
            geometry_msgs::Twist takeoff_vel;
            takeoff_vel.linear.x = 0;
            takeoff_vel.linear.y = 0;
            takeoff_vel.linear.z = 0.5;
            mavros_utils.update(boost::make_shared<geometry_msgs::Twist>(takeoff_vel));
            if (abs(mavros_utils._mav_odom.position(2) - TAKEOFF_HEIGHT) < 0.1)
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

                start_pose.pose.position.x = mavros_utils._mav_odom.position(0);
                start_pose.pose.position.y = mavros_utils._mav_odom.position(1);
                start_pose.pose.position.z = mavros_utils._mav_odom.position(2);
                start_pose.pose.orientation.x =  mavros_utils._mav_odom.attitude.x();
                start_pose.pose.orientation.y =  mavros_utils._mav_odom.attitude.y();
                start_pose.pose.orientation.z =  mavros_utils._mav_odom.attitude.z();
                start_pose.pose.orientation.w =  mavros_utils._mav_odom.attitude.w();

            }
            geometry_msgs::Twist hover_vel;
            hover_vel.linear.x = 0;
            hover_vel.linear.y = 0;
            hover_vel.linear.z = 0.5;
            if((mavros_utils._mav_odom.position(2) - start_pose.pose.position.z) < -0.2)
                hover_vel.linear.z = 1; 
            else if((mavros_utils._mav_odom.position(2) - start_pose.pose.position.z) > 0.2)
                hover_vel.linear.z = -1; 
            else
                hover_vel.linear.z = -(mavros_utils._mav_odom.position(2) - start_pose.pose.position.z)*5; 
            mavros_utils.update(boost::make_shared<geometry_msgs::Twist>(hover_vel));


            // pose.pose = start_pose.pose;
            // local_linear_vel_pub.publish(linear_vel_msg);

            // local_position_pub.publish(pose);
        }
        else if (fsm.now_state == CtrlFSM::RUNNING)
        {
            if (fsm.last_state != CtrlFSM::RUNNING)
            {
                ROS_INFO_STREAM("MODE: RUNNING-"<<ctrl_mode);
            }
            
        }

        else if (fsm.now_state == CtrlFSM::LANDING)
        {
            if (fsm.last_state != CtrlFSM::LANDING)
            {
                ROS_INFO("MODE: LAND");
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
        
        mavros_utils.send_atti_cmd();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
