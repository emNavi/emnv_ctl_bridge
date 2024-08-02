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


#define ROS_RATE 100.0
#define PUB_MODE "ATTI"

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
        geometry_msgs::Twist airgym_vel;

        airgym_vel.linear.x = msg->linear.x;
        airgym_vel.linear.y = msg->linear.y;
        airgym_vel.linear.z = msg->linear.z;

        // airgym_vel.linear.x = MyMath::clamp<double>(msg->linear.x, -1, 1);
        // airgym_vel.linear.y = MyMath::clamp<double>(msg->linear.y, -1, 1);
        // airgym_vel.linear.z = MyMath::clamp<double>(msg->linear.z, -1, 1);

        mavros_utils_ptr->update(boost::make_shared<geometry_msgs::Twist>(airgym_vel));

    }
}
mavros_msgs::AttitudeTarget des_body_rate;
Eigen::Vector3d des_rate;
double des_thrust;

void des_body_rate_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    fsm.update_cmd_update_time(ros::Time::now());
    if (fsm.now_state == CtrlFSM::RUNNING)
    {
        des_body_rate.body_rate = msg->body_rate;
        des_rate(0) = msg->body_rate.x;
        des_rate(1) = msg->body_rate.y;
        des_rate(2) = msg->body_rate.z;

        des_thrust = msg->thrust;
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
struct node_param
{
    std::string cmd_pub_type;
    double takeoff_height;
    int drone_id;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_ctrl_node");
    ros::NodeHandle nh("~");

    node_param param;

    nh.param<int>("drone_id", param.drone_id, 99);
    nh.param<double>("takeoff_height", param.takeoff_height, 0.3);
    nh.param<std::string>("cmd_pub_type", param.cmd_pub_type, "ATTI");



    std::cout << "drone id " << param.drone_id << std::endl;
    std::cout << "takeoff_height" << param.cmd_pub_type << std::endl;
    ros::Time hover_above_land_start_time = ros::Time::now();

    mavros_utils_ptr = new MavrosUtils(nh);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    fsm.Init_FSM();

    // vrpn - vision_pose
    // ros::Subscriber vrpn_pose = nh.subscribe<geometry_msgs::PoseStamped>("/quadrotor_control/odom", 10, vrpn_cb);
    ros::Subscriber vrpn_pose = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_pose", 10, vrpn_cb);


    // direct command
    ros::Subscriber  pva_yaw_sub= nh.subscribe<mavros_msgs::PositionTarget>("pos_cmd", 10, pva_yaw_cb);
    ros::Subscriber local_linear_vel_sub = nh.subscribe<geometry_msgs::Twist>("vel_sp_sub", 10, local_linear_vel_cb);
    ros::Subscriber rate_sp_sub = nh.subscribe<geometry_msgs::Twist>("rate_sp_sub", 10, local_linear_vel_cb);

    ros::Subscriber takeoff_cmd_sub = nh.subscribe<std_msgs::Float32MultiArray>("/swarm_takeoff", 10, boost::bind(takeoff_cmd_cb, _1, param.drone_id));
    ros::Subscriber land_cmd_sub = nh.subscribe<std_msgs::Float32MultiArray>("/swarm_land", 10, boost::bind(land_cmd_cb, _1, param.drone_id));
    
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);


    mavros_utils_ptr->connect();
    ros::Rate rate(ROS_RATE);
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
            mavros_utils_ptr->set_motors_idling(); //不能停


        }
        else if (fsm.now_state == CtrlFSM::TAKEOFF)
        {
            if (fsm.last_state != CtrlFSM::TAKEOFF)
            {
                ROS_INFO("MODE: TAKEOFF");
                start_pose.pose.position.x = mavros_utils_ptr->_mav_odom.position(0);
                start_pose.pose.position.y = mavros_utils_ptr->_mav_odom.position(1);
                start_pose.pose.position.z = param.takeoff_height;
            }
            
            geometry_msgs::Twist takeoff_vel;
            takeoff_vel.linear.x = -(mavros_utils_ptr->_mav_odom.position(0) - start_pose.pose.position.x)*3; 
            takeoff_vel.linear.y = -(mavros_utils_ptr->_mav_odom.position(1) - start_pose.pose.position.y)*3; 
            takeoff_vel.linear.z = -(mavros_utils_ptr->_mav_odom.position(2) - start_pose.pose.position.z)*3; 
            takeoff_vel.linear.x = MyMath::clamp<double>(takeoff_vel.linear.x, -1, 1);
            takeoff_vel.linear.y = MyMath::clamp<double>(takeoff_vel.linear.y, -1, 1);
            takeoff_vel.linear.z = MyMath::clamp<double>(takeoff_vel.linear.z, -1, 1);


            mavros_utils_ptr->update(boost::make_shared<geometry_msgs::Twist>(takeoff_vel));
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

                start_pose.pose.position.x = mavros_utils_ptr->_mav_odom.position(0);
                start_pose.pose.position.y = mavros_utils_ptr->_mav_odom.position(1);
                start_pose.pose.position.z = mavros_utils_ptr->_mav_odom.position(2);
                start_pose.pose.orientation.x =  mavros_utils_ptr->_mav_odom.attitude.x();
                start_pose.pose.orientation.y =  mavros_utils_ptr->_mav_odom.attitude.y();
                start_pose.pose.orientation.z =  mavros_utils_ptr->_mav_odom.attitude.z();
                start_pose.pose.orientation.w =  mavros_utils_ptr->_mav_odom.attitude.w();
                std::cout << "hover position" <<" "<< start_pose.pose.position.x << " "<< start_pose.pose.position.y <<" "<< start_pose.pose.position.z<<std::endl;


            }
            geometry_msgs::Twist hover_vel;
            hover_vel.linear.x = -(mavros_utils_ptr->_mav_odom.position(0) - start_pose.pose.position.x)*3; 
            hover_vel.linear.y = -(mavros_utils_ptr->_mav_odom.position(1) - start_pose.pose.position.y)*3; 
            hover_vel.linear.z = -(mavros_utils_ptr->_mav_odom.position(2) - start_pose.pose.position.z)*3; 
            hover_vel.linear.x = MyMath::clamp<double>(hover_vel.linear.x, -1, 1);
            hover_vel.linear.y = MyMath::clamp<double>(hover_vel.linear.y, -1, 1);
            hover_vel.linear.z = MyMath::clamp<double>(hover_vel.linear.z, -1, 1);

            if(PUB_MODE == "ATTI")
            {
                mavros_utils_ptr->update(boost::make_shared<geometry_msgs::Twist>(hover_vel));
            }
            else if(PUB_MODE == "RATE")
            {
                mavros_utils_ptr->hover_update(boost::make_shared<geometry_msgs::Twist>(hover_vel));
                des_rate(0) = mavros_utils_ptr->_mav_atti_cmd.rate(0);
                des_rate(1) = mavros_utils_ptr->_mav_atti_cmd.rate(1);
                des_rate(2) = mavros_utils_ptr->_mav_atti_cmd.rate(2);
                des_thrust = mavros_utils_ptr->_mav_atti_cmd.thrust;
            }
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
                start_pose.pose.position.x = mavros_utils_ptr->_mav_odom.position(0);
                start_pose.pose.position.y = mavros_utils_ptr->_mav_odom.position(1);
                start_pose.pose.position.z = mavros_utils_ptr->_mav_odom.position(2);
                start_pose.pose.orientation.x =  mavros_utils_ptr->_mav_odom.attitude.x();
                start_pose.pose.orientation.y =  mavros_utils_ptr->_mav_odom.attitude.y();
                start_pose.pose.orientation.z =  mavros_utils_ptr->_mav_odom.attitude.z();
                start_pose.pose.orientation.w =  mavros_utils_ptr->_mav_odom.attitude.w();
                ROS_INFO("MODE: LAND");
                hover_above_land_start_time = ros::Time::now();
            }

            geometry_msgs::Twist land_vel;
            land_vel.linear.x = -(mavros_utils_ptr->_mav_odom.position(0) - start_pose.pose.position.x)*3; 
            land_vel.linear.y = -(mavros_utils_ptr->_mav_odom.position(1) - start_pose.pose.position.y)*3; 
            land_vel.linear.z = -0.3; 
            land_vel.linear.x = MyMath::clamp<double>(land_vel.linear.x, -1, 1);
            land_vel.linear.y = MyMath::clamp<double>(land_vel.linear.y, -1, 1);
            land_vel.linear.z = MyMath::clamp<double>(land_vel.linear.z, -1, 1);


            mavros_utils_ptr->update(boost::make_shared<geometry_msgs::Twist>(land_vel));

            if(mavros_utils_ptr->get_hover_thrust() > 0.11)
            {
                hover_above_land_start_time = ros::Time::now();
            }
            if(ros::Time::now() -  hover_above_land_start_time > ros::Duration(2))
            {
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
        if(PUB_MODE == "ATTI")
        {
            mavros_utils_ptr->send_atti_cmd();
        }
        else if(PUB_MODE == "RATE")
        {
            // ROS_INFO_STREAM()
            mavros_utils_ptr->send_rate_cmd(des_rate,des_thrust);
        }


        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
