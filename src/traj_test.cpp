#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Eigen>
#include <fstream>


#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "emnv_ctl_bridge/PvayCommand.h"
#include "emnv_ctl_bridge/poly_traj/polynomial_traj.hpp"
#include <std_msgs/String.h>

nav_msgs::Path path;

geometry_msgs::PoseStamped pose;

PolynomialTraj gl_traj;


ros::Publisher key_point_paths_pub,paths_pub,cmd_pub;

Eigen::MatrixXd key_pos;
Eigen::Vector3d start_vel, end_vel, start_acc, end_acc;
Eigen::VectorXd segment_times;


std::ofstream csvFile;
template<typename T>
T getRequiredParam(ros::NodeHandle& nh, const std::string& name) {
    T val;
    if (!nh.getParam(name, val)) {
        ROS_ERROR_STREAM("Missing parameter: " << name);
        throw std::runtime_error("Missing parameter: " + name);
    }
    return val;
}

bool play_trajectory = false;
void bridgeStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    // ROS_INFO_STREAM("Received string message: " << msg->data);
    if(msg->data == "HOVER")
    {
        play_trajectory = true;
    }
}
int main(int argc, char* argv[])
{
    // Initialise the node
    ros::init(argc, argv, "polynomial_traj_test");
    // Start the node by initialising a node handle
    ros::NodeHandle nh("~");



    ros::Subscriber string_sub = nh.subscribe<std_msgs::String>("bridge_status", 10, bridgeStatusCallback);
    cmd_pub = nh.advertise<emnv_ctl_bridge::PvayCommand>("cmd", 100, true);

    bool pub_cmd_flag = getRequiredParam<bool>(nh, "pub_cmd_flag");

    int key_point_num = 0;
    key_point_num = getRequiredParam<int>(nh, "key_point_num");
    key_pos.resize(3, key_point_num);
    for(int i = 0; i < key_point_num; i++)
    {
        std::string key_point_name = "key_point_" + std::to_string(i);
        key_pos(0, i) = getRequiredParam<double>(nh, key_point_name + "_x");
        key_pos(1, i) = getRequiredParam<double>(nh, key_point_name + "_y");
        key_pos(2, i) = getRequiredParam<double>(nh, key_point_name + "_z");
    }
    ROS_INFO_STREAM("key_point_num = " << key_point_num);
    segment_times.resize(key_point_num-1);
    for(int i = 0; i < key_point_num-1; i++)
    {
        segment_times(i) = getRequiredParam<double>(nh, "segment_time_" + std::to_string(i));
    }

    start_vel(0) = getRequiredParam<double>(nh, "start_vel_x");
    start_vel(1) = getRequiredParam<double>(nh, "start_vel_y");
    start_vel(2) = getRequiredParam<double>(nh, "start_vel_z");
    end_vel(0) = getRequiredParam<double>(nh, "end_vel_x");
    end_vel(1) = getRequiredParam<double>(nh, "end_vel_y");
    end_vel(2) = getRequiredParam<double>(nh, "end_vel_z");
    start_acc(0) = getRequiredParam<double>(nh, "start_acc_x");
    start_acc(1) = getRequiredParam<double>(nh, "start_acc_y");
    start_acc(2) = getRequiredParam<double>(nh, "start_acc_z");
    end_acc(0) = getRequiredParam<double>(nh, "end_acc_x");
    end_acc(1) = getRequiredParam<double>(nh, "end_acc_y");
    end_acc(2) = getRequiredParam<double>(nh, "end_acc_z");
            

    // 创建 CSV 文件
    std::string filename = "trajectory.csv";
    csvFile.open(filename.c_str());
    // 写入 CSV 文件头
    csvFile << "p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z" << std::endl;
    paths_pub = nh.advertise<nav_msgs::Path>("real_path", 100, true);

    ROS_INFO_STREAM(key_pos);
    std::cout << "Key point set" << std::endl;
    std::cout << "Start velocity: " << start_vel.transpose() << std::endl;
    std::cout << "End velocity: " << end_vel.transpose() << std::endl
                << "Start acceleration: " << start_acc.transpose() << std::endl
                << "End acceleration: " << end_acc.transpose() << std::endl;

    std::cout << "Segment times: ";
    for(int i = 0; i < segment_times.size(); i++)
    {
        std::cout << segment_times(i) << " ";
    }
    std::cout << std::endl;


    for(int i = 0; i < key_point_num; i++)
    {
        pose.header.frame_id = "world";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = key_pos(0,i);
        pose.pose.position.y = key_pos(1,i);
        pose.pose.position.z = key_pos(2,i);
        path.poses.push_back(pose);
    }


    ROS_INFO("Start to generate trajectory");
    gl_traj = PolynomialTraj::minSnapTraj(key_pos, start_vel, end_vel, start_acc, end_acc, segment_times);
    ROS_INFO("OK");

    gl_traj.init();
    double global_duration_ = gl_traj.getTimeSum();
    ROS_INFO("global_duration = %f s", global_duration_);


    while ((ros::ok() && !play_trajectory) && pub_cmd_flag)
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    ROS_INFO("Start to play trajectory");
    double last_yaw = 0;
    emnv_ctl_bridge::PvayCommand cmd;
    cmd.header.frame_id = "world";
    double ts = 0.1; // time step for trajectory evaluation
    int index = 0;
    for (double t = 0; t < global_duration_; t += ts)
    {   
        if(ros::ok() == false)
        {
            break;
        }
        Eigen::Vector3d pt = gl_traj.evaluate(t);
        Eigen::Vector3d pt_v = gl_traj.evaluateVel(t);
        Eigen::Vector3d pt_a = gl_traj.evaluateAcc(t);


        csvFile << pt(0) << "," << pt(1) << "," << pt(2) << ","
        << pt_v(0) << "," << pt_v(1) << "," << pt_v(2) << ","
        << pt_a(0) << "," << pt_a(1) << "," << pt_a(2) << std::endl;


        if(pub_cmd_flag)
        {
            cmd.header.stamp = ros::Time::now();
            cmd.position.x = pt(0);
            cmd.position.y = pt(1);
            cmd.position.z = pt(2);
            cmd.velocity.x = pt_v(0);
            cmd.velocity.y = pt_v(1);
            cmd.velocity.z = pt_v(2);
            cmd.acceleration.x = pt_a(0);
            cmd.acceleration.y = pt_a(1);
            cmd.acceleration.z = pt_a(2);
            double yaw  = last_yaw;
            if(t+0.5 > global_duration_)
            {
                yaw  = last_yaw;
            }
            else{
                Eigen::Vector3d pt_next = gl_traj.evaluate(t+0.5);
                // 计算 yaw_vector
                Eigen::Vector3d yaw_vector = pt_next - pt;
                double vector_length = yaw_vector.norm();
                if (vector_length >= 0.1) {
                    yaw = std::atan2(yaw_vector[1], yaw_vector[0]);
                }
            }
            cmd.yaw = yaw;
            // 计算 yaw 角
            cmd_pub.publish(cmd);
            last_yaw = yaw;
        }



        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped cur_pos;
        cur_pos.header.frame_id = "world";
        cur_pos.header.stamp = ros::Time::now();
        cur_pos.header.seq = index;
        cur_pos.pose.position.x = pt(0);
        cur_pos.pose.position.y = pt(1);
        cur_pos.pose.position.z = pt(2);
        index++;
        path.poses.push_back(cur_pos);
        paths_pub.publish(path);

        // pub cmd quadrotor_msgs::PositionCommand
        ros::Duration(ts).sleep();
        ros::spinOnce();
    }
    ros::spin();
    csvFile.close();

    // Main has ended, return 0
    return 0;
}