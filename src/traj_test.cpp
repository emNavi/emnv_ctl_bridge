#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Eigen>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "emnv_ctl_bridge/PvayCommand.h"
#include "emnv_ctl_bridge/poly_traj/polynomial_traj.hpp"
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

nav_msgs::Path path;

geometry_msgs::PoseStamped pose;

PolynomialTraj gl_traj;

ros::Publisher key_point_paths_pub, paths_pub, cmd_pub;

Eigen::MatrixXd key_pos;
Eigen::Vector3d start_vel, end_vel, start_acc, end_acc;
Eigen::VectorXd segment_times;

std::ofstream csvFile;
template <typename T>
T getRequiredParam(ros::NodeHandle &nh, const std::string &name)
{
    T val;
    if (!nh.getParam(name, val))
    {
        ROS_ERROR_STREAM("Missing parameter: " << name);
        throw std::runtime_error("Missing parameter: " + name);
    }
    return val;
}

bool play_trajectory = false;
void bridgeStatusCallback(const std_msgs::String::ConstPtr &msg)
{
    // ROS_INFO_STREAM("Received string message: " << msg->data);
    if (msg->data == "HOVER")
    {
        play_trajectory = true;
    }
}
int main(int argc, char *argv[])
{
    // Initialise the node
    ros::init(argc, argv, "polynomial_traj_test");
    // Start the node by initialising a node handle
    ros::NodeHandle nh("~");
    std::string traj_config_file = getRequiredParam<std::string>(nh, "traj_config_file");
    int num_key_points = 0;
    double avg_v = 1.0;
    double max_v = 3.0;
    double max_a = 3.0;
    double pub_ts = 0.1;
    bool is_loop_mode = false;
    try
    {
        YAML::Node config = YAML::LoadFile(traj_config_file);
        num_key_points = config["key_point"].size();
        key_pos.resize(3, num_key_points);
        for (int i = 0; i < num_key_points; ++i)
        {
            key_pos(0, i) = config["key_point"][i][0].as<double>();
            key_pos(1, i) = config["key_point"][i][1].as<double>();
            key_pos(2, i) = config["key_point"][i][2].as<double>();
        }
        avg_v = config["average_speed"].as<double>();
        max_v = config["max_vel"].as<double>();
        max_a = config["max_acc"].as<double>();
        pub_ts = config["pub_time_step"].as<double>();
        is_loop_mode = config["is_loop_mode"].as<bool>();
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR_STREAM("YAML error: " << e.what() << " " << traj_config_file);
    }
    ROS_INFO_STREAM("num_key_points = " << num_key_points);

    //  cal segment_times
    segment_times.resize(key_pos.cols() - 1);
    for (int i = 0; i < key_pos.cols() - 1; i++)
    {
        double dist = (key_pos.col(i + 1) - key_pos.col(i)).norm();
        segment_times(i) = dist / avg_v;
        if (segment_times(i) < 0.1)
            segment_times(i) = 0.1;
    }
    ros::Subscriber string_sub = nh.subscribe<std_msgs::String>("bridge_status", 10, bridgeStatusCallback);
    cmd_pub = nh.advertise<emnv_ctl_bridge::PvayCommand>("cmd", 100, true);

    start_vel = Eigen::Vector3d::Zero();
    end_vel = Eigen::Vector3d::Zero();
    start_acc = Eigen::Vector3d::Zero();
    end_acc = Eigen::Vector3d::Zero();

    paths_pub = nh.advertise<nav_msgs::Path>("real_path", 100, true);

    ROS_INFO_STREAM(key_pos);
    std::cout << "Key point set" << std::endl;
    std::cout << "Start velocity: " << start_vel.transpose() << std::endl;
    std::cout << "End velocity: " << end_vel.transpose() << std::endl
              << "Start acceleration: " << start_acc.transpose() << std::endl
              << "End acceleration: " << end_acc.transpose() << std::endl;

    for (int i = 0; i < num_key_points; i++)
    {
        pose.header.frame_id = "world";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = key_pos(0, i);
        pose.pose.position.y = key_pos(1, i);
        pose.pose.position.z = key_pos(2, i);
        path.poses.push_back(pose);
    }
    ROS_INFO("Start to generate trajectory");
    gl_traj = PolynomialTraj::minSnapTraj(key_pos, start_vel, end_vel, start_acc, end_acc, segment_times);
    gl_traj.init();
    double global_duration_ = gl_traj.getTimeSum();
    ROS_INFO("global_duration = %f s", global_duration_);

    while (play_trajectory == false && ros::ok())
    {
        ROS_INFO("Waiting for the drone to arrive at the HOVER state before starting the trajectory...");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    ROS_INFO("Start to play trajectory");
    do
    {
        double last_yaw = 0;
        emnv_ctl_bridge::PvayCommand cmd;
        cmd.header.frame_id = "world";
        double ts = 0.1; // time step for trajectory evaluation
        int index = 0;
        for (double t = 0; t < global_duration_; t += ts)
        {
            if (ros::ok() == false)
            {
                break;
            }
            index++;
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            Eigen::Vector3d pt_v = gl_traj.evaluateVel(t);
            Eigen::Vector3d pt_a = gl_traj.evaluateAcc(t);


            double yaw = last_yaw;
            if (t + 0.5 > global_duration_)
            {
                yaw = last_yaw;
            }
            else
            {
                Eigen::Vector3d pt_next = gl_traj.evaluate(t + 0.5);
                Eigen::Vector3d pt_next_v = gl_traj.evaluateVel(t + 0.5);
                Eigen::Vector3d pt_next_a = gl_traj.evaluateAcc(t + 0.5);


                // 计算 yaw_vector
                Eigen::Vector3d yaw_vector = pt_next - pt;
                double vector_length = yaw_vector.norm();
                if (vector_length >= 0.1)
                {
                    yaw = std::atan2(yaw_vector[1], yaw_vector[0]);
                }
            }

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
            cmd.yaw = yaw;
            // cmd.yaw = 0;

            // 计算 yaw 角
            cmd_pub.publish(cmd);
            last_yaw = yaw;

            path.header.frame_id = "world";
            path.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped cur_pos;
            cur_pos.header.frame_id = "world";
            cur_pos.header.stamp = ros::Time::now();
            cur_pos.header.seq = index;
            cur_pos.pose.position.x = pt(0);
            cur_pos.pose.position.y = pt(1);
            cur_pos.pose.position.z = pt(2);
            path.poses.push_back(cur_pos);
            paths_pub.publish(path);

            // pub cmd quadrotor_msgs::PositionCommand
            ros::Duration(ts).sleep();
            ros::spinOnce();
        }
    } while (ros::ok() && is_loop_mode);
    return 0;
}