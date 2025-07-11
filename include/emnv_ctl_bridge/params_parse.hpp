#ifndef __PARAMS_PARSE_HPP_
#define __PARAMS_PARSE_HPP_
#include <Eigen/Eigen>
struct ParamsParse
{
    std::string ctrl_pub_level;
    std::string ctrl_mode;
    std::string ros_namespace;
    std::string drone_config_path; // 机型配置文件路径
    double takeoff_height;
    int drone_id;
    double loop_rate;
    std::string name;
    bool enable_vel_transpose_b2w = false; // 是否启用机体坐标系到世界坐标系的速度转换
    bool enable_imu_dt_check = true;
    bool enable_odom_timeout_check = true;
};

#endif
