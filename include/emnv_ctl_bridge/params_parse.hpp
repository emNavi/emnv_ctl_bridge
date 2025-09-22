#ifndef __PARAMS_PARSE_HPP_
#define __PARAMS_PARSE_HPP_
#include <Eigen/Eigen>
struct ParamsParse
{
    std::string ctrl_pub_level;
    std::string ctrl_mode;
    std::string ros_namespace;
    std::string drone_config_path; // 机型配置文件路径
    std::string ref_odom_topic; // 里程计话题
    double takeoff_height;
    int drone_id;
    double loop_rate;
    std::string name;
    bool enable_imu_dt_check = true;
    bool enable_odom_timeout_check = true;
    bool use_vrpn_convert = false; // 是否使用vrpn_convert的动捕数据
};

#endif
