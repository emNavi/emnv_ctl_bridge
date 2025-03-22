#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Float32MultiArray.h>
#include <boost/shared_ptr.hpp>

#include "ctrl_bridge/FSM.hpp"
#include "ctrl_bridge/mavros_utils.hpp"


//默认设置，可以在launch文件中修改
#define CMD_PUB_RATE 100.0
#define PUB_DEFAULT_MODE "ATTI"

enum class CtrlMode {
    QUAD_T,
    RATE_T,
    PVA_Ys,
    Unknown // 处理无效输入
};


enum class CmdPubType {
    ATTI,
    RATE,
    POSY,
    Unknown // 处理无效输入
};

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

int process_mode(ros::NodeHandle nh)
{
    std::string cmd_pub_type;
    std::string ctrl_mode;
    nh.getParam("ctrl_mode", ctrl_mode);
    nh.getParam("cmd_pub_type", cmd_pub_type);
    CmdPubType cmd_pub_type_enum = cmdPubMap[cmd_pub_type];
    CtrlMode ctrl_mode_enum = ctrlModeMap[cmd_pub_type];
    if(cmd_pub_type_enum == CmdPubType::Unknown || ctrl_mode_enum == CtrlMode::Unknown)
    {
        // 不一定管用
        ROS_ERROR("Invalid input");
        return -1;
    }

    if(cmd_pub_type_enum == CmdPubType::ATTI)
    {
        if(ctrl_mode_enum == CtrlMode::QUAD_T)
        {
            return 0;
        }
        else if (ctrl_mode_enum == CtrlMode::RATE_T)
        {
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"cmd_pub_type\"");
            return -1;
        }
    }
    else if(cmd_pub_type_enum == CmdPubType::RATE)
    {
        if (ctrl_mode_enum == CtrlMode::RATE_T || ctrl_mode_enum == CtrlMode::QUAD_T || ctrl_mode_enum == CtrlMode::PVA_Ys)
        {
            return 0;
        }
        else{
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"cmd_pub_type\"");
            return -1;
        }
    }
    else if(cmd_pub_type_enum == CmdPubType::POSY)
    {
        if (ctrl_mode_enum == CtrlMode::RATE_T || ctrl_mode_enum == CtrlMode::QUAD_T)
        {
            ROS_ERROR("Invalid correspondence between \"ctrl_mode\" and \"cmd_pub_type\"");
            return -1;
        }
    }
}




#include "control_for_gym/mavros_utils.hpp"

MavrosUtils* mavros_utils_ptr = nullptr;
ParamsParse params_parse;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ctrl_bridge");
    ros::NodeHandle nh("~");

    MavrosUtils::CTRL_OUTPUT_LVEVL ctrl_out_level;
    nh.param<int>("drone_id", params_parse.drone_id, 99);
    nh.param<double>("takeoff_height", params_parse.takeoff_height, 0.3);
    nh.param<std::string>("ctrl_out_level", params_parse.ctrl_out_level, "ATTI");
    nh.param<double>("loop_rate", params_parse.loop_rate, 100.0);
    nh.param<std::string>("name", params_parse.name, "drone");

    
    if (params_parse.ctrl_out_level == "ATTI")
    {
        ctrl_out_level = MavrosUtils::ATTI;
    }
    else if (params_parse.ctrl_out_level == "RATE")
    {
        ctrl_out_level = MavrosUtils::RATE;
    }

    std::cout << "ctrl_out_level " << params_parse.ctrl_out_level << std::endl;
    std::cout << "drone id " << params_parse.drone_id << std::endl;
    std::cout << "takeoff_height" << params_parse.takeoff_height << std::endl;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    mavros_utils_ptr = new MavrosUtils(nh,ctrl_out_level);
    mavros_utils_ptr->waitConnected();
    mavros_utils_ptr->ctrl_loop();
    return 0;
}
