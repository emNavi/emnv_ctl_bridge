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
