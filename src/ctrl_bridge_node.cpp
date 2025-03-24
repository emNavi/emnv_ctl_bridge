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




MavrosUtils* mavros_utils_ptr = nullptr;
ParamsParse params_parse;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ctrl_bridge");
    ros::NodeHandle nh("~");

    CmdPubType ctrl_out_level;
    nh.param<int>("drone_id", params_parse.drone_id, 99);
    nh.param<double>("takeoff_height", params_parse.takeoff_height, 0.3);
    nh.param<std::string>("ctrl_out_level", params_parse.ctrl_out_level, "ATTI");
    nh.param<double>("loop_rate", params_parse.loop_rate, 100.0);
    nh.param<std::string>("name", params_parse.name, "drone");


    std::string cmd_pub_type;
    std::string ctrl_mode;
    nh.getParam("ctrl_mode", ctrl_mode);
    nh.getParam("cmd_pub_type", cmd_pub_type);

    std::cout << "ctrl_out_level " << params_parse.ctrl_out_level << std::endl;
    std::cout << "drone id " << params_parse.drone_id << std::endl;
    std::cout << "takeoff_height" << params_parse.takeoff_height << std::endl;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    mavros_utils_ptr = new MavrosUtils(nh);
    if(mavros_utils_ptr->set_bridge_mode(ctrl_mode, cmd_pub_type) < 0)
    {
        printf("set_bridge_mode error\n");
        return -1;
    }
    std::cout << "set_bridge_mode success" << std::endl;
    mavros_utils_ptr->waitConnected();
    mavros_utils_ptr->ctrl_loop();
    return 0;
}
