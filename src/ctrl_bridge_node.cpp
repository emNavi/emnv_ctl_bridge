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

#include "emnv_ctl_bridge/FSM.hpp"
#include "emnv_ctl_bridge/mavros_utils.hpp"




MavrosUtils* mavros_utils_ptr = nullptr;
ParamsParse params_parse;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "emnv_ctl_bridge");
    ros::NodeHandle nh("~");
    params_parse.ros_namespace = ros::this_node::getNamespace();
    CmdPubType ctrl_pub_level;
    nh.param<double>("takeoff_height", params_parse.takeoff_height, 0.3);
    nh.param<std::string>("ctrl_pub_level", params_parse.ctrl_pub_level, "ATTI");
    nh.getParam("ctrl_mode", params_parse.ctrl_mode);

    nh.param<double>("loop_rate", params_parse.loop_rate, 100.0);
    nh.param<std::string>("drone_name", params_parse.name, "drone");

    nh.param<bool>("enable_vel_transpose_b2w", params_parse.enable_vel_transpose_b2w, false);

    nh.param<std::string>("drone_config_path", params_parse.drone_config_path, "");

    std::cout << "ctrl_pub_level " << params_parse.ctrl_pub_level << std::endl;
    std::cout << "takeoff_height" << params_parse.takeoff_height << std::endl;



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    mavros_utils_ptr = new MavrosUtils(nh, params_parse);
    mavros_utils_ptr->waitConnected();
    mavros_utils_ptr->ctrl_loop();
    return 0;
}
