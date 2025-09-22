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


#include <ctime>


MavrosUtils* mavros_utils_ptr = nullptr;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "emnv_ctl_bridge");
    ros::NodeHandle nh("~");
    ParamsParse params_parse;
    params_parse.ros_namespace = ros::this_node::getNamespace();
    CmdPubType ctrl_pub_level;
    nh.param<double>("takeoff_height", params_parse.takeoff_height, 0.5);
    nh.param<std::string>("ctrl_pub_level", params_parse.ctrl_pub_level, "ATTI");
    nh.getParam("ctrl_mode", params_parse.ctrl_mode);

    nh.param<double>("loop_rate", params_parse.loop_rate, 100.0);
    nh.getParam("drone_id", params_parse.drone_id);

    nh.param<bool>("enable_imu_dt_check", params_parse.enable_imu_dt_check, true);
    nh.param<bool>("enable_odom_timeout_check", params_parse.enable_odom_timeout_check, true);
    nh.param<std::string>("ref_odom_topic", params_parse.ref_odom_topic, "/mavros/local_position/odom");


    nh.param<std::string>("drone_config_path", params_parse.drone_config_path, "");
    nh.param<bool>("use_vrpn_convert", params_parse.use_vrpn_convert, false);

    std::cout << "ctrl_pub_level " << params_parse.ctrl_pub_level << std::endl;
    std::cout << "takeoff_height" << params_parse.takeoff_height << std::endl;

    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    std::cout << "Current time: " 
              << now.tv_sec << " seconds, " 
              << now.tv_nsec << " nanoseconds" 
              << std::endl;
    ros::Time ros_now = ros::Time::now();
    std::cout << "Current ROS time: " 
              << ros_now.sec << " seconds, " 
              << ros_now.nsec << " nanoseconds" 
              << std::endl;

    if (ros::Time::isSimTime())
    {
        ROS_INFO("Waiting for /clock to become valid...");
        ros::Time::waitForValid();
        ROS_INFO("Clock is now valid.");
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    mavros_utils_ptr = new MavrosUtils(nh, params_parse);
    mavros_utils_ptr->waitConnected();
    mavros_utils_ptr->ctrl_loop();
    return 0;
}
