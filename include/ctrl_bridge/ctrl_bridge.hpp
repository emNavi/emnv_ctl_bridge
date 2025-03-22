#include "ctrl_bridge/FSM.hpp"
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

struct action_flag
{
    bool recv_land_cmd = true;
    bool recv_takeoff_cmd = true;

};


class ctrl_bridge
{
private:
    CtrlFSM fsm;
    ros::Publisher local_raw_pub, local_linear_vel_pub;
    ros::Publisher vision_pose_pub;
    action_flag action_f;

public:
    ctrl_bridge(ros::NodeHandle &_nh);
    ~ctrl_bridge();

    // callback
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
            local_linear_vel_pub.publish(msg);
        }
    }
    void vrpn_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    { // 创建一个新的 PoseStamped 消息
        geometry_msgs::PoseStamped modified_msg;
        modified_msg.header.stamp = ros::Time::now();
        modified_msg.header.frame_id = msg->header.frame_id; // 保留原来的 frame_id

        // 对位置进行变换（例如，添加一个偏移量）
        modified_msg.pose.position.x = msg->pose.position.x / 1000.0; // 偏移量为 1.0 米
        modified_msg.pose.position.y = msg->pose.position.y / 1000.0;
        modified_msg.pose.position.z = msg->pose.position.z / 1000.0;

        // 对方向（四元数）进行变换（这里保持不变，仅作为示例）
        modified_msg.pose.orientation = msg->pose.orientation;
        vision_pose_pub.publish(modified_msg);
    }


void takeoff_cmd_cb(const std_msgs::Int32MultiArray::ConstPtr &msg, int drone_id)
{

    for (int i = 0; i < msg->data.size() + 1e-2; i++)
    {
        if (msg->data[i] == drone_id)
        {
            action_f.recv_takeoff_cmd = true;
        }
    }
}

bool land_cmd_flag = false;
void land_cmd_cb(const std_msgs::Int32MultiArray::ConstPtr &msg, int drone_id)
{
    for (int i = 0; i < msg->data.size() + 1e-2; i++)
    {
        if (msg->data[i] == drone_id)
        {
            action_f.recv_land_cmd = true;
        }
    }
}
};

ctrl_bridge::ctrl_bridge(ros::NodeHandle &_nh)
{
    int drone_id;
    double _param_takeoff_height = 0.3;
    _nh.param<int>("drone_id", drone_id, 99);
    _nh.param<double>("takeoff_height", _param_takeoff_height, 0.3);


    ros::Time hover_above_land_start_time = ros::Time::now();

    std::cout << "drone id " << drone_id << std::endl;
    std::cout << "takeoff_height" << _param_takeoff_height << std::endl;

}

ctrl_bridge::~ctrl_bridge()
{
}
