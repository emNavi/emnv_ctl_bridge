#ifndef _FSM_HPP_
#define _FSM_HPP_
#include <ros/ros.h>
#include <ros/time.h>
#include <unordered_map>
#include <string>
class CtrlFSM
{
public:
    enum State_t
    {
        IDLE = 1,
        INITIAL,
        TAKEOFF,
        HOVER,
        RUNNING,
        LANDING
    };
    std::unordered_map<State_t, std::string> stateStringMap = {
        {IDLE, "IDLE"},
        {INITIAL, "INITIAL"},
        {TAKEOFF, "TAKEOFF"},
        {HOVER, "HOVER"},
        {RUNNING, "RUNNING"},
        {LANDING, "LANDING"}
    };

    State_t now_state = IDLE;
    State_t last_state = IDLE;
    ros::Time last_try_offboard_time;
    ros::Time last_try_arm_time;
    ros::Time last_try_land_time;

    std::unordered_map<std::string, std::pair<bool, bool>> flags{
        // default , current
        {"offboard_done", {false, false}},  
        {"cmd_vaild", {false, false}},      
        {"arm_done", {false, false}},
        {"land_done", {false, false}},   
        {"recv_land_cmd", {false, false}},    
        {"recv_takeoff_cmd", {false, false}},        
        {"takeoff_done", {false, false}}
    };


    bool getFlag(const std::string& name) const {
        auto it = flags.find(name);
        if (it != flags.end()) {
            return it->second.second;
        }
        std::cerr << "Warning: Flag '" << name << "' not found!\n";
        return false;
    }
    // 设置 flag 当前值
    void setFlag(const std::string& name, bool value) {
        if (flags.find(name) != flags.end()) {
            flags[name].second = value;
        } else {
            std::cerr << "Error: Trying to set an undefined flag '" << name << "'!\n";
        }
    }
    // 重置所有 flag 为默认值
    void resetFlags() {
        for (auto& flag : flags) {
            flag.second.second = flag.second.first; // 恢复到默认值
        }
    }

    void Init_FSM()
    {
        now_state = IDLE;
        last_state = IDLE;
        resetFlags();
#define TIME_OFFSET_SEC 1000
        last_recv_pva_time = ros::Time::now() - ros::Duration(TIME_OFFSET_SEC);
        last_try_offboard_time = ros::Time::now() - ros::Duration(TIME_OFFSET_SEC);
        last_try_arm_time = ros::Time::now() - ros::Duration(4);
        last_try_land_time = ros::Time::now() - ros::Duration(TIME_OFFSET_SEC);
        ROS_INFO("init FSM");
    }

    inline void process()
    {
        // 状态切换只能是状态切换
        last_state = now_state;
        switch (now_state)
        {
        case IDLE:
        {
            if(getFlag("recv_takeoff_cmd"))
            {
                now_state = INITIAL;
            }
            break;
        }
        case INITIAL:
        {
            if(getFlag("arm_done") && getFlag("offboard_done"))
            {
                now_state = TAKEOFF;
            }
            break;
        }
        case TAKEOFF:
        {
            if (getFlag("takeoff_done"))
            {
                now_state = HOVER;
            }
            break;
        }
        case HOVER:
        {

            if (getFlag("cmd_vaild"))
            {
                now_state = RUNNING;
            }

            if (getFlag("recv_land_cmd"))
            {
                now_state = LANDING;
            }

            break;
        }
        case RUNNING:
        {
            if (!getFlag("cmd_vaild"))
            {
                now_state = HOVER;
            }

            if (getFlag("recv_land_cmd"))
            {
                now_state = LANDING;
            }
            break;
        }
        case LANDING:
        {
            if(getFlag("land_done"))
            {
                now_state = IDLE;
            }
        }
        default:
            break;
        }
        if (last_state != IDLE && now_state == IDLE)
        {
            resetFlags();
        }
        modeCheckLog();
    }
    std::string stateToString(State_t state) {
        auto it = stateStringMap.find(state);
        if (it != stateStringMap.end()) {
            return it->second;
        } else {
            return "UNKNOWN"; // 处理未知 enum 值
        }
    }
    void modeCheckLog()
    {
        if (now_state != last_state)
        {
            ROS_INFO("FSM: %s -> %s", stateToString(last_state).c_str(),stateToString(now_state).c_str());
        }
    }

    void updateCtrlCmdTimestamp(ros::Time now_time)
    {
        last_recv_pva_time = now_time;
    }
    bool isCmdVaild()
    {

        if (ros::Time::now() - last_recv_pva_time < ros::Duration(1.0))
        {
            return true;
        }
        else
        {
            if(static_cast<int>((ros::Time::now() - last_recv_pva_time).toSec())/3 == 0)
            {
                ROS_INFO_STREAM("cmd timeout(s) "<<ros::Time::now() - last_recv_pva_time);
            }
            return false;
        }
    }

private:
    ros::Time last_recv_pva_time;

    // add takeoff cmd recv flag;
};

#endif