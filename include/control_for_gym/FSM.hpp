#ifndef _FSM_HPP_
#define _FSM_HPP_
#include <ros/ros.h>
#include <ros/time.h>
class CtrlFSM
{
public:
    enum State_t
    {
        INITIAL = 1,
        TAKEOFF,
        HOVER,
        RUNNING,
        LANDING
    };
    State_t now_state = INITIAL;
    State_t last_state = INITIAL;
    ros::Time last_try_offboard_time;
    ros::Time last_try_arm_time;
    ros::Time last_try_land_time;
    void Init_FSM()
    {
        now_state = INITIAL;
        last_state = INITIAL;
        offboard_flag = false;
        takeoff_over_flag = false;
        land_flag = false;
        cmd_vaild_flag = false;
        arm_done_flag = false;

#define TIME_OFFSET_SEC 1000
        last_recv_pva_time = ros::Time::now() - ros::Duration(TIME_OFFSET_SEC);
        last_try_offboard_time = ros::Time::now() - ros::Duration(TIME_OFFSET_SEC);
        last_try_arm_time = ros::Time::now() - ros::Duration(4);
        last_try_land_time = ros::Time::now() - ros::Duration(TIME_OFFSET_SEC);
        ROS_INFO("init FSM");
    }

    inline void process()
    {
        last_state = now_state;
        switch (now_state)
        {
        case INITIAL:
        {
            if (getArmFlag() && getOffboardFlag())
            {
                now_state = TAKEOFF;
            }
            break;
        }
        case TAKEOFF:
        {
            if (getTakeoffOverFlag())
            {
                now_state = HOVER;
            }
            break;
        }
        case HOVER:
        {
            if (isCmdVaild())
            {
                now_state = RUNNING;
            }

            if (getLandFlag())
            {
                now_state = LANDING;
            }

            break;
        }

        case RUNNING:
        {
            if (!isCmdVaild())
            {
                now_state = HOVER;
            }

            if (getLandFlag())
            {
                now_state = LANDING;
            }
            break;
        }

        default:
            break;
        }
    }
    // ********************************
    bool getOffboardFlag()
    {
        return offboard_flag;
    }
    void setOffboardFlag(bool flag)
    {
        offboard_flag = flag;
    }

    bool getTakeoffOverFlag()
    {
        return takeoff_over_flag;
    }
    void setTakeoffOverFlag(bool flag)
    {
        takeoff_over_flag = flag;
    }

    bool getArmFlag()
    {
        return arm_done_flag;
    }
    void setArmFlag(bool flag)
    {
        arm_done_flag = flag;
    }

    bool getLandFlag()
    {
        return land_flag;
    }

    void setLandFlag(bool flag)
    {
        land_flag = flag;
    }
    // ********************************

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
    bool offboard_flag;
    bool takeoff_over_flag;
    bool cmd_vaild_flag;
    bool arm_done_flag;
    bool land_flag;
    ros::Time last_recv_pva_time;

    // add takeoff cmd recv flag;
};

#endif