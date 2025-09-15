#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandLong, CommandLongRequest
import subprocess

def reboot_fcu():
    rospy.init_node('reboot_fcu', anonymous=True)
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        req = CommandLongRequest()
        req.command = 246          # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        req.param1  = 1            # 1 -> 重启飞控
        req.param2  = 0            # 0 -> 不重启伴飞计算机
        req.param3  = 0            # 保留
        req.param4  = 0
        req.param5  = 0
        req.param6  = 0
        req.param7  = 0
        resp = cmd(req)
        if resp.success:
            rospy.loginfo("Reboot command accepted!")
        else:
            rospy.logwarn("Reboot command rejected, result=%d", resp.result)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    reboot_fcu()
    # subprocess.call(['rosnode', 'kill', '-a'])
    # rospy.spin()