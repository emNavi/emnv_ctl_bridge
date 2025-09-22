#!/usr/bin/env python3

import rospy
import signal
import subprocess
import os
import sys

#!/usr/bin/env python3

def kill_gazebo_processes():
    # List of common gazebo-related process names
    gazebo_processes = [
        "gzserver",
        "gzclient",
        "gazebo",
        # "roslaunch",  # Sometimes gazebo is launched via roslaunch
    ]
    for proc in gazebo_processes:
        try:
            subprocess.run(
                ["pkill", "-9", "-f", proc],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            rospy.logwarn(f"Failed to kill {proc}: {e}")

def shutdown_handler(signum, frame):
    rospy.loginfo("Caught Ctrl+C, killing gazebo-related processes...")
    kill_gazebo_processes()
    rospy.loginfo("Exiting node.")
    sys.exit(0)

def main():
    rospy.init_node('gazebo_force_exit', anonymous=True)
    signal.signal(signal.SIGINT, shutdown_handler)
    rospy.loginfo("gazebo_force_exit node started. Press Ctrl+C to force kill gazebo processes.")
    rospy.spin()

if __name__ == '__main__':
    main()