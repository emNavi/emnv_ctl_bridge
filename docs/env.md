# Environment Setup
> Compatible environment: Ubuntu 20.04, ROS1

- ROS installation
- PX4 source code build (optional, required for Gazebo software-in-the-loop control)
- ROS package installation


## ROS Noetic Installation [(Tsinghua Mirror)](https://mirrors.tuna.tsinghua.edu.cn/help/ros/)
Create `/etc/apt/sources.list.d/ros-latest.list` with the following content:
```bash
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main
```

Then run the following commands to trust the ROS GPG key and update the package index:

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full

# Install required packages
sudo apt install ros-noetic-mavros*
cd /opt/ros/noetic/lib/mavros 
sudo ./install_geographiclib_datasets.sh
```

## PX4 Build

- [Environment Setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- [Source Build](https://docs.px4.io/main/en/dev_setup/building_px4.html)

<!-- Before starting, you need to complete the PX4 source code and ROS configuration -->
## Ctrl_Bridge Build


```bash
mkdir -p ctrl_bridge_ws/src
cd ctrl_bridge_wss/src
git clone https://github.com/emNavi/emnv_ctl_bridge.git
cd ..
catkin_make
```

<!-- 
### Eigen Library Not Found
```
find_package(Eigen3 REQUIRED) # try to find manually installed eigen (Usually in /usr/local with provided FindEigen3.cmake)
message("Eigen lib find")

message(${EIGEN3_INCLUDE_DIRS})
# The header directory is EIGEN3_INCLUDE_DIRS, don't use the wrong variable

``` -->