# 环境配置
> 兼容环境 ubuntu20.04 ROS1

- ros安装
- Px4 源码编译 (可选，如果需要基于gazebo的软件在环控制) 
- ros pkg 安装


## ros noetic 安装[(tsinghua mirror)](https://mirrors.tuna.tsinghua.edu.cn/help/ros/)
新建 `/etc/apt/sources.list.d/ros-latest.list`，内容为：
```bash
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main
```

然后再输入如下命令，信任 ROS 的 GPG Key，并更新索引：

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full

# 安装必要的包
sudo apt install ros-noetic-mavros*
cd /opt/ros/noetic/lib/mavros 
sudo ./install_geographiclib_datasets.sh
```

## px4 编译

- [环境准备](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- [源码构建](https://docs.px4.io/main/en/dev_setup/building_px4.html)

<!-- 在开始之前需要你完成px4源码和ros的配置 -->
## Ctrl_Bridge 编译


```bash
mkdir -p ctrl_bridge_ws/src
cd ctrl_bridge_wss/src
git clone https://github.com/emNavi/emnv_ctl_bridge.git
cd ..
catkin_make
```

<!-- 
### Eigen 库找不到
```
find_package(Eigen3 REQUIRED) # try to find manually installed eigen (Usually in /usr/local with provided FindEigen3.cmake)
message("Eigen lib find")

message(${EIGEN3_INCLUDE_DIRS})
# 头文件目录为 EIGEN3_INCLUDE_DIRS ，不要用错

``` -->