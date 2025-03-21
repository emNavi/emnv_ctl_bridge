# emnavi ctl bridge

emnavi_ctl_bridge 是介于 px4 和 用户算法之间的中间件。

> 兼容环境 ubuntu20.04

## 使用gazebo 控制

在开始之前需要你完成px4源码和ros的配置

### px4 源码编译

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh  
```

### ros 安装

> 国内安装 noetic


新建 `/etc/apt/sources.list.d/ros-latest.list`，内容为：
```bash
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main
```

然后再输入如下命令，信任 ROS 的 GPG Key，并更新索引：

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
```

## 在真实场景中使用

### 位姿数据pipeline
真实场景中的使用涉及了位姿数据的传递， 我们提供了多种方式
**类GPS场景（state mode 0）** 
此模式下
```
GPS -> px4 ->emnavi_ctl_bridge(local_pos)
```

**VIO 或 LIO 场景（state mode 1）** 

> 此模式下无法使用 官方原生固件，需要在ekf中关闭imu bias 估计，否则将造成系统崩溃

```
                      emnavi_ctl_bridge(local_pos)
                        ^
                        |
VIO/LIO(vision_pose) -> px4 
```
**VIO 或 LIO 场景（state mode 2）** 
```
VIO/LIO(vison_pose) -> emnavi_ctl_bridge -- Ctl signal --> px4
```





control_for_gym is a PX4 middleware stack to publish NN-inferenced commands to expected levels of PX4 control loop. It is integrated in `AirGym-Real` for sending commands to PX4 autopilot during the Sim-to-Real.

Two functions are primarily implemented: constructing a finite state machine (FSM) to enable switching between trained policy control and classical PID control; and forwarding control commands to the PX4 Autopilot controller based on the selected control hierarchy.

## Control bridge

- Four types of controls are implemented:
  - VEL + YAW
  - POS + YAW
  - ATTI + THRUST
  - RATE + THRUST
 
- PID controls in FSM:
  - Takeoff
  - Hovering

Use channal 8 to switch classic control and nerual network inference control.