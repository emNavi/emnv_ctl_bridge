# emnavi ctl bridge

emnavi_ctl_bridge 是介于 px4 和 用户算法之间的中间件。通过将常用功能封装，方便调试阶段的开发。

## 软件在环使用
## Command
```bash
# source src/control_for_gym/Tools/help_func.sh # 默认已经包含在了 devel/setup.bash 中
takeoff drone
land drone
# "drone" 是一个参数，在launch文件中配置
```

## Real Env
```bash
roslaunch control_for_gym ctrl_bridge.launch
```
## Gazebo Env
Connect with gazebo_sitl
```bash
roslaunch control_for_gym sitl_ctrl_bridge.launch
```

## Ctrl Mode

ctrl bridge 支持用户输入多种模式的控制指令，当输入不同模式时，对应的消息类型也会改变

> ctrl mode 与 command publish type 有依赖关系

比如


## command publish type

> 注意这里需要和 Ctrl Mode 区分开
mavros 可以通过多种指令控制飞机

不同模式的指令输入如下
- ATTI: 四元数姿态+归一化的油门推力(4+1)
- RATE: 机体坐标系下的角速度+归一化的油门推力(3+1)

- POSY_S: 期望位置 + 期望速度 + 期望加速度 + yaw + yaw_speed (3+3+3+1+1) 位置环工作在Px4上
<!-- - LMPC:  -->

> 需要注意的是，使用POSY_S模式时需要 local_position 输出30hz 的有效值(即Px4获取到了有效的位置信息)

除此之外，ctl bridge 具备自主起飞与降落的功能
  - ctrl_bridge/takeoff Bool msg
  - ctrl_bridge/land  Bool msg

## 用户控制不生效阶段

默认情况下，在起飞与降落阶段，用户程序不介入控制(用户控制指令将被ctrl_bridge拒绝)，



## 悬停油门估计

在CmdPubType 不是 POSY时，悬停油门的估计需要在
Ctrl_bridge 中实现。油门的估计取决于

- 归一化油门的z轴方向分量
- Z轴方向的加速度


## Stable MODE

当用户输入Invalid时，飞行器自动切换至自稳模式，自稳模式下需要有效的位置信息。

自稳模式下 起飞和降落阶段不可被打断


## traj replay
### record

> 当前轨迹仅支持 POSY 控制模式
> 数据默认存储在 ~/Document/ctrl_bridge/replay_traj/ 下

数据录制时，轨迹被存储在 YYMMDDhhmmss.csv.active中。录制程序退出完成时，数据文件被重命名为 YYMMDDhhmmss.csv

### 重构轨迹

我们使用五次多项式重构轨迹并重采样轨迹为YYMMDDhhmmss_ploy.csv



## 状态获取

ctrl_bridge 默认mavros打开，可以从px4中获取姿态信息

- 电压 (TODO)
- 

# 遥控器接管

- 可以设置一个拨杆用于切换 程序控制和遥控器控制，我们默认你有一个拨杆被设置成了
