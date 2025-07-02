# 关于group

为了方便仿真时多机使用，mavutils订阅的mavros话题均会自动加入group前缀

# 悬停油门估计

> 在CmdPubType 不是 POSY时，需要估计悬停油门。油门的估计依赖于
> - 归一化油门的z轴方向分量
> - Z轴方向的加速度

悬停油门估计使用了单变量EKF, $a_z^{meas}$是世界坐标系下测量的z轴加速度，$u_{hover}$ 是悬停油门(0~1)u是归一化油门，$g$是重力加速度，测量方程为
$$
a_z^{meas}  =  \frac{u}{u_{hover}}g+noise
$$

在代码中
- 从`/mavros/imu/data`拿到的加速度的是baselink坐标系下的，通过坐标系转换获得世界坐标系下的z轴测量值。
- u 使用进入混控器前的油门设定值

估计模块留出了一些设置接口:

> 参考:px4悬停油门估计


# 降落判定

# 起飞降落指令

```bash
# source src/control_for_gym/Tools/help_func.sh # 默认已经包含在了 devel/setup.bash 中
takeoff drone
land drone
# "drone" 是一个参数，在launch文件中配置
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