# 使用细节说明


## 状态机
ctrl_bridge 提供了一个上层状态机，状态会发送到`/ctrl_bridge/bridge_status`话题下。
降落后状态机重置，状态回到IDLE 等待下一次起飞。
> 一些里程计如vins在飞行一轮后，不再具备起飞条件，需要重启里程计


## 设置odom信息

通过在launch文件中修改 ref_odom 可以使用
- VIO，LIO的里程计信息
- /mavros/local_position/odom

## 控制

### ctrl_mode

定义 ctrl_mode 是上层的规控系统提供给ctl_bridge的控制指令类型，可选择 `POSY,ATTI,RATE`
- POSY 是最常见的 pvay
- ATTI 即规控系统提供 四元数姿态和推力设定值
- RATE 即规控系统提供 三轴角速度和推力设定值


### cmd_out_level
cmd_out_level 是 ctrl_bridge 发送给px4的指令类型可选择
- `POSY`: 期望位置 + 期望速度 + 期望加速度 + yaw + yaw_speed (3+3+3+1+1) 位置环工作在Px4上
- `ATTI`: 四元数姿态+归一化的油门推力(4+1)
- `RATE`: 机体坐标系下的角速度+归一化的油门推力(3+1)

> 需要注意的是，使用POSY模式时需要 local_position 输出30hz 的有效值(即Px4获取到了有效的位置信息)。

### ctrl_mode 与cmd_out_level的依赖关系

- ctrl_mode为`RATE` 时，cmd_out_level 仅能为`RATE`
- ctrl_mode为`ATTI` 时，cmd_out_level 可为`ATTI|RATE`
- ctrl_mode为`POSY` 时，cmd_out_level 可为`POSY|ATTI|RATE`

当对应关系有问题时程序退出，并抛出以下错误
```bash
[ERROR] [1751528439.591269879]: Invalid correspondence between "ctrl_mode" and "ctrl_level"
```

1. 当 ctrl_mode为`POSY`，cmd_out_level 为 `POSY`时，控制由px4计算。
1. 当 ctrl_mode为`ATTI`，cmd_out_level 为 `ATTI`时，控制由px4计算。
1. 当 ctrl_mode为`RATE`，cmd_out_level 为 `RATE`时，控制由px4计算。
1. 当 ctrl_mode为`POSY`，cmd_out_level 为 `ATTI`时，ctrl_bridge中的linear_controller和会生效，即位置环由ctrl_bridge计算。
1. 当 ctrl_mode为`POSY`，cmd_out_level 为 `RATE`时，ctrl_bridge中的linear_controller和AttitudeController会生效，即位置环和姿态环由ctrl_bridge计算。
1. 当 ctrl_mode为`ATTI`，cmd_out_level 为 `RATE`时，ctrl_bridge中的AttitudeController生效，即姿态环由ctrl_bridge计算。





### 降落判定
- 连续2s降落速度小于`-0.1m/s`
- 当悬停油门估计生效时，连续2s悬停油门估计值小于`0.1`
### 起飞降落指令

ctl bridge 提供快速起飞与降落的功能
  - /ctrl_bridge/takeoff Bool msg
  - /ctrl_bridge/land  Bool msg
```bash
# source src/control_for_gym/Tools/help_func.sh # 默认已经包含在了 devel/setup.bash 中
source devel/setup.bash
takeoff drone
land drone
# "drone" 是一个参数（drone_name），在launch文件中配置
# 你也可以一次起飞或降落多个飞机
takeoff drone1,drone2
land drone1,drone2
```

起飞降落阶段具有以下特性
> 1. 在起飞与降落阶段，用户上层规控程序不介入控制(规控控制指令将被ctrl_bridge拒绝)
> 2. 起飞与降落指令互斥，即仅以最后收到的指令类型为准
> 3. 起飞过程能被降落直接打断



## 状态估计
### 悬停油门估计

若cmd_out_level设置为`ATTI`或`RATE`时，ctl_bridge需要完成悬停油门估计的任务。
> 油门的估计依赖于
> - 归一化油门的world坐标系下z轴方向分量
> - world坐标系下Z轴方向的加速度

悬停油门估计使用了单变量EKF, $a_z^{meas}$是世界坐标系下测量的z轴加速度，$u_{hover}$ 是悬停油门(0~1)u是归一化油门，$g$是重力加速度，测量方程为
$$
a_z^{meas}  =  \frac{u}{u_{hover}}g+noise
$$




在代码中
- 从`/mavros/imu/data`拿到的加速度的是baselink坐标系下的，通过坐标系转换获得世界坐标系下的z轴测量值。
- u 使用进入混控器前的油门设定值

估计模块留出了一些设置接口:
```yaml
hover_thrust_ekf:
  init_hover_thrust: 0.6
  hover_thrust_max: 0.8
  hover_thrust_noise: 0.1
  process_noise: 0.0036
```
- init_hover_thrust： 即估计的悬停油门初值，若不清楚可设为0.1，设置小了，起飞过程会比较缓慢，设置大了，起飞过程可能过冲。若cmd_out_level设置为`ATTI`或`RATE`,那么可以在起飞并成功悬停后通过`rostopic echo /ctrl_bridge/hover_thrust`获得实时的悬停油门估计
- hover_thrust_max: 悬停油门估计值的上限
- hover_thrust_noise: 悬停油门噪声
- process_noise： imu加速度测量噪声

> 参考:px4悬停油门估计

<!-- 
## 遥控器控制
### 遥控器强制降落
TODO
- 可以设置一个拨杆用于切换 程序控制和遥控器控制，我们默认你有一个拨杆被设置成了cmd_valid 开关，若状态估计正常，你可以使用开关强制切换成降落模式。 -->



## 多机部署

针对仿真时多机使用，ctrl_bridge适配了`<group>` ，mavutils订阅的mavros话题均会自动加入group前缀


## 轨迹生成模块
### 五次多项式轨迹
可以生成五次多项式轨迹方便快速测试控制效果。需要注意的是
- 轨迹不具备避障功能
- 轨迹不考虑实际场地约束，即生成轨迹形状仅依赖关键点和执行时间设置
- 更多使用请参考[轨迹生成模块](./docs/ploy_traj.md)
### lemniscate
生成8字轨迹
