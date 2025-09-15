# Gazebo px4

通过软件在环快速测试代码。px固件版本1.15.4已经通过测试。

```mermaid
graph RL
    A["gazebo(udp port:11455...)"] --> B["mavros (fcu_url:udp)"]
    B --> C[ctrl_bridge]
    C --> E[User Algorithm]
```


# 最小Gazebo环境测试


开始前你需要[编译px4](./Px4_Compile.md), 此外，为了更好的测试，你可以[修改px4仿真代码](./Px4_Compile.md#如何提高定位精度)以提高定位精度

编译运行可执行文件直接打开gazebo仿真器和px4软件在环仿真
```bash
make px4_sitl gazebo-classic_iris
```

现在在emnv_ctl_bridge的ws下，拉起emnv_ctl_bridge节点和mavros节点
```bash
catkin_make
source devel/setup.bash

roslaunch emnv_ctl_bridge 0minimal_gazebo_test.launch
```


现在在命令行中输入
```
source devel/setup.bash
# 起飞
takeoff 1

# 降落 
land 1
```
## 发送控制指令
例如
```bash
rostopic pub /traj_test/cmd emnv_ctl_bridge/PvayCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
position: {x: 1.0, y: 1.0, z: 1.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
acceleration: {x: 0.0, y: 0.0, z: 0.0}
yaw: 1.6
yaw_dot: 0.0
kx: [0.0, 0.0, 0.0]
kv: [0.0, 0.0, 0.0]
trajectory_id: 0
trajectory_flag: 0" -r 10

```

<!-- ## 端口选择
Px4 软件在环中 不同的端口不是相同的会对消息选择性发送，我们需要使用携带有加速度信息的端口  -->

以上是简单的测试
之后的测试均基于gazebo_ros进行，你需要安装 emnv_scene

<!-- 
## 时间 

当gazebo环境过于复杂时，仿真速度可能会降低，由于控制循环参考的是本机时间而不是仿真时间，会导致速度ctl_bridge的控制速度不是设定值 -->



# 使用roslaunch的Gazebo环境测试
同样，开始前你需要[编译px4](./Px4_Compile.md), 此外，为了更好的测试，你需要[修改px4仿真代码](./Px4_Compile.md#如何提高定位精度)以提高定位精度

编译完成后，你需要在 PX4-Autopilot 目录下输入以下命令

```bash
echo "export PX4_HOME=$(pwd)"  >> ~/.bashrc
```

现在打开一个新的terminal，输入
```bash
roslaunch emnv_ctl_bridge simlple_all_in_one_test.launch
```

现在在命令行中输入
```bash
source devel/setup.bash
# 起飞
takeoff 1

# 降落 
land 1
```



# Trouble Shooting
## gazebo仿真时间问题
gazebo仿真默认使用仿真时间：即开启仿真器时的时间为0。

在使用`make px4_sitl gazebo-classic_iris`开启px4_sitl时，但是由于没有启动`gazebo_ros`将仿真时间发布到`/clock`，因此在launch `emnv_ctl_bridge` 和 `mavros`时，务必将`/use_sim_time`置为false
```xml
<!-- sim time -->
<param name="/use_sim_time" type="bool" value="false"/>
```
该参数会将强制所有ros节点（包括`emnv_ctl_bridge`和`mavros`）使用真实时间。否则`emnv_ctl_bridge`将会陷入等待仿真时间的无限循环（参见`/emnv_ctl_bridge/src/ctrl_bridge_node.cpp`主循环）。

在使用roslaunch拉起`px4_sitl`, `gazebo`, `gazebo_ros`, `emnv_ctl_bridge`, `mavros`时，gazebo将会默认将仿真时间发布到`/clock`，以便其他节点订阅。

### terminate called after throwing an instance of ‘std::runtime_error’ what(): Duration is out of dual 32-bit range
程序中有某些关于ros::time的变量超出了限制或者为负，会引发这个报错。检查所有的ros::time相关定义变量查找问题。