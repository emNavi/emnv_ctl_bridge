# Gazebo px4

通过软件在环快速测试代码
```
gazebo(udp port [11455,,]) -> mavros (fcu_url:udp) -> ctrl_bridge  -> User Algorithm
```


# 最小Gazebo环境测试


 开始前你需要[编译px4](./Px4_Compile.md)

```bash
make px4_sitl gazebo
```

现在在ros_ws下 
```bash
catkin_make
source devel/setup.bash

roslaunch emnv_ctl_bridge simple_gazebo_test.launch
```

另启一个窗口
```bash
source devel/setup.bash

```

Px4 软件在环中 不同的端口不是相同的会对消息选择性发送，我们需要使用携带有加速度信息的端口


```bash
make px4_sitl gazebo-classic_iris
```


## 修改定位精度

默认情况下px4的定位精度比较差，对于对精度要求高的任务无法满足，我们可以降低仿真器中传感器的噪声以提高定位精度
### 关闭gps噪声
在`Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_gps_plugin.cpp`中，注释掉Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/gps/gps.sdf 中
```bash
<!-- <gpsNoise>true</gpsNoise> -->
```
在`Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf` 中降低噪声
```bash
<gyroscopeNoiseDensity>0.000018665</gyroscopeNoiseDensity>
<gyroscopeRandomWalk>3.8785e-08</gyroscopeRandomWalk>
<gyroscopeBiasCorrelationTime>1000.0</gyroscopeBiasCorrelationTime>
<gyroscopeTurnOnBiasSigma>0.00087</gyroscopeTurnOnBiasSigma>
<accelerometerNoiseDensity>0.000186</accelerometerNoiseDensity>
<accelerometerRandomWalk>0.00006</accelerometerRandomWalk>
<accelerometerBiasCorrelationTime>300.0</accelerometerBiasCorrelationTime>
<accelerometerTurnOnBiasSigma>0.00196</accelerometerTurnOnBiasSigma>
```
<!-- > 并不是单一参数导致的 -->

## 时间 
当gazebo环境过于复杂时，仿真速度可能会降低，由于控制循环参考的是本机时间而不是仿真时间，会导致速度ctl_bridge的控制速度不是设定值