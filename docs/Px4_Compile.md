# Px4 编译

> 需要编译 1.15以及以上

测试环境
- ubuntu20.04


## 配置环境
参考资料
- https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html


```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```



### gazebo 测试

```bash
make px4_sitl gazebo-classic
```


## troubleshoot

### protobuf 不兼容问题

```bash
In file included from /home/hao/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/CommandMotorSpeed.pb.cc:6:
/home/hao/WorkSpace1/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/CommandMotorSpeed.pb.h:14:10: fatal error: google/protobuf/runtime_version.h: No such file or directory
   14 | #include "google/protobuf/runtime_version.h"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
```

因为版本太新了，需要把新的删除，
```
which protoc
```

### docker 环境

- 开始前请确保已经安装了docker

TODO


## 如何提高定位精度

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