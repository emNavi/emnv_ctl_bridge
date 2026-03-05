# PX4 Build

> Version 1.15 or above is required.

Test environment:
- Ubuntu 20.04


## Setting Up the Environment
Reference:
- https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html


```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```



### Gazebo Test

```bash
make px4_sitl gazebo-classic
```


## Troubleshooting

### Protobuf Incompatibility

```bash
In file included from /home/hao/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/CommandMotorSpeed.pb.cc:6:
/home/hao/WorkSpace1/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic/CommandMotorSpeed.pb.h:14:10: fatal error: google/protobuf/runtime_version.h: No such file or directory
   14 | #include "google/protobuf/runtime_version.h"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
```

The installed protobuf version is too new. Remove the newer version:
```
which protoc
```

### Docker Environment

- Make sure Docker is installed before proceeding.

TODO


## How to Improve Localization Accuracy

By default, PX4's localization accuracy is poor and may not meet the requirements of precision-demanding tasks. We can reduce the sensor noise in the simulator to improve localization accuracy.

### Disable GPS Noise
In `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/gps/gps.sdf`, comment out the following line:
```bash
<!-- <gpsNoise>true</gpsNoise> -->
```
In `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf`, reduce the noise values:
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
<!-- > It is not caused by a single parameter -->