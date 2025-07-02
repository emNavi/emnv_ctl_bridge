# Px4 编译


> 需要编译 1.15以及以上

参考资料
- https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html


## 配置环境

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

```

## docker 环境

- 开始前请确保已经安装了docker

### px4 源码编译

> 支持1.13.3

```bash
make px4_sitl gazebo-classic
```


## protobuf 不兼容问题

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