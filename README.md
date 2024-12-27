# control for gym

**Control for gym** is a middleware between MAVROS and the control program.

## requirement
```bash
sudo apt install ros-noetic-mavros*
```
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
<!-- ## 常见问题

### Eigen 库找不到
```
find_package(Eigen3 REQUIRED) # try to find manually installed eigen (Usually in /usr/local with provided FindEigen3.cmake)
message("Eigen lib find")

message(${EIGEN3_INCLUDE_DIRS})
# 头文件目录为 EIGEN3_INCLUDE_DIRS ，不要用错

``` -->