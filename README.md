# Control
## requirement
```bash
sudo apt install ros-noetic-mavros-extras
```
## ego_planner 实机使用
```bash
roslaunch single_offb_pkg swarm_single.launch drone_id:=EGO_ID
```

### Eigen 库找不到
```
find_package(Eigen3 REQUIRED) # try to find manually installed eigen (Usually in /usr/local with provided FindEigen3.cmake)
message("Eigen lib find")

message(${EIGEN3_INCLUDE_DIRS})
# 头文件目录为 EIGEN3_INCLUDE_DIRS ，不要用错

```


