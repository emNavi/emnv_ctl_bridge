# 在真实场景中使用

### 位姿数据pipeline
真实场景中的使用涉及了位姿数据的传递， 我们提供了多种方式
**类GPS场景（state mode 0）** 
此模式下
```
GPS -> px4 ->emnavi_ctl_bridge(local_pos)
```

**VIO 或 LIO 场景（state mode 1）** 

> 此模式下无法使用 官方原生固件，需要在ekf中关闭imu bias 估计，否则将造成系统崩溃

```
                      emnavi_ctl_bridge(local_pos)
                        ^
                        |
VIO/LIO(vision_pose) -> px4 
```
**VIO 或 LIO 场景（state mode 2）** 
```
VIO/LIO(vison_pose) -> emnavi_ctl_bridge -- Ctl signal --> px4
```
