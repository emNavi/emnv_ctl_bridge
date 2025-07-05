# emnavi ctl bridge

emnavi_ctl_bridge 是介于 px4 和 用户算法之间的中间件。通过将常用功能封装，方便调试阶段的开发。

- 开始前请阅读[使用说明](./docs/guide.md)
- [环境构建](./docs/env.md)
- [软件在环使用](./docs/Gazebo_px4.md)

# 特性
## 遥控器强制降落
TODO
- 可以设置一个拨杆用于切换 程序控制和遥控器控制，我们默认你有一个拨杆被设置成了cmd_valid 开关，若状态估计正常，你可以使用开关强制切换成悬停模式。
> 悬停模式下需要有效的位置信息。
## 自动状态机重置
- 降落后状态机自动重置
- 降落后可以进行再次起飞
## name自定义
- 可以设置 drone_name
## 快捷起飞降落指令
可以使用 `takeoff drone_name` 实现起飞，例如
```bash
takeoff drone1
takeoff drone1,drone2
```
也可以使用`land drone_name`实现降落

- 起飞与降落指令互斥，即仅以最后收到的指令类型为准
- 起飞过程能被降落直接打断

## 轨迹生成模块

可以生成五次多项式轨迹方便快速测试控制效果。

- 轨迹不具备避障功能
- 轨迹不考虑实际场地约束，即生成轨迹形状仅依赖关键点和执行时间设置
- 更多使用请参考[轨迹生成模块](./docs/ploy_traj.md)

## 兼容多种odom信息

- VIO，LIO的里程计信息(TODO 差一个坐标系转换)
- /mavros/local_position/odom


## 兼容动捕信息




TODO
