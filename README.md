# emnavi ctl bridge

emnavi_ctl_bridge 是介于 px4 和 用户算法之间的中间件。通过将常用功能封装，方便调试阶段的开发。

- [环境构建](./docs/env.md)
- [软件在环使用](./docs/Gazebo_px4.md)
- [使用细节说明](./docs/guide.md)

# 特性
## 遥控器强制降落
TODO
- 可以设置一个拨杆用于切换 程序控制和遥控器控制，我们默认你有一个拨杆被设置成了cmd_valid 开关，若状态估计正常，你可以使用开关强制切换成悬停模式。
> 悬停模式下需要有效的位置信息。
## 自动状态机重置
- 降落后状态机自动重置
- 降落后可以进行再次起飞
## name自定义
- 可以设置 drone_id
## 快捷起飞降落指令
可以使用 `takeoff drone_id` 实现起飞，例如
```bash
takeoff drone1
takeoff drone1,drone2
```
也可以使用`land drone_id`实现降落
- [实机飞行](./docs/Real_Env.md)
- [多项式轨迹](./docs/ploy_traj.md)



