# 多项式轨迹生成

## 快速开始

launch中提供了gazebo演示示例`2gazebo_traj_test.launch`
```bash
source devel/setup.bash
roslaunch 
```
## traj replay
### record

> 当前轨迹仅支持 POSY 控制模式
> 数据默认存储在 ~/Document/ctrl_bridge/replay_traj/ 下

数据录制时，轨迹被存储在 YYMMDDhhmmss.csv.active中。录制程序退出完成时，数据文件被重命名为 YYMMDDhhmmss.csv

### 重构轨迹

我们使用五次多项式重构轨迹并重采样轨迹为YYMMDDhhmmss_ploy.csv




## 轨迹跟踪
<!-- 
已知
$$
q_{curyaw}^{despr},q_{world}^{desyaw},
$$

我可以求得

$$
\begin{aligned}
q_{baselink}^{des} &= q_{baselink}^{desyaw} \times q_{desyaw}^{despr} \\
&=  q_{baselink}^{desyaw} \times (q_{curyaw}^{desyaw})^{-1}  \times q_{curyaw}^{despr} \\
\end{aligned}
$$

我需要

$$
\begin{aligned}
q_{world}^{des}&=q_{world}^{baselink}\times q_{baselink}^{des}\\
&= q_{world}^{baselink}\times(q_{world}^{baselink} )^{-1} \times q_{world}^{desyaw} \times (q_{curyaw}^{desyaw})^{-1} \times q_{curyaw}^{despr}\\
&= q_{world}^{desyaw} \times (q_{curyaw}^{desyaw})^{-1} \times q_{curyaw}^{despr}
\end{aligned}
$$ -->