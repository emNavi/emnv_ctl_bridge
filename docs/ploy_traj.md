## traj replay
### record

> 当前轨迹仅支持 POSY 控制模式
> 数据默认存储在 ~/Document/ctrl_bridge/replay_traj/ 下

数据录制时，轨迹被存储在 YYMMDDhhmmss.csv.active中。录制程序退出完成时，数据文件被重命名为 YYMMDDhhmmss.csv

### 重构轨迹

我们使用五次多项式重构轨迹并重采样轨迹为YYMMDDhhmmss_ploy.csv