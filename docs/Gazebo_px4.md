# Gazebo px4

```
gazebo(udp port [11455,,]) -> mavros (fcu_url:udp) -> ctrl_bridge  -> User Algorithm
```

## Px4 软件在环中 不同的端口不是相同的

- 111 不发送加速度信息 ，即不发送


```bash
make px4_sitl gazebo-classic
```