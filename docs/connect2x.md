# connect2x

## Installing Dependencies

```bash
sudo apt install python3-zmq
pip install tabulate
```
conn2x uses `224.0.0.1` as the multicast address for automatic device discovery, with a default port of `32946` (configurable in the config file). The multi-device communication part uses ZMQ one-to-many publishing, with a default port of `32945` (also configurable in the config file).



conn2x provides three message forwarding modes:
1. `share_topic`: both sends messages to other devices and receives messages from them
2. `pub_only`: only sends messages to other devices
3. `sub_only`: only receives messages from other devices

> For `pub_only` mode, a common use case is aggregating data on a single master device. Therefore, when publishing, a prefix is added to all `pub_only` topics. For example, if you want to visualize the positions of all drones in rviz on a single host, and the drone positions are obtained from `/mavros/local_position/pose`, you can set `zmq_pub_only_topic_prefix` to `/dronex`. The receiver will then receive the topic as `/dronex/mavros/local_position/pose`.



> Note: ROS1 nodes cannot determine whether a received message was originally sent by themselves. Therefore, if you use the same topic for sharing, messages will loop indefinitely. For `share_topic`, if the published message is `/msg1`, the receiver should receive it as `/conn2x/msg1`.