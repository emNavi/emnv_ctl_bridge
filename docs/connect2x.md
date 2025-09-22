# connect2x

## 安装依赖

```bash
sudo apt install python3-zmq
pip install tabulate
```
conn2x使用`224.0.0.1`作为自动发现的广播地址,端口默认为`32946`，可在config文件中调整。多机通讯的部分使用了zmq的一对多的发送，端口默认为`32945`（可在config文件中调整）。



conn2x 提供了三种消息转发模式:
1. share_topic：既要发送出去，也要接收其他设备的消息
2. pub_only: 仅将消息发送至其他设备，
2. sub_only： 仅从其他设备接收消息

> 针对pub_only 模式， 一种常用的场景是将数据汇总到一台主设备上，因此，运行在发送时为所有pub_only的topic添加一个前缀，举例来说，假如我希望在一台主机上rviz可视化所有飞机位置，飞机的位置都用`/mavros/local_position/pose` 获取，那么我们可以设置发布时的前缀`zmq_pub_only_topic_prefix`为`/dronex`,这样接收方收到的topic就是 `/dronex/mavros/local_position/pose`



> 需要注意的是: ros1节点无法判断接收的消息是不是自己刚刚发出去的，所以如果使用同一个topic共享消息那么就会不断回环，因此对于share topic,如果发布消息是`/msg1`, 那么接收方接收的消息应该是`/conn2x/msg1`。