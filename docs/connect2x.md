# connect2x

## 安装依赖

```bash
sudo apt install python3-zmq
pip install tabulate
```

ros1节点无法判断接收的消息是不是自己刚刚发出去的，所以如果使用同一个topic共享消息那么就会不断回环，因此如果发布消息是`/msg1`, 那么接收方接收的消息应该是`/conn2x/msg1`。
