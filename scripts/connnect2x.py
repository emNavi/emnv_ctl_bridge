#!/usr/bin/env python3
import zmq
import threading
import time
import rospy
import yaml
import io
import socket
import fcntl
import struct
import random
import queue
from tabulate import tabulate

# discover port 
# 224.0.0.1  32946
# zmq pub port 32945


class Connect2X:
    def __init__(self, config_file):
        
        self._parse_config(config_file)
        self.partner_ips = []  # 存储已发现设备的IP地址
        self.sub_sockets = []
        self.msg_queue = queue.Queue()
        self._init_zmq_publisher(self.pub_port)
        self._sub_ros_topic()
        # 开始搜索设备
        self._boardcast_self()
        self._find_device()



    def _parse_config(self, config_file):
        # Load and parse the YAML file
        with open(config_file, 'r') as f:
            topic_type_map = yaml.safe_load(f)
        print("Parsed topic-type mapping:")
        self.topics_dict = dict()
        for topic, msg_type in topic_type_map.get('share_topic', {}).items():
            self.topics_dict[topic] = {"type":msg_type, "zmq_pub": True, "zmq_sub": True}
        for topic, msg_type in topic_type_map.get('zmq_sub_only_topic', {}).items():
            self.topics_dict[topic] = {"type":msg_type, "zmq_pub": False, "zmq_sub": True}
        # Handle zmq_pub_only_topic_prefix if present
        pub_only_prefix = topic_type_map.get('zmq_pub_only_topic_prefix')
        if pub_only_prefix:
            self.zmq_pub_only_topic_prefix = pub_only_prefix
        else:
            self.zmq_pub_only_topic_prefix = ""
        for topic, msg_type in topic_type_map.get('zmq_pub_only_topic', {}).items():
            self.topics_dict[topic] = {"type":msg_type, "zmq_pub": True, "zmq_sub": False}
        table = [
            [topic, info["type"], info["zmq_pub"], info["zmq_sub"]]
            for topic, info in self.topics_dict.items()
        ]
        # 添加表头并打印
        print(tabulate(table, headers=["Topic", "Message Type", "isPub", "isSub"], tablefmt="grid"))

        
        self.netcard_name = topic_type_map.get('netcard_name')
        self.pub_port = topic_type_map.get('zmq_pub_port', 32945)
        self.sub_port = topic_type_map.get('zmq_sub_port', 32945)
        self.discover_port = topic_type_map.get('discover_port', 32946)
        self.ip_addr = self.get_netcard_ip(self.netcard_name)

    def _init_zmq_publisher(self, pub_port):
        """Initialize the ZMQ publisher."""
        self.context = zmq.Context()
        # Publisher socket
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://{self.ip_addr}:{pub_port}")
        t = threading.Thread(target=self._zmq_send_thread, daemon=True)
        t.start()
        
    def _add_zmq_subscriber(self, sub_ip ,sub_port):
        print(f"Connecting to device at {sub_ip}:{sub_port}")
        sub_socket = self.context.socket(zmq.SUB)
        sub_socket.connect(f"tcp://{sub_ip}:{sub_port}")
        for topic, msg_info in self.topics_dict.items():
            if(msg_info["zmq_sub"] == False):
                continue
            sub_socket.setsockopt_string(zmq.SUBSCRIBE, topic)
            print(f"Subscribed to topic: {topic}")
        self.sub_sockets.append(sub_socket)
        t = threading.Thread(target=self._zmq_receive, args=(sub_socket,), daemon=True)
        t.start()
    def _zmq_receive(self,sub_socket):
        while True:
            frames = sub_socket.recv_multipart()
            if len(frames) != 2:
                rospy.logwarn(f"Expected 2 frames, got {len(frames)} — discarding")
                print(f"Received frames: {frames}")
                return
            topic = frames[0].decode('utf-8')
            raw_msg = frames[1]
            if topic not in self.topics_dict:
                rospy.logwarn(f"Received message for unknown topic: {topic}")
                return
            msg_class = self.topic_to_class[topic]
            msg_instance = msg_class()
            print(f"Deserializing with class: {msg_class.__name__}")
            msg_instance.deserialize(raw_msg)
            self.ros_publishers[topic].publish(msg_instance)
    def _zmq_send_thread(self):
        """Thread to send messages to ZMQ."""
        while not rospy.is_shutdown():
            topic, raw_msg = self.msg_queue.get()
            self.pub_socket.send_multipart([topic.encode('utf-8'), raw_msg])
    def _sub_ros_topic(self):
        ''' 只需要一次，即订阅自己并转发'''
        self.topic_to_class = {}
        for topic, msg_info in self.topics_dict.items():
            type_str = msg_info["type"]
            try:
                pkg, msg = type_str.split('/')
                mod = __import__(pkg + '.msg', fromlist=[msg])
                self.topic_to_class[topic] = getattr(mod, msg)
            except (ImportError, AttributeError, ValueError) as e:
                rospy.logerr(f"Failed to import {type_str} for topic {topic}: {e}")
        for topic, msg_type in self.topics_dict.items():
            # 回调工厂，防止闭包变量问题
            def callback_factory(t):
                # rospy.loginfo(f"Creating callback for topic: {t}")
                def callback(msg):
                    buff = io.BytesIO()
                    msg.serialize(buff)
                    raw_msg = buff.getvalue()
                    self.msg_queue.put((t, raw_msg))

                return callback
            if(msg_type["zmq_pub"] == True):
                if(msg_type["zmq_sub"] == True):
                    rospy.Subscriber(topic, self.topic_to_class[topic], callback_factory(topic))
                else:
                    rospy.Subscriber(topic, self.topic_to_class[topic], callback_factory(self.zmq_pub_only_topic_prefix+topic), queue_size=10)
            if not hasattr(self, 'ros_publishers'):
                self.ros_publishers = {}
            if(msg_type["zmq_sub"] == True):
                if topic not in self.ros_publishers:
                    self.ros_publishers[topic] = rospy.Publisher(topic, self.topic_to_class[topic], queue_size=10)

    def get_netcard_ip(self,ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(
            fcntl.ioctl(
                s.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack('256s', ifname[:15].encode('utf-8'))
            )[20:24]
        )

    def _boardcast_self(self):
        print(f"IP address for {self.netcard_name}: {self.ip_addr}")
        def udp_broadcast():
            udp_port = 50000  # 可根据需要修改端口
            message = f"{self.ip_addr},{self.pub_port}".encode('utf-8')
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            while True:
                udp_socket.sendto(message, ('224.0.0.1', udp_port))
                time.sleep(1 + 0.2 * (random.random() - 0.5))  # 每秒广播一次，加随机抖动
        threading.Thread(target=udp_broadcast, daemon=True).start()
    
    def _find_device(self):
        """Continuously discover devices on the network in a background thread."""
        def discover_loop():
            udp_port = 50000  # 与广播端口保持一致
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            udp_socket.bind(('', udp_port))
            print("Listening for devices on UDP broadcast...")
            while True:
                try:
                    udp_socket.settimeout(2)
                    data, addr = udp_socket.recvfrom(1024)
                    msg = data.decode('utf-8')
                    # 兼容原有格式和新格式
                    if ',' in msg:
                        ip, port = msg.split(',', 1)
                        if(ip != self.ip_addr):
                            if ip not in self.partner_ips and ip!= self.ip_addr:
                                print(f"Discovered new device IP: {ip}, Port: {port}")
                                self.partner_ips.append(ip)
                                self._add_zmq_subscriber(ip, int(port))
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error in discover_loop: {e}")
                    continue

        threading.Thread(target=discover_loop, daemon=True).start()




if __name__ == "__main__":
    rospy.init_node('discover_client_node')
    config_file = rospy.get_param('~config_file', "")
    if(config_file == ""):
        exit("Please set the config_file parameter in the launch file.")
    connectx = Connect2X(config_file)
    # Discover devices and connect to them
    print("ConnectX initialized and ready.")
    rospy.spin()