#!/usr/bin/env python3

import numpy as np
from emnv_ctl_bridge.msg import PvayCommand
from geometry_msgs.msg import Point, Vector3
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy
from std_msgs.msg import String
class Lemniscate3D:
    def __init__(self, max_xy=2, a=1.0, z_range=1.0,z_base_h=1.5, t_max=2 * np.pi, dt=0.01):
        self.a = a      # XY平面形状控制
        self.max_xy = max_xy
        self.c = z_range      # Z轴振幅
        self.z_base_h = z_base_h  # Z轴基准高度
        self.dt = dt    # 采样步长
        self.t_max = t_max
        self.t = np.pi/2# 从 -t_max 开始，形成对称“∞”字
        self.finished = False
        self.last_yaw = 0.0  # 上一个yaw角度

    def _r(self, t):
        """位置"""
        a, c = self.a, self.c
        x = a * np.cos(t) / (1 + np.sin(t)**2)
        y = a * np.cos(t) * np.sin(t) / (1 + np.sin(t)**2)
        z = c * np.sin(t)+ self.z_base_h
        return np.array([x, y, z])

    def _dr(self, t):
        """一阶导数（速度）"""
        a, c = self.a, self.c
        sin_t = np.sin(t)
        cos_t = np.cos(t)
        denom = (1 + sin_t**2)
        d_denom = 2 * sin_t * cos_t

        dx = (-a * sin_t * denom - a * cos_t * d_denom) / denom**2
        # dy = (a * cos_t**2 * denom + a * sin_t * cos_t * denom - a * cos_t * sin_t * d_denom) / denom**2
        dy = (a * (cos_t**2 - sin_t**2) * denom - 2 * a * cos_t**2 * sin_t**2) / denom**2
        dz = c * cos_t
        return np.array([dx, dy, dz])

    def _ddr(self, t):
        """二阶导数（加速度）——用数值微分估计"""
        eps = 1e-3
        v1 = self._dr(t - eps)
        v2 = self._dr(t + eps)
        return (v2 - v1) / (2 * eps)

    def get_point(self,now_t):
        """获取下一个采样点"""
        if now_t > self.t_max:
            self.finished = True
            return None

        r = self._r(now_t)
        v = self._dr(now_t)
        a = self._ddr(now_t)

        result = {
            "t": self.t,
            "position": r.tolist(),
            "velocity": v.tolist(),
            "acceleration": a.tolist()
        }

        return result

def get_yaw_from_points(point, next_point,last_yaw):
    """计算yaw角度"""
    if not (isinstance(point, (list, np.ndarray)) and isinstance(next_point, (list, np.ndarray))):
        raise ValueError("Both point and next_point must be list or numpy array of length 3.")
    if len(point) != 3 or len(next_point) != 3:
        raise ValueError("Both point and next_point must have length 3.")
    yaw_vector = np.array(next_point) - np.array(point)
    vector_length = np.linalg.norm(yaw_vector[:2])
    if vector_length >= 0.1:
        return np.arctan2(yaw_vector[1], yaw_vector[0])
    return last_yaw

if __name__ == "__main__":
    rospy.init_node("lemniscate_publisher")

    a = rospy.get_param("~a", 3.0)  # XY平面形状控制参数
    z_range = rospy.get_param("~z_range", 1.0)  # Z轴振幅
    z_base_h = rospy.get_param("~z_base_h", 1.5)  # Z轴基准高度
    t_max = rospy.get_param("~t_max", 160 * np.pi)  # 最大时间
    dt = rospy.get_param("~dt", 0.01)
    just_view = rospy.get_param("~just_view", False)

    lem = Lemniscate3D(a=a, z_range=z_range,z_base_h = z_base_h , t_max=t_max, dt=dt)

    ready2publish =False
    pub_point = rospy.Publisher("lemniscate_point", PvayCommand, queue_size=10)
    path_pub = rospy.Publisher("lemniscate_path", Path, queue_size=1, latch=True)

    def bridge_status_callback(msg):
        global ready2publish
        # rospy.loginfo(f"Bridge status: {msg.data}")
        if msg.data == "HOVER":
            rospy.loginfo("Bridge is ready to publish lemniscate points.")
            ready2publish = True

    sub_status = rospy.Subscriber("bridge_status", String, bridge_status_callback)
    while not rospy.is_shutdown():
        if(ready2publish):
            break
        time.sleep(1)  # 等待订阅器准备好
    current_time = 0.0
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "world"

    point = lem.get_point(0.0)
    point_next = lem.get_point(0.0 + dt)



    guide_time = 4.0  # 引导时间
    if point is None:
        rospy.logwarn("No points available in the lemniscate trajectory.")
        rospy.signal_shutdown("No points available in the lemniscate trajectory.")
    for i in range(0, int(guide_time / dt)):
        yaw = get_yaw_from_points(point['position'], point_next['position'],lem.last_yaw)
        if(just_view == False):
            l_cmd = PvayCommand()
            l_cmd.header.stamp = rospy.Time.now()
            l_cmd.header.frame_id = "world"
            l_cmd.position = Point(*point['position'])
            l_cmd.velocity = Vector3(0.0, 0.0, 0.0)  # 初始速度为0
            l_cmd.acceleration = Vector3(0.0, 0.0, 0.0)  # 初始加速度为0
            l_cmd.yaw = yaw
            pub_point.publish(l_cmd)
            rospy.sleep(dt)
    # 正式发布
    current_time = 0.0
    while not rospy.is_shutdown():
        point = lem.get_point(current_time)
        if point is not None:
            next_point = lem.get_point(current_time + 0.3)
            yaw = get_yaw_from_points(point['position'], next_point['position'],lem.last_yaw)
            if(just_view == False):
                l_cmd = PvayCommand()
                l_cmd.header.stamp = rospy.Time.now()
                l_cmd.header.frame_id = "world"
                l_cmd.position = Point(*point['position'])
                l_cmd.velocity = Vector3(*point['velocity'])
                l_cmd.acceleration = Vector3(*point['acceleration'])
                l_cmd.yaw = yaw
                pub_point.publish(l_cmd)
            # 发布路径消息
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"
            pose.pose.position = Point(*point['position'])
            path_msg.poses.append(pose)
            if len(path_msg.poses) > 100* 2:  # 限制路径长度
                path_msg.poses.pop(0)
            path_pub.publish(path_msg)
        current_time = current_time + dt
        rospy.sleep(dt)