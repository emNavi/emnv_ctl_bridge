#!/usr/bin/env python3

import dash
from dash import dcc, html
import dash.dependencies
import rospy
from sensor_msgs.msg import Imu
import plotly.graph_objs as go

import sys
import signal

# 定义自定义的 Ctrl+C 捕获函数
def signal_handler(sig, frame):
    rospy.loginfo("Custom Ctrl+C handler: Shutting down...")
    rospy.signal_shutdown("Ctrl+C pressed")  # 触发 ROS 的关闭机制
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


# 创建 Dash 应用
app = dash.Dash(__name__)

# 初始化数据存储
x_data, y_data, z_data = [0], [0], [0]

# ROS 回调函数，接收 IMU 数据
def imu_callback(msg):
    global x_data, y_data, z_data
    x_data.append(msg.linear_acceleration.x)
    y_data.append(msg.linear_acceleration.y)
    z_data.append(msg.linear_acceleration.z)

# 初始化 ROS 节点
rospy.init_node('dash_ros_node')
rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)

# 设置页面布局
app.layout = html.Div([
    html.H1('Real-Time IMU Data'),
    dcc.Graph(id='imu-graph'),
    dcc.Interval(
        id='graph-update',
        interval=50,  # 每秒更新一次
        n_intervals=0
    )
])

# 定义回调函数，用于更新图表
@app.callback(
    dash.dependencies.Output('imu-graph', 'figure'),
    [dash.dependencies.Input('graph-update', 'n_intervals')]
)
def update_graph(n_intervals):
    # 使用最新的 IMU 数据更新图表
    figure = {
        'data': [
            go.Scatter(
                x=list(range(len(x_data))),
                y=x_data,
                mode='lines+markers',
                name='X Axis Acceleration'
            ),
            go.Scatter(
                x=list(range(len(y_data))),
                y=y_data,
                mode='lines+markers',
                name='Y Axis Acceleration'
            ),
            go.Scatter(
                x=list(range(len(z_data))),
                y=z_data,
                mode='lines+markers',
                name='Z Axis Acceleration'
            ),
        ],
        'layout': go.Layout(
            title='IMU Data',
            xaxis=dict(title='Time'),
            yaxis=dict(title='Acceleration'),
            showlegend=True
        )
    }
    return figure

# 启动 Dash 应用
if __name__ == '__main__':
    app.run_server(debug=True, use_reloader=False)  # use_reloader=False 是为了避免重复初始化 ROS 节点
