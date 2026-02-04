#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# 改成你的消息包名：例如 my_robot_msgs.msg
from odometry_z_axis.msg import OdometryZAxis

import matplotlib
matplotlib.use("TkAgg")  # 常见桌面环境可用；如有问题可改 Qt5Agg 等
import matplotlib.pyplot as plt


class OdomZPlotter(Node):
    def __init__(self):
        super().__init__('odom_z_plotter')

        self.sub = self.create_subscription(
            OdometryZAxis,
            '/odom/z',
            self.cb,
            qos_profile_sensor_data
        )

        # 只保留最近 N 个点，避免内存无限增长
        self.max_points = 2000
        self.t_buf = deque(maxlen=self.max_points)
        self.z_cm_buf = deque(maxlen=self.max_points)

        self.t0 = time.time()
        self.latest_cm = None

        # matplotlib 初始化
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_title("Z height (cm) from /odom/z")
        self.ax.set_xlabel("t (s)")
        self.ax.set_ylabel("z (cm)")
        self.ax.grid(True)

        # 用 ROS 定时器刷新图（不把绘图放进回调，降低阻塞风险）
        self.timer = self.create_timer(0.05, self.update_plot)  # 20 Hz 刷新

        self.get_logger().info("Subscribed to /odom/z and plotting z in cm.")

    def cb(self, msg: OdometryZAxis):
        z_cm = msg.z_m * 100.0 + 9.0
        self.latest_cm = z_cm

        t = time.time() - self.t0
        self.t_buf.append(t)
        self.z_cm_buf.append(z_cm)

        # 输出当前高度（cm）
        # 你也可以改成 self.get_logger().info，但终端会更“刷屏”
        print(f"当前高度: {z_cm:.2f} cm (seq={msg.seq}, valid={msg.valid}, status={msg.status})")

    def update_plot(self):
        if not self.t_buf:
            return

        self.line.set_data(self.t_buf, self.z_cm_buf)
        self.ax.relim()
        self.ax.autoscale_view()

        # 右上角显示当前值
        if self.latest_cm is not None:
            self.ax.set_title(f"Z height (cm) from /odom/z | current: {self.latest_cm:.2f} cm")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = OdomZPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
