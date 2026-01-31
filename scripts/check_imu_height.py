#!/usr/bin/env python3
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from tenaxis_msg.msg import TenaxisImu

import matplotlib
import matplotlib.pyplot as plt


class RelativeAltitudePlotter(Node):
    """
    订阅 TenaxisImu / altitude_m
    - 第一次收到 altitude_m 作为零点
    - 计算相对高度
    - EMA + 可选跳变抑制 + 死区
    - 弹窗实时画曲线：红=原始，蓝=滤波
    """

    def __init__(self):
        super().__init__("plot_relative_altitude")

        # ROS 参数
        self.declare_parameter("imu_topic", "/imu/jy901b")
        self.declare_parameter("window_sec", 10.0)     # 显示最近多少秒
        self.declare_parameter("expected_hz", 100.0)   # 预计输入频率（用于缓冲大小）
        self.declare_parameter("ui_hz", 20.0)          # 界面刷新频率

        # 滤波参数
        self.declare_parameter("ema_alpha", 0.1)       # 0.05~0.2 常用
        self.declare_parameter("max_step_m", 0.03)     # 0 禁用；抑制“方波跳变”
        self.declare_parameter("deadband_m", 0.005)    # 0 禁用；抑制小抖动

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.expected_hz = float(self.get_parameter("expected_hz").value)
        self.ui_hz = float(self.get_parameter("ui_hz").value)

        self.alpha = float(self.get_parameter("ema_alpha").value)
        self.max_step_m = float(self.get_parameter("max_step_m").value)
        self.deadband_m = float(self.get_parameter("deadband_m").value)

        self.alpha = max(0.0, min(1.0, self.alpha))
        self.max_step_m = max(0.0, self.max_step_m)
        self.deadband_m = max(0.0, self.deadband_m)

        # QoS：传感器流用 BEST_EFFORT 合理
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(TenaxisImu, self.imu_topic, self.cb_imu, qos)

        # 数据
        self.alt0 = None
        self.alt_rel_filt = None

        # 最近窗口缓存
        buf_len = int(max(50, self.window_sec * self.expected_hz))
        self.t_buf = deque(maxlen=buf_len)      # 相对时间（秒）
        self.raw_buf = deque(maxlen=buf_len)    # 原始相对高度
        self.filt_buf = deque(maxlen=buf_len)   # 滤波相对高度

        self.t0_wall = None  # 用墙钟建立相对时间轴（稳定，不依赖 ROS 时间）
        self.last_rx_wall = None

        # Matplotlib 初始化（弹窗）
        self._init_plot()

        # UI 定时刷新
        self.ui_timer = self.create_timer(1.0 / max(1.0, self.ui_hz), self.on_ui_timer)

        self.get_logger().info(
            f"Plotting relative altitude from: {self.imu_topic}, "
            f"window={self.window_sec}s, ui_hz={self.ui_hz}, "
            f"ema_alpha={self.alpha}, max_step_m={self.max_step_m}, deadband_m={self.deadband_m}"
        )

    def _init_plot(self):
        # 确保交互模式
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Relative Altitude (red=raw, blue=filtered)")
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("relative altitude (m)")
        self.ax.grid(True)

        # 两条线：按你要求的颜色
        (self.line_raw,) = self.ax.plot([], [], color="red", linewidth=1.0, label="raw")
        (self.line_filt,) = self.ax.plot([], [], color="blue", linewidth=1.5, label="filtered")
        self.ax.legend(loc="upper right")

        # 让窗口弹出来
        self.fig.show()
        self.fig.canvas.draw()

    def _apply_filters(self, x: float) -> float:
        # 初始化滤波器
        if self.alt_rel_filt is None:
            self.alt_rel_filt = x
            return self.alt_rel_filt

        y = self.alt_rel_filt
        dx = x - y

        # 死区
        if self.deadband_m > 0.0 and abs(dx) < self.deadband_m:
            return y

        # 跳变抑制：限制单次变化
        if self.max_step_m > 0.0:
            if dx > self.max_step_m:
                x = y + self.max_step_m
            elif dx < -self.max_step_m:
                x = y - self.max_step_m

        # EMA
        y = y + self.alpha * (x - y)
        self.alt_rel_filt = y
        return y

    def cb_imu(self, msg: TenaxisImu):
        alt = float(msg.altitude_m)
        if not math.isfinite(alt):
            return

        now_wall = time.monotonic()
        if self.t0_wall is None:
            self.t0_wall = now_wall

        if self.alt0 is None:
            self.alt0 = alt
            self.get_logger().info(f"Altitude origin set: alt0={self.alt0:.3f} m")
            return

        alt_rel_raw = alt - self.alt0
        alt_rel_filt = self._apply_filters(alt_rel_raw)

        t_rel = now_wall - self.t0_wall
        self.t_buf.append(t_rel)
        self.raw_buf.append(alt_rel_raw)
        self.filt_buf.append(alt_rel_filt)

        self.last_rx_wall = now_wall

    def on_ui_timer(self):
        # 窗口被关闭就退出
        if not plt.fignum_exists(self.fig.number):
            self.get_logger().info("Plot window closed. Shutting down node.")
            rclpy.shutdown()
            return

        if len(self.t_buf) < 2:
            return

        t = list(self.t_buf)
        y_raw = list(self.raw_buf)
        y_filt = list(self.filt_buf)

        self.line_raw.set_data(t, y_raw)
        self.line_filt.set_data(t, y_filt)

        # x 轴跟随窗口
        t_max = t[-1]
        t_min = max(0.0, t_max - self.window_sec)
        self.ax.set_xlim(t_min, t_max)

        # y 轴自适应：用 raw/filt 合并范围，留一点边距
        y_all = y_raw + y_filt
        y_min = min(y_all)
        y_max = max(y_all)
        pad = max(0.05, 0.1 * (y_max - y_min + 1e-9))
        self.ax.set_ylim(y_min - pad, y_max + pad)

        # 刷新
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = RelativeAltitudePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
