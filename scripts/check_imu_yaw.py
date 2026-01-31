#!/usr/bin/env python3
import math
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from tenaxis_msg.msg import TenaxisImu


@dataclass
class Quat:
    w: float
    x: float
    y: float
    z: float


def quat_normalize(q: Quat) -> Quat:
    n = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
    if n < 1e-12:
        return Quat(1.0, 0.0, 0.0, 0.0)
    return Quat(q.w/n, q.x/n, q.y/n, q.z/n)


def quat_mul(a: Quat, b: Quat) -> Quat:
    return Quat(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    )


def small_angle_quat(wx, wy, wz, dt) -> Quat:
    wnorm = math.sqrt(wx*wx + wy*wy + wz*wz)
    if wnorm < 1e-12:
        return Quat(1.0, 0.0, 0.0, 0.0)
    th = wnorm * dt
    s = math.sin(th * 0.5) / wnorm
    return quat_normalize(Quat(math.cos(th*0.5), wx*s, wy*s, wz*s))


def yaw_from_quat(q: Quat) -> float:
    # ZYX yaw
    q = quat_normalize(q)
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


class PrintYaw(Node):
    def __init__(self):
        super().__init__('print_yaw_deg')

        self.declare_parameter('imu_topic', '/imu/jy901b')
        self.declare_parameter('print_hz', 10.0)          # 终端输出频率
        self.declare_parameter('max_dt', 0.2)             # gyro 积分最大 dt
        self.declare_parameter('gyro_reset_on_jump', True)
        self.declare_parameter('mag_yaw_sign_flip', False)  # 如果你想把 mag yaw 反号对齐可设 true

        self.imu_topic = self.get_parameter('imu_topic').value
        self.print_hz = float(self.get_parameter('print_hz').value)
        self.max_dt = float(self.get_parameter('max_dt').value)
        self.gyro_reset_on_jump = bool(self.get_parameter('gyro_reset_on_jump').value)
        self.mag_yaw_sign_flip = bool(self.get_parameter('mag_yaw_sign_flip').value)

        # QoS：BEST_EFFORT 兼容 SensorDataQoS
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(TenaxisImu, self.imu_topic, self.cb, qos)

        self.lock = threading.Lock()
        self.last_msg = None
        self.last_stamp_sec = None
        self.q_gyro = Quat(1.0, 0.0, 0.0, 0.0)

        self.timer = self.create_timer(1.0 / max(1.0, self.print_hz), self.on_timer)

        print("time(s)  yaw_euler(deg)  yaw_quat(deg)  yaw_mag(deg)  yaw_gyro(deg)")

    def cb(self, msg: TenaxisImu):
        with self.lock:
            self.last_msg = msg

    def on_timer(self):
        with self.lock:
            msg = self.last_msg
        if msg is None:
            return

        # 时间戳（用于打印/积分）
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 1) 设备欧拉 yaw
        yaw_euler = float(msg.euler_rpy.z)  # rad

        # 2) 设备四元数 yaw
        q_dev = Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        yaw_quat = yaw_from_quat(q_dev)

        # 3) mag tilt-comp yaw（用设备 roll/pitch 做补偿）
        roll = float(msg.euler_rpy.x)
        pitch = float(msg.euler_rpy.y)
        yaw_mag = self.mag_tilt_comp_yaw(msg, roll, pitch)
        if self.mag_yaw_sign_flip:
            yaw_mag = -yaw_mag
        yaw_mag = wrap_pi(yaw_mag)

        # 4) gyro 积分 yaw
        yaw_gyro = self.update_gyro_yaw(msg, t)

        # 转 deg + 保留两位小数
        print(f"{t:8.2f}  "
              f"{rad2deg(wrap_pi(yaw_euler)):8.2f}        "
              f"{rad2deg(wrap_pi(yaw_quat)):8.2f}       "
              f"{rad2deg(yaw_mag):8.2f}      "
              f"{rad2deg(wrap_pi(yaw_gyro)):8.2f}")

    def mag_tilt_comp_yaw(self, msg: TenaxisImu, roll: float, pitch: float) -> float:
        mx = float(msg.magnetic_field.x)
        my = float(msg.magnetic_field.y)
        mz = float(msg.magnetic_field.z)

        cr = math.cos(roll);  sr = math.sin(roll)
        cp = math.cos(pitch); sp = math.sin(pitch)

        # 倾斜补偿（把磁场投影到水平面）
        xh = mx*cp + mz*sp
        yh = mx*sr*sp + my*cr - mz*sr*cp

        # 这里 atan2(-yh, xh) 是常见 ENU 形式
        return math.atan2(-yh, xh)

    def update_gyro_yaw(self, msg: TenaxisImu, t: float) -> float:
        if self.last_stamp_sec is None:
            self.last_stamp_sec = t
            self.q_gyro = quat_normalize(Quat(msg.orientation.w, msg.orientation.x,
                                              msg.orientation.y, msg.orientation.z))
            return yaw_from_quat(self.q_gyro)

        dt = t - self.last_stamp_sec
        self.last_stamp_sec = t
        if dt <= 0.0:
            return yaw_from_quat(self.q_gyro)

        if dt > self.max_dt:
            if self.gyro_reset_on_jump:
                self.q_gyro = quat_normalize(Quat(msg.orientation.w, msg.orientation.x,
                                                  msg.orientation.y, msg.orientation.z))
            return yaw_from_quat(self.q_gyro)

        wx = float(msg.angular_velocity.x)
        wy = float(msg.angular_velocity.y)
        wz = float(msg.angular_velocity.z)

        dq = small_angle_quat(wx, wy, wz, dt)
        self.q_gyro = quat_normalize(quat_mul(self.q_gyro, dq))
        return yaw_from_quat(self.q_gyro)


def main():
    rclpy.init()
    node = PrintYaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
