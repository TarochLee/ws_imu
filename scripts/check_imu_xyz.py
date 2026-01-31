#!/usr/bin/env python3

# 这个代码可以在 RViz 中显示 IMU 的四元数、欧拉角、陀螺积分和磁力计姿态对比。
import math
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from tenaxis_msg.msg import TenaxisImu
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ----------------- 四元数/旋转工具 -----------------
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


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    # ZYX (yaw-pitch-roll)
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)

    w = cy*cp*cr + sy*sp*sr
    x = cy*cp*sr - sy*sp*cr
    y = sy*cp*sr + cy*sp*cr
    z = sy*cp*cr - cy*sp*sr
    return quat_normalize(Quat(w, x, y, z))


def quat_rotate_vec(q: Quat, v):
    # v' = q * [0,v] * q^{-1}
    qn = quat_normalize(q)
    vq = Quat(0.0, v[0], v[1], v[2])
    qi = Quat(qn.w, -qn.x, -qn.y, -qn.z)
    r = quat_mul(quat_mul(qn, vq), qi)
    return (r.x, r.y, r.z)


def small_angle_quat(wx, wy, wz, dt) -> Quat:
    # dq = [cos(|w|dt/2), sin(|w|dt/2)*axis]
    wnorm = math.sqrt(wx*wx + wy*wy + wz*wz)
    if wnorm < 1e-12:
        return Quat(1.0, 0.0, 0.0, 0.0)
    th = wnorm * dt
    s = math.sin(th * 0.5) / wnorm
    return quat_normalize(Quat(math.cos(th*0.5), wx*s, wy*s, wz*s))


# ----------------- 主节点 -----------------
class ImuDebugMarkers(Node):
    def __init__(self):
        super().__init__('imu_debug_markers_array')

        # 参数
        self.declare_parameter('imu_topic', '/imu/jy901b')
        self.declare_parameter('frame_id', 'imu_link')     # RViz 显示坐标系
        self.declare_parameter('publish_hz', 10.0)         # MarkerArray 发布频率
        self.declare_parameter('axis_len', 0.6)            # 坐标轴长度
        self.declare_parameter('line_width', 0.04)         # 线宽
        self.declare_parameter('offset', 1.4)              # 四组之间间距（沿 X）
        self.declare_parameter('use_msg_stamp', False)     # True: marker 用 msg stamp; False: 用 now()
        self.declare_parameter('max_dt', 0.2)              # gyro 积分最大步长
        self.declare_parameter('gyro_reset_on_jump', True) # dt 超过 max_dt 是否重置
        self.declare_parameter('mag_yaw_only', False)      # True: mag 只显示 yaw(roll/pitch=0)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.axis_len = float(self.get_parameter('axis_len').value)
        self.line_width = float(self.get_parameter('line_width').value)
        self.offset = float(self.get_parameter('offset').value)
        self.use_msg_stamp = bool(self.get_parameter('use_msg_stamp').value)
        self.max_dt = float(self.get_parameter('max_dt').value)
        self.gyro_reset_on_jump = bool(self.get_parameter('gyro_reset_on_jump').value)
        self.mag_yaw_only = bool(self.get_parameter('mag_yaw_only').value)

        # 订阅 QoS：对齐 SensorDataQoS（BEST_EFFORT）
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(TenaxisImu, self.imu_topic, self.cb, qos)

        # MarkerArray 发布（一次 publish，避免“点一点出”）
        self.pub = self.create_publisher(MarkerArray, '/imu/jy901b/debug_markers', 10)

        self.lock = threading.Lock()
        self.last_msg = None
        self.last_stamp_sec = None

        # gyro 积分姿态
        self.q_gyro = Quat(1.0, 0.0, 0.0, 0.0)

        # timer
        self.timer = self.create_timer(1.0 / max(1.0, self.publish_hz), self.on_timer)

        self.get_logger().info("IMU debug markers started. Topic: /imu/jy901b/debug_markers")

    def cb(self, msg: TenaxisImu):
        with self.lock:
            self.last_msg = msg

    def on_timer(self):
        with self.lock:
            msg = self.last_msg

        if msg is None:
            return

        # 选择 stamp
        if self.use_msg_stamp:
            stamp = msg.header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        # msg 时间（用于积分）
        msg_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 1) 设备四元数
        q_dev = quat_normalize(Quat(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ))

        # 2) 设备欧拉
        roll = float(msg.euler_rpy.x)
        pitch = float(msg.euler_rpy.y)
        yaw = float(msg.euler_rpy.z)
        q_euler = quat_from_rpy(roll, pitch, yaw)

        # 3) gyro 积分
        q_gyro = self.update_gyro_integration(msg, msg_stamp_sec)

        # 4) 磁力计航向（倾斜补偿）
        q_mag = self.compute_mag_pose(msg, roll, pitch)

        # 组装 MarkerArray（四组，每组 3 条轴线 + 文字）
        arr = MarkerArray()
        markers = []

        # 四组 origin 沿 X 排开
        origins = [
            (0.0, 0.0, 0.0),
            (self.offset, 0.0, 0.0),
            (2.0*self.offset, 0.0, 0.0),
            (3.0*self.offset, 0.0, 0.0),
        ]
        titles = ["Quat(device)", "Euler(device)", "Gyro(integrate)", "Mag(yaw tilt-comp)"]
        qs = [q_dev, q_euler, q_gyro, q_mag]
        group_ns = ["quat", "euler", "gyro", "mag"]

        # 生成 marker
        base_id = 1000
        for gi in range(4):
            markers += self.make_axes_group(
                ns=f"imu_dbg_{group_ns[gi]}",
                base_id=base_id + gi*10,
                q=qs[gi],
                origin=origins[gi],
                stamp=stamp
            )
            markers.append(self.make_text(
                ns="imu_dbg_text",
                mid=2000 + gi,
                text=titles[gi],
                pos=(origins[gi][0], origins[gi][1], origins[gi][2] + self.axis_len + 0.15),
                stamp=stamp
            ))

        # 统一 header
        for m in markers:
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
        arr.markers = markers

        self.pub.publish(arr)

    def update_gyro_integration(self, msg: TenaxisImu, stamp_sec: float) -> Quat:
        # 第一次：用设备四元数做初值
        if self.last_stamp_sec is None:
            self.last_stamp_sec = stamp_sec
            self.q_gyro = quat_normalize(Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z))
            return self.q_gyro

        dt = stamp_sec - self.last_stamp_sec
        self.last_stamp_sec = stamp_sec

        if dt <= 0.0:
            return self.q_gyro

        if dt > self.max_dt:
            if self.gyro_reset_on_jump:
                self.q_gyro = quat_normalize(Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z))
            return self.q_gyro

        wx = float(msg.angular_velocity.x)
        wy = float(msg.angular_velocity.y)
        wz = float(msg.angular_velocity.z)

        dq = small_angle_quat(wx, wy, wz, dt)
        self.q_gyro = quat_normalize(quat_mul(self.q_gyro, dq))
        return self.q_gyro

    def compute_mag_pose(self, msg: TenaxisImu, roll: float, pitch: float) -> Quat:
        # 磁场：Tesla（比例不重要，只要方向）
        mx = float(msg.magnetic_field.x)
        my = float(msg.magnetic_field.y)
        mz = float(msg.magnetic_field.z)

        cr = math.cos(roll);  sr = math.sin(roll)
        cp = math.cos(pitch); sp = math.sin(pitch)

        # 倾斜补偿到水平面
        xh = mx*cp + mz*sp
        yh = mx*sr*sp + my*cr - mz*sr*cp

        yaw_mag = math.atan2(-yh, xh)

        if self.mag_yaw_only:
            return quat_from_rpy(0.0, 0.0, yaw_mag)
        return quat_from_rpy(roll, pitch, yaw_mag)

    def make_axes_group(self, ns: str, base_id: int, q: Quat, origin, stamp):
        # 每组 3 个 Marker（X红、Y绿、Z蓝）
        ox, oy, oz = origin
        out = []

        # axis unit vectors in local frame
        axes = [
            ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), base_id + 0),  # X red
            ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0), base_id + 1),  # Y green
            ((0.0, 0.0, 1.0), (0.0, 0.6, 1.0), base_id + 2),  # Z blue-ish
        ]

        for axis_vec, color, mid in axes:
            m = Marker()
            m.ns = ns
            m.id = mid
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = self.line_width
            m.color.a = 1.0
            m.color.r = float(color[0])
            m.color.g = float(color[1])
            m.color.b = float(color[2])

            vx, vy, vz = quat_rotate_vec(q, axis_vec)

            p0 = Point(); p1 = Point()
            p0.x, p0.y, p0.z = ox, oy, oz
            p1.x = ox + vx * self.axis_len
            p1.y = oy + vy * self.axis_len
            p1.z = oz + vz * self.axis_len

            m.points = [p0, p1]
            out.append(m)

        return out

    def make_text(self, ns: str, mid: int, text: str, pos, stamp):
        m = Marker()
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        m.scale.z = 0.2
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.text = text
        return m


def main():
    rclpy.init()
    node = ImuDebugMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
