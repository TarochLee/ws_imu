#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster

from tenaxis_msg.msg import TenaxisImu


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def vec_norm(v) -> float:
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])


def quat_norm(q) -> float:
    return math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])


def quat_normalize(q):
    n = quat_norm(q)
    if n <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n)


def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def rotate_vec_by_quat(v, q):
    # v_world = q*(0,v)*q_conj, q is body->world
    vx, vy, vz = v
    vq = (0.0, vx, vy, vz)
    return quat_mul(quat_mul(q, vq), quat_conj(q))[1:4]


def rpy_from_quat(q):
    w, x, y, z = q
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi/2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return quat_normalize((w, x, y, z))


def yaw_from_mag_tilt_comp(m_body, roll, pitch):
    # tilt compensation: returns yaw in rad
    mx, my, mz = m_body
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)

    Xh = mx * cp + mz * sp
    Yh = mx * sr * sp + my * cr - mz * sr * cp

    # 轴系不同可能需要改符号：若 yaw 反向，把 -Yh 改成 Yh
    yaw = math.atan2(-Yh, Xh)
    return wrap_pi(yaw)


class ImuTrajObserverOrigin(Node):
    """
    目标：
    - 不依赖任何外部 link/静态 TF
    - 节点自己发布 TF: map -> imu_link
    - 以首次收到数据为原点 (x=y=z=0)
    - z 使用相对高度: altitude_m - altitude0
    - yaw 用磁力计观测更新；roll/pitch 用 IMU orientation
    - XY 使用加速度积分 + ZUPT 抑制漂移
    """

    def __init__(self):
        super().__init__("imu_traj_observer_origin")

        self.declare_parameter("imu_topic", "/imu/jy901b")
        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("child_frame", "imu_link")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("path_max_len", 2000)
        self.declare_parameter("max_dt", 0.2)
        self.declare_parameter("g", 9.8)

        # ZUPT
        self.declare_parameter("enable_zupt", True)
        self.declare_parameter("zupt_gyro_th", 0.03)  # rad/s
        self.declare_parameter("zupt_acc_th", 0.25)   # m/s^2, | |a_world|-g |
        self.declare_parameter("zupt_hold_s", 0.15)

        # MAG yaw update
        self.declare_parameter("enable_mag_yaw", True)
        self.declare_parameter("yaw_gain", 0.05)           # 0..1
        self.declare_parameter("mag_norm_gate_ratio", 0.35)
        self.declare_parameter("mag_avg_alpha", 0.01)

        # Origin handling
        self.declare_parameter("reset_on_start", True)      # 第一次消息重置原点
        self.declare_parameter("use_baro_z", True)          # z 由相对气压高度提供（不积分）

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.path_max_len = int(self.get_parameter("path_max_len").value)
        self.max_dt = float(self.get_parameter("max_dt").value)
        self.g = float(self.get_parameter("g").value)

        self.enable_zupt = bool(self.get_parameter("enable_zupt").value)
        self.zupt_gyro_th = float(self.get_parameter("zupt_gyro_th").value)
        self.zupt_acc_th = float(self.get_parameter("zupt_acc_th").value)
        self.zupt_hold_s = float(self.get_parameter("zupt_hold_s").value)

        self.enable_mag_yaw = bool(self.get_parameter("enable_mag_yaw").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.mag_norm_gate_ratio = float(self.get_parameter("mag_norm_gate_ratio").value)
        self.mag_avg_alpha = float(self.get_parameter("mag_avg_alpha").value)

        self.reset_on_start = bool(self.get_parameter("reset_on_start").value)
        self.use_baro_z = bool(self.get_parameter("use_baro_z").value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(TenaxisImu, self.imu_topic, self.cb_imu, qos)

        self.pub_path = self.create_publisher(Path, "/imu/jy901b/path", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/imu/jy901b/pose", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.lock = threading.Lock()
        self.last_msg = None
        self.last_t = None

        # state: relative origin
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        self.vx = 0.0
        self.vy = 0.0

        self.alt0 = None  # altitude origin
        self.origin_inited = False

        # attitude
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_est = 0.0
        self.yaw_inited = False

        # mag gate
        self.mag_norm_avg = None

        # ZUPT
        self.still_time = 0.0
        self.bax_w = 0.0
        self.bay_w = 0.0

        self.path = Path()
        self.path.header.frame_id = self.fixed_frame

        period = 1.0 / max(1.0, self.publish_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"start: sub={self.imu_topic} tf={self.fixed_frame}->{self.child_frame} "
            f"origin=first_msg relative_z={self.use_baro_z} mag_yaw={self.enable_mag_yaw} zupt={self.enable_zupt}"
        )

        # 立即发一个 TF（初值），避免 RViz 切 fixed frame 时短暂无 TF
        q0 = quat_from_rpy(0.0, 0.0, 0.0)
        self.publish_tf(q0)

    def cb_imu(self, msg: TenaxisImu):
        with self.lock:
            self.last_msg = msg

    def on_timer(self):
        with self.lock:
            msg = self.last_msg

        if msg is None:
            q_out = quat_from_rpy(self.roll, self.pitch, self.yaw_est)
            self.publish_tf(q_out)
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # -------- origin init (first message) --------
        if (not self.origin_inited) and self.reset_on_start:
            self.px = self.py = self.pz = 0.0
            self.vx = self.vy = 0.0
            self.bax_w = self.bay_w = 0.0
            self.still_time = 0.0
            self.alt0 = float(msg.altitude_m) if self.use_baro_z else 0.0
            self.origin_inited = True
            self.path.poses.clear()
            self.get_logger().info(f"origin set: alt0={self.alt0:.3f} m, pos reset to (0,0,0)")

        # -------- attitude from IMU quaternion --------
        q_imu = quat_normalize((
            float(msg.orientation.w),
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
        ))
        self.roll, self.pitch, yaw_imu = rpy_from_quat(q_imu)

        # init yaw
        if not self.yaw_inited:
            self.yaw_est = yaw_imu
            self.yaw_inited = True

        # -------- yaw observation from magnetometer --------
        if self.enable_mag_yaw:
            m = (
                float(msg.magnetic_field.x),
                float(msg.magnetic_field.y),
                float(msg.magnetic_field.z),
            )
            m_norm = vec_norm(m)
            if self.mag_norm_avg is None:
                self.mag_norm_avg = m_norm
            else:
                self.mag_norm_avg = (1.0 - self.mag_avg_alpha) * self.mag_norm_avg + self.mag_avg_alpha * m_norm

            rel = 0.0
            if self.mag_norm_avg and self.mag_norm_avg > 1e-12:
                rel = abs(m_norm - self.mag_norm_avg) / self.mag_norm_avg

            if m_norm > 1e-12 and rel < self.mag_norm_gate_ratio:
                yaw_meas = yaw_from_mag_tilt_comp(m, self.roll, self.pitch)
                e = wrap_pi(yaw_meas - self.yaw_est)
                self.yaw_est = wrap_pi(self.yaw_est + self.yaw_gain * e)

        # fused output orientation
        q_out = quat_from_rpy(self.roll, self.pitch, self.yaw_est)

        # -------- relative baro Z --------
        if self.use_baro_z and self.alt0 is not None:
            self.pz = float(msg.altitude_m) - self.alt0
        else:
            self.pz = 0.0

        # -------- time init --------
        if self.last_t is None:
            self.last_t = t
            self.publish_all(msg, q_out)
            return

        dt = t - self.last_t
        self.last_t = t
        if not (0.0 < dt < self.max_dt):
            self.publish_all(msg, q_out)
            return

        # -------- XY prediction: accel integrate in world --------
        a_b = (
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )

        # rotate to world using fused q_out
        a_w = rotate_vec_by_quat(a_b, q_out)

        # remove gravity (你的数据静止时 z≈9.8，所以必须减 g)
        a_lin_w = (a_w[0], a_w[1], a_w[2] - self.g)

        # -------- ZUPT observation --------
        if self.enable_zupt:
            w_b = (
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            )
            gyro_ok = vec_norm(w_b) < self.zupt_gyro_th
            acc_ok = abs(vec_norm(a_w) - self.g) < self.zupt_acc_th
            still = gyro_ok and acc_ok

            if still:
                self.still_time += dt
            else:
                self.still_time = 0.0

            if self.still_time >= self.zupt_hold_s:
                # 静止时：让 a_lin_w(水平)逼近 0，用它估偏置，并把速度观测为 0
                alpha = 0.02
                self.bax_w = (1.0 - alpha) * self.bax_w + alpha * a_lin_w[0]
                self.bay_w = (1.0 - alpha) * self.bay_w + alpha * a_lin_w[1]
                self.vx = 0.0
                self.vy = 0.0

        # subtract bias on horizontal accel
        ax = a_lin_w[0] - self.bax_w
        ay = a_lin_w[1] - self.bay_w

        # integrate (semi-implicit Euler)
        self.vx += ax * dt
        self.vy += ay * dt
        self.px += self.vx * dt
        self.py += self.vy * dt

        self.publish_all(msg, q_out)

    def publish_all(self, msg: TenaxisImu, q_out):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.fixed_frame
        pose.pose.position.x = self.px
        pose.pose.position.y = self.py
        pose.pose.position.z = self.pz
        pose.pose.orientation.w = q_out[0]
        pose.pose.orientation.x = q_out[1]
        pose.pose.orientation.y = q_out[2]
        pose.pose.orientation.z = q_out[3]
        self.pub_pose.publish(pose)

        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = self.fixed_frame
        self.path.poses.append(pose)
        if len(self.path.poses) > self.path_max_len:
            self.path.poses = self.path.poses[-self.path_max_len:]
        self.pub_path.publish(self.path)

        # TF 用 now()，避免 RViz 切 fixed frame 时因为 TF 时间戳问题消失
        self.publish_tf(q_out)

    def publish_tf(self, q_out):
        tfm = TransformStamped()
        tfm.header.stamp = self.get_clock().now().to_msg()
        tfm.header.frame_id = self.fixed_frame
        tfm.child_frame_id = self.child_frame
        tfm.transform.translation.x = self.px
        tfm.transform.translation.y = self.py
        tfm.transform.translation.z = self.pz
        tfm.transform.rotation.w = q_out[0]
        tfm.transform.rotation.x = q_out[1]
        tfm.transform.rotation.y = q_out[2]
        tfm.transform.rotation.z = q_out[3]
        self.tf_broadcaster.sendTransform(tfm)


def main():
    rclpy.init()
    node = ImuTrajObserverOrigin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
