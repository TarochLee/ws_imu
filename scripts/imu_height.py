#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tenaxis_msg.msg import TenaxisImu


def median(xs):
    ys = sorted(xs)
    return ys[len(ys)//2] if ys else 0.0


def quat_normalize(w, x, y, z):
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-12:
        return 1.0, 0.0, 0.0, 0.0, False
    return w/n, x/n, y/n, z/n, True


def quat_to_dcm_bn(q0, q1, q2, q3):
    # C_b^n：机体系(body) -> 导航系(nav, ENU) 的方向余弦矩阵
    c11 = 1.0 - 2.0*(q2*q2 + q3*q3)
    c12 = 2.0*(q1*q2 - q0*q3)
    c13 = 2.0*(q1*q3 + q0*q2)

    c21 = 2.0*(q1*q2 + q0*q3)
    c22 = 1.0 - 2.0*(q1*q1 + q3*q3)
    c23 = 2.0*(q2*q3 - q0*q1)

    c31 = 2.0*(q1*q3 - q0*q2)
    c32 = 2.0*(q2*q3 + q0*q1)
    c33 = 1.0 - 2.0*(q1*q1 + q2*q2)

    return (c11, c12, c13,
            c21, c22, c23,
            c31, c32, c33)


def baro_height_from_pressure(P, P0, T0=288.15, L=0.0065, R=287.05, g=9.80665):
    """
    标准大气近似（对“相对高度”够用）：
    h = (T0/L) * [ (P/P0)^(-R*L/g) - 1 ]
    """
    if P <= 0.0 or P0 <= 0.0:
        return 0.0
    exp = -(R * L) / g
    return (T0 / L) * ((P / P0) ** exp - 1.0)


class KF_ZVB:
    """
    3 状态卡尔曼滤波（更适合你这种“静止时气压量化跳变”的情况）：
      x = [ 高度z, 垂直速度vz, 垂直加速度零偏b ]^T
    预测（输入是垂直净加速度 a_net）：
      a_used = a_net - b
      z'  = z  + vz*dt + 0.5*a_used*dt^2
      vz' = vz + a_used*dt
      b'  = b  （随机游走）

    观测：
      z_baro = z + 噪声
    """
    def __init__(self, sigma_a=1.0, sigma_b=0.02, sigma_z=0.10):
        self.sigma_a = float(sigma_a)   # 垂直净加速度噪声 (m/s^2)
        self.sigma_b = float(sigma_b)   # bias 随机游走强度 (m/s^2/sqrt(s))
        self.sigma_z = float(sigma_z)   # 气压高度观测噪声 (m)

        self.reset()

    def reset(self):
        self.z = 0.0
        self.v = 0.0
        self.b = 0.0

        # P 3x3
        self.P00 = 1.0; self.P01 = 0.0; self.P02 = 0.0
        self.P10 = 0.0; self.P11 = 1.0; self.P12 = 0.0
        self.P20 = 0.0; self.P21 = 0.0; self.P22 = 0.5

    def predict(self, dt, a_net):
        if dt <= 0.0:
            return

        # ---- 状态预测 ----
        a_used = a_net - self.b
        self.z = self.z + self.v*dt + 0.5*a_used*dt*dt
        self.v = self.v + a_used*dt
        # b 保持不变（随机游走在 Q 里体现）

        # ---- 协方差预测 ----
        # F = [[1, dt, -0.5*dt^2],
        #      [0, 1,  -dt     ],
        #      [0, 0,   1      ]]
        dt2 = dt*dt
        f02 = -0.5*dt2
        f12 = -dt

        # P = F P F^T + Q
        # 先算 FP
        FP00 = self.P00 + dt*self.P10 + f02*self.P20
        FP01 = self.P01 + dt*self.P11 + f02*self.P21
        FP02 = self.P02 + dt*self.P12 + f02*self.P22

        FP10 = self.P10 + f12*self.P20
        FP11 = self.P11 + f12*self.P21
        FP12 = self.P12 + f12*self.P22

        FP20 = self.P20
        FP21 = self.P21
        FP22 = self.P22

        # 再算 P = FP * F^T
        P00 = FP00 + dt*FP01 + f02*FP02
        P01 = FP01 + dt*FP11 + f02*FP12
        P02 = FP02 + dt*FP12 + f02*FP22

        P10 = FP10 + f12*FP20
        P11 = FP11 + f12*FP21
        P12 = FP12 + f12*FP22

        P20 = FP20
        P21 = FP21
        P22 = FP22

        # Q：按“加速度噪声驱动 z,v” + “bias 随机游走”
        sa2 = self.sigma_a * self.sigma_a
        sb2 = self.sigma_b * self.sigma_b

        # 加速度噪声离散化（对应文章里 Q 的 2x2 部分）
        Q00 = 0.25*(dt2*dt2)*sa2
        Q01 = 0.5*(dt2*dt)*sa2
        Q11 = (dt2)*sa2

        # bias random walk：b' = b + w_b, Var(w_b)=sb2*dt
        Q22 = sb2 * dt

        self.P00 = P00 + Q00
        self.P01 = P01 + Q01
        self.P02 = P02

        self.P10 = P10 + Q01
        self.P11 = P11 + Q11
        self.P12 = P12

        self.P20 = P20
        self.P21 = P21
        self.P22 = P22 + Q22

    def update_z(self, z_meas):
        # H = [1, 0, 0]
        R = self.sigma_z * self.sigma_z
        y = z_meas - self.z
        S = self.P00 + R
        if S <= 1e-12:
            return

        K0 = self.P00 / S
        K1 = self.P10 / S
        K2 = self.P20 / S

        self.z = self.z + K0*y
        self.v = self.v + K1*y
        self.b = self.b + K2*y

        # P = (I - K H) P
        P00 = (1.0 - K0) * self.P00
        P01 = (1.0 - K0) * self.P01
        P02 = (1.0 - K0) * self.P02

        P10 = self.P10 - K1 * self.P00
        P11 = self.P11 - K1 * self.P01
        P12 = self.P12 - K1 * self.P02

        P20 = self.P20 - K2 * self.P00
        P21 = self.P21 - K2 * self.P01
        P22 = self.P22 - K2 * self.P02

        self.P00, self.P01, self.P02 = P00, P01, P02
        self.P10, self.P11, self.P12 = P10, P11, P12
        self.P20, self.P21, self.P22 = P20, P21, P22


class ImuBaroKFPrintCN(Node):
    def __init__(self):
        super().__init__('imu_baro_kf_print3_cn')

        self.declare_parameter('topic', '/imu/jy901b')
        self.declare_parameter('baseline_samples', 200)
        self.declare_parameter('print_hz', 10.0)
        self.declare_parameter('use_ros_time', True)

        self.declare_parameter('g', 9.80665)
        self.declare_parameter('remove_gravity', True)

        self.declare_parameter('T0', 288.15)
        self.declare_parameter('L', 0.0065)
        self.declare_parameter('R_air', 287.05)

        # KF 参数
        self.declare_parameter('sigma_a', 1.0)    # 垂直净加速度噪声
        self.declare_parameter('sigma_b', 0.02)   # 零偏随机游走
        self.declare_parameter('sigma_z', 0.10)   # 气压高度观测噪声（你这 1Pa≈0.09m，可从0.10m起）

        self.topic = str(self.get_parameter('topic').value)
        self.baseline_samples = int(self.get_parameter('baseline_samples').value)
        self.print_hz = float(self.get_parameter('print_hz').value)
        self.use_ros_time = bool(self.get_parameter('use_ros_time').value)

        self.g = float(self.get_parameter('g').value)
        self.remove_gravity = bool(self.get_parameter('remove_gravity').value)

        self.T0 = float(self.get_parameter('T0').value)
        self.L = float(self.get_parameter('L').value)
        self.R_air = float(self.get_parameter('R_air').value)

        sigma_a = float(self.get_parameter('sigma_a').value)
        sigma_b = float(self.get_parameter('sigma_b').value)
        sigma_z = float(self.get_parameter('sigma_z').value)
        self.kf = KF_ZVB(sigma_a=sigma_a, sigma_b=sigma_b, sigma_z=sigma_z)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=500,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(TenaxisImu, self.topic, self.cb, qos)

        self.P0 = None
        self._pbuf = []
        self.t_last = None

        self._print_period = 1.0 / max(0.5, self.print_hz)
        self._next_print = time.time()

        self.get_logger().info(
            f"订阅: {self.topic} | 基准样本={self.baseline_samples} | 打印频率={self.print_hz}Hz | "
            f"去重力={self.remove_gravity} | KF(sigma_a={sigma_a}, sigma_b={sigma_b}, sigma_z={sigma_z})"
        )

    def _stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def cb(self, msg: TenaxisImu):
        # 时间
        if self.use_ros_time:
            t = self._stamp_to_sec(msg.header.stamp)
            if t <= 1e-6:
                t = time.time()
        else:
            t = time.time()

        P = float(msg.pressure_pa)
        ax_b = float(msg.linear_acceleration.x)
        ay_b = float(msg.linear_acceleration.y)
        az_b = float(msg.linear_acceleration.z)

        # 基准气压 P0（让高度从 0m 开始）
        if self.P0 is None:
            if P > 0.0:
                self._pbuf.append(P)
            if len(self._pbuf) >= self.baseline_samples:
                self.P0 = median(self._pbuf)
                self.kf.reset()
                self.t_last = t
            self._print_baseline(P)
            return

        dt = 0.0 if self.t_last is None else (t - self.t_last)
        self.t_last = t
        if dt < 0.0:
            dt = 0.0
        if dt > 0.2:
            dt = 0.0

        # 姿态四元数 -> DCM
        qw = float(msg.orientation.w)
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw, qx, qy, qz, att_ok = quat_normalize(qw, qx, qy, qz)

        if att_ok:
            C = quat_to_dcm_bn(qw, qx, qy, qz)
            # 加速度 body -> nav(ENU)
            az_n = C[6]*ax_b + C[7]*ay_b + C[8]*az_b  # Up
        else:
            az_n = az_b  # 退化debug

        # 重力补偿：得到垂直净加速度
        az_net = az_n - self.g if self.remove_gravity else az_n

        # 气压高度（蓝线）
        z_baro = baro_height_from_pressure(P, self.P0, T0=self.T0, L=self.L, R=self.R_air, g=self.g)

        # KF 融合（红线）
        self.kf.predict(dt, az_net)
        self.kf.update_z(z_baro)

        # 中文打印（重点输出高度）
        now = time.time()
        if now >= self._next_print:
            self._next_print = now + self._print_period
            print(
                f"气压(Pa)={P:.0f}  "
                f"气压高度_蓝(cm)={z_baro*100:+.1f}  "
                f"融合高度_红(cm)={self.kf.z*100:+.1f}  "
                f"融合垂直速度(m/s)={self.kf.v:+.3f}  "
                f"估计加速度零偏(m/s^2)={self.kf.b:+.3f}  "
                f"垂直净加速度(m/s^2)={az_net:+.3f}  "
                f"姿态有效={1 if att_ok else 0}"
            )

    def _print_baseline(self, P):
        now = time.time()
        if now >= self._next_print:
            self._next_print = now + self._print_period
            print(f"[基准中] 采集气压基准P0: {len(self._pbuf)}/{self.baseline_samples}  当前气压={P:.0f}Pa")


def main():
    rclpy.init()
    node = ImuBaroKFPrintCN()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
