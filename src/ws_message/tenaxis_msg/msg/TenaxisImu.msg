# 十轴 IMU 自定义消息
#
# 所有物理量均为国际单位制（SI）
#
# 单位说明：
#   线加速度 linear_acceleration : m/s^2
#   角速度   angular_velocity    : rad/s
#   磁场     magnetic_field      : Tesla
#   欧拉角   euler_rpy            : rad
#   四元数   orientation          : 无量纲（单位四元数）
#   气压     pressure_pa          : Pa
#   高度     altitude_m           : m

# ROS 时间戳与坐标系
std_msgs/Header header

# 线加速度（三轴，加速度计）
geometry_msgs/Vector3 linear_acceleration

# 角速度（三轴，陀螺仪）
geometry_msgs/Vector3 angular_velocity

# 磁场强度（三轴，磁力计）
geometry_msgs/Vector3 magnetic_field

# 欧拉角（滚转 Roll，俯仰 Pitch，偏航 Yaw）
# 注意：该字段与四元数表达的姿态不同，语义上直接对应设备输出的角度数据。
# 如无直接输出的欧拉角，则需要自行推导计算或不填
geometry_msgs/Vector3 euler_rpy

# 姿态四元数（通常由 IMU 内部融合算法给出/自行推导的四元数）
geometry_msgs/Quaternion orientation

# 气压计数据（帕）
float32 pressure_pa

# 由气压计计算得到的高度（米）
float32 altitude_m
