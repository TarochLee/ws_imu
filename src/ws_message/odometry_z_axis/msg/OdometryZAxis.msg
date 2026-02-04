# OdometryZAxis
# 一维 Z 轴里程计输出，包含不确定度（精度/置信度）和诊断信息
#
# 单位（SI）：
#   z_m      : 米 (m)
#   vz_mps   : 米/秒 (m/s)
#   az_mps2  : 米/秒^2 (m/s^2)
#   *_var    : 对应量的方差（单位平方）
#   *_std    : 对应量的标准差（与量本身单位相同）
#
# 约定：
#   如果某项方差/标准差不可用，填 -1.0

std_msgs/Header header

# 坐标系信息（显式携带，便于工程集成与排错）
string frame_id        # 例如 "odom_z"
string child_frame_id  # 例如 "base_link"

# Z 轴状态估计值
float64 z_m            # 高度/位移（Z），单位：m
float64 vz_mps         # 垂直速度（Z 轴速度），单位：m/s
float64 az_mps2        # 垂直加速度（Z 轴加速度），单位：m/s^2

# 不确定度（方差）
float64 z_var          # z_m 的方差，单位：m^2
float64 vz_var         # vz_mps 的方差，单位：(m/s)^2
float64 az_var         # az_mps2 的方差，单位：(m/s^2)^2

# 不确定度（标准差）
float64 z_std          # z_m 的标准差，单位：m
float64 vz_std         # vz_mps 的标准差，单位：m/s
float64 az_std         # az_mps2 的标准差，单位：m/s^2

# 滤波/融合状态
bool valid             # 当前输出是否有效（综合健康状态判定）
bool fused_baro        # 最近是否融合了气压计高度（barometer/altitude）
bool fused_imu_acc     # 最近是否使用了 IMU 加速度（用于预测或更新）

uint8 filter_type      # 滤波器类型：0=未知，1=EKF，2=UKF
uint8 status           # 状态：0=OK，1=WARNING，2=ERROR
string status_msg      # 简短诊断信息（用于日志/界面显示）

# 诊断信息（调参与监控很有用）
float64 baro_innov_m   # 气压高度创新量/残差：z_meas - z_pred（单位：m）
float64 baro_innov_var # 创新方差（innovation variance）

# 时间与计数信息
float64 dt_s           # 本次滤波使用的时间步长 dt（单位：s）
uint32 seq             # 单调递增的输出序号（可用于丢包/延迟分析）
