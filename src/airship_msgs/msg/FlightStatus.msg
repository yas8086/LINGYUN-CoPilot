# 灵云01号伴飞电脑 — 飞控聚合状态
# 由 airship_fc::fc_monitor 订阅 MAVROS topic 聚合生成, 统一发布到 /fc/status
std_msgs/Header header

# MAVROS 连接状态(心跳看门狗)
bool online

# 姿态角 (欧拉角, 度)
float32 roll_deg
float32 pitch_deg
float32 yaw_deg

# 位置/GPS
float64 lat        # 纬度 [deg]
float64 lon        # 经度 [deg]
float32 alt_amsl   # 海拔高度 [m]
float32 alt_rel    # 相对起飞点高度 [m]

# 速度 (ENU, m/s)
float32 vx
float32 vy
float32 vz

# 飞行状态
string flight_mode  # 当前飞行模式
bool armed          # 是否解锁

# 电池
float32 battery_voltage   # [V]
float32 battery_current   # [A]
float32 battery_remaining # 剩余电量 0~1