# 灵云01号伴飞电脑 — 飞控聚合状态
# 由 airship_fc::fc_monitor 订阅 MAVROS topic 聚合生成, 统一发布到 /fc/status
std_msgs/Header header

# MAVROS 连接状态(心跳看门狗)
bool online

# 姿态角 (欧拉角, 度)
float32 roll_deg
float32 pitch_deg
float32 yaw_deg

# 角速度 (rad/s)
float32 roll_rate
float32 pitch_rate
float32 yaw_rate

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
uint8 landed_state  # 着陆状态 (0=undefined 1=on_ground 2=in_air ...)

# 空速/地速 (m/s) 与爬升率/油门
# 注: 安装版 mavros_msgs 无 Airspeed.msg, airspeed 与 true_airspeed 同源 (MAVROS VfrHud.airspeed=真空速)
float32 airspeed       # 空速 [m/s] (MAVROS VfrHud.airspeed)
float32 true_airspeed  # 真空速 TAS [m/s] (MAVROS VfrHud.airspeed; 升级 mavros_msgs 后可分离 IAS/TAS)
float32 groundspeed    # 地速 [m/s]
float32 climb_rate     # 垂直爬升率 [m/s]
float32 throttle       # 油门 [%] (MAVROS VfrHud.throttle, 0~100)

# 航向
float32 heading_deg     # 航向角 [deg] (MAVROS compass_hdg, 0~360)

# 电机输出 (归一化 0~1, 8 路, 与 M0..M7 对应)
float32[8] motor_outputs

# 电池
float32 battery_voltage   # [V]
float32 battery_current   # [A]
float32 battery_remaining # 剩余电量 0~1