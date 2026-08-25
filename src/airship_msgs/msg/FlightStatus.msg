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

# EKF 估计器状态 (来自 /mavros/estimator_status/status, MAVLink ESTIMATOR_STATUS)
bool ekf_const_pos_mode    # 位置锁定模式 (GPS 不可用/退化), true 表示定位精度骤降
bool ekf_gps_glitch        # GPS 信号毛刺
bool ekf_accel_error       # 加速度计估计错误
bool ekf_pos_vert_agl      # 垂直相对地面位置估计有效

# GPS 原始数据 (来自 /mavros/global_position/raw/gps, MAVLink GPS_RAW_INT)
uint8  gps_fix_type        # [GPS_FIX_TYPE] 0=无GPS 1=NO_FIX 2=2D 3=3D 4=DGPS 5/6=RTK
uint8  gps_satellites      # 可见卫星数, 255=未知
uint16 gps_eph             # 水平精度 HDOP (unitless), UINT16_MAX=未知
uint16 gps_epv             # 垂直精度 VDOP (unitless), UINT16_MAX=未知

# 电机 ESC 遥测 (来自 /mavros/esc_status + /mavros/esc_telemetry, DroneCAN/CAN 电调)
# 前提: 电调走 DroneCAN/CAN 接入飞控才回传; PWM 供电无法获取, 此时 esc_count=0
uint8         esc_count                  # 实际有遥测数据的电调数量 (0 表示电调未提供遥测)
float32[10]   esc_rpm                    # 各电调转速 [RPM]
float32[10]   esc_voltage                # 各电调电压 [V]
float32[10]   esc_current                # 各电调电流 [A]
float32[10]   esc_temperature            # 各电调温度 [degC]

# 电池
float32 battery_voltage   # [V]
float32 battery_current   # [A]
float32 battery_remaining # 剩余电量百分比 0~100