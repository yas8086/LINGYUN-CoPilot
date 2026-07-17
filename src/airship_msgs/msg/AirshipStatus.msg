# 灵云01号飞艇综合状态
# 聚合来自MAVLink的关键遥测数据，供伴飞电脑各节点订阅

std_msgs/Header header

# ===== 时间与连接 =====
builtin_interfaces/Time stamp
bool connected              # MAVLink连接状态
uint8 system_id             # 飞控系统ID

# ===== 解锁与模式 =====
bool armed                  # 是否解锁
uint8 nav_state             # 导航模式(0-8, 见飞行模式文档)
string mode_name            # 模式名称(可读)
bool landed                 # 着陆状态(飞艇悬停时为true, 属正常)
bool offboard_enabled       # Offboard模式是否激活

# ===== 姿态 (NED坐标系) =====
float32[4] quaternion       # 姿态四元数 [w,x,y,z]
float32 roll                # 横滚角(不可控, 仅参考) [rad]
float32 pitch               # 俯仰角(正=抬头) [rad]
float32 yaw                 # 偏航角 [rad]
float32 rollspeed           # 横滚角速度 [rad/s]
float32 pitchspeed          # 俯仰角速度 [rad/s]
float32 yawspeed            # 偏航角速度 [rad/s]

# ===== 本地位置 (NED坐标系) =====
float32 x                   # 北向位置 [m]
float32 y                   # 东向位置 [m]
float32 z                   # 下向位置 [m], 飞艇高度 = -z
float32 vx                  # 北向速度 [m/s]
float32 vy                  # 东向速度 [m/s]
float32 vz                  # 下向速度 [m/s], 上升速度 = -vz
float32 heading             # 航向角 [rad]

# ===== 全局位置 =====
float64 latitude            # 纬度 [deg]
float64 longitude           # 经度 [deg]
float32 altitude_amsl       # 海拔高度 [m]
float32 altitude_relative   # 相对起飞点高度 [m]
float32 altitude_agl        # 离地高度 [m]

# ===== 电池与系统 =====
float32 battery_voltage     # 电池电压 [V]
float32 battery_current     # 电池电流 [A]
float32 battery_remaining   # 电池剩余百分比 [0-100]

# ===== 飞艇专属约束状态 =====
float32 alt_max_limit       # 高度硬限位 AS_ALT_MAX [m]
float32 alt_min_limit       # 高度低限位 AS_ALT_MIN [m]
bool alt_limit_exceeded     # 是否超出高度限位
