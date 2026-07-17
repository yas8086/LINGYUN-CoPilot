# 安全监控状态
# 由 airship_safety 节点发布，监控飞艇专属安全约束

std_msgs/Header header

# ===== 高度安全 =====
bool altitude_violation     # 高度超限(超出AS_ALT_MIN~AS_ALT_MAX)
float32 altitude            # 当前相对高度 [m]
float32 alt_max             # 高度上限 [m]
float32 alt_min             # 高度下限 [m]
float32 alt_margin          # 距离最近限位的余量 [m]

# ===== 偏航安全 =====
bool yaw_rate_exceeded      # 偏航角速度超限
float32 yaw_rate            # 当前偏航角速度 [rad/s]
float32 yaw_rate_limit      # 偏航角速度限制 [rad/s]

# ===== 俯仰安全 =====
bool pitch_exceeded         # 俯仰角超限(±15度)
float32 pitch               # 当前俯仰角 [rad]
float32 pitch_limit         # 俯仰角限制 [rad]

# ===== 通信安全 =====
bool link_lost              # 通信链路丢失
float32 link_age            # 最后一次收到数据的时间差 [s]

# ===== 电池安全 =====
bool battery_low            # 电池低电量
float32 battery_voltage     # 电池电压 [V]

# ===== 综合状态 =====
bool safe_to_control        # 是否安全可进行Offboard控制
string warning_message      # 警告信息(为空表示无警告)
uint8 safety_level          # 安全等级 0=正常 1=警告 2=危险 3=紧急
