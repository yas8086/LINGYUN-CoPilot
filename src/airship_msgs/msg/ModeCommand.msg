# 飞行模式切换命令
# 对应 MAV_CMD_DO_SET_MODE, 模式编号见飞行模式文档

std_msgs/Header header

# ===== 模式定义 (与飞控AirshipMode一致) =====
# 0=Manual, 1=Stable, 2=Altitude, 3=Position
# 4=Offboard, 5=Takeoff, 6=Land, 7=Failsafe, 8=Task
uint8 mode

# 模式枚举常量
uint8 MODE_MANUAL = 0
uint8 MODE_STABLE = 1
uint8 MODE_ALTITUDE = 2
uint8 MODE_POSITION = 3
uint8 MODE_OFFBOARD = 4
uint8 MODE_TAKEOFF = 5
uint8 MODE_LAND = 6
uint8 MODE_FAILSAFE = 7
uint8 MODE_TASK = 8

# ===== 附加命令 =====
bool arm                    # true=解锁, false=上锁
bool set_mode               # 是否切换模式
bool takeoff                # 起飞命令
bool land                   # 降落命令
bool return_to_launch       # 返航命令

# ===== 起飞参数 =====
float32 takeoff_alt         # 起飞目标高度 [m], 默认AS_TAKEOFF_ALT=20m
