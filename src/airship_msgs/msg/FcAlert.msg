# 飞控参数异常告警消息 (由 airship_fc 节点在异常检测触发时发布)
# 多级异常等级: INFO / WARNING / CRITICAL / EMERGENCY
# 用于向数传链路 / 上位机上报飞控参数越界告警

std_msgs/Header header

# ===== 异常等级 =====
uint8 LEVEL_INFO = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_CRITICAL = 2
uint8 LEVEL_EMERGENCY = 3

uint8 level                # 异常等级 (LEVEL_*)

# ===== 参数标识 =====
string param_name          # 异常参数名 (如 roll / pitch / altitude_agl / battery_voltage)
float32 current_value      # 当前值
float32 threshold          # 触发阈值

# ===== 详情 =====
string description         # 人类可读描述
string suggestion          # 建议操作 (可空)
bool active                # 当前是否仍处于该异常状态 (false 表示恢复解除)
