# 设备告警消息 (由 airship_monitor 聚合各设备状态后生成)
# 用于统一上报告警给数传链路 / 上位机

std_msgs/Header header

# ===== 设备标识 =====
uint8 DEVICE_BMS = 1
uint8 DEVICE_MPPT = 2
uint8 DEVICE_DCDC = 3
uint8 device_type            # 设备类型 (DEVICE_*)

# ===== 告警信息 =====
uint8 SEVERITY_INFO = 0
uint8 SEVERITY_WARNING = 1
uint8 SEVERITY_CRITICAL = 2
uint8 severity               # 严重级别 (SEVERITY_*)
uint16 code                  # 设备相关故障位/错误码
string message               # 人类可读描述
bool active                  # 当前是否仍处于该告警状态