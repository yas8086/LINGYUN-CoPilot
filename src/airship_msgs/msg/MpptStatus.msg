# MPPT光伏控制器状态 (MPPT-CAN通信协议 V1.2)
# 由 airship_mppt 节点解析 CAN 帧后发布

std_msgs/Header header

# ===== 连接状态 =====
bool online                 # CAN 链路是否在线
uint8 device_addr           # 设备地址 (0x00-0xF7)
uint16 can_id               # 本帧源 CAN ID(调试用)

# ===== 电压/电流 =====
float32 pv_voltage          # 光伏电压 [V]
float32 pv_current          # 光伏电流 [A]
float32 pv_power            # 光伏功率 [W] (计算值)
float32 battery_voltage     # 电池电压 [V]
float32 charge_current      # 充电电流 [A]
float32 rated_voltage       # 电池额定电压 [V]
float32 rated_current       # 充电额定电流 [A]

# ===== 发电量 =====
float32 energy_today        # 日发电量 [kWh]
float32 energy_month        # 月发电量 [kWh]
float32 energy_total        # 总发电量 [kWh]

# ===== 温度与状态 =====
float32 air_temp            # 机内空气温度 [℃]
float32 module_temp         # 模块温度 [℃]
uint16 charge_state         # 充电状态码 (见协议充电状态表)
uint16 fault_state          # 故障状态位 (见协议故障状态表)
uint16 control_mode         # 设备控制模式 (见协议设备控制模式表)
bool charging_enabled       # 充电开关是否开启