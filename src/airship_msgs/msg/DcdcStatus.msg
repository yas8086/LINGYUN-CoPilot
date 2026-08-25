# DCDC电源模块状态 (昊瑞昌 HRC-GFD360 / 迪龙 DE6KS16 协议)
# 由 airship_dcdc 节点解析 CAN 帧后发布

std_msgs/Header header

# ===== 连接状态 =====
bool online                 # CAN 链路是否在线

# ===== 输入/输出 =====
float32 input_voltage       # 输入电压 [V] (母线电压)
float32 output_voltage      # 输出电压 [V]
float32 output_current      # 输出电流 [A]
float32 output_power        # 输出功率 [W] (计算值)
float32 ambient_temp        # 环境温度 [℃]
float32 heatsink_temp       # 散热器温度 [℃]

# ===== 使能/状态 =====
bool output_enabled         # 输出是否开启
uint8 fault_word            # 故障状态字节 (Bit映射见协议故障表)

# ===== 设定值(下行, 最近一次下发) =====
float32 set_voltage         # 输出电压设定值 [V]
float32 set_current         # 输出电流限流值 [A]