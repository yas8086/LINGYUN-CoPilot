# 12S 备用电源 BMS 状态 (北辰串口协议 V1.4)
# 由 airship_backup_bms 节点经串口(默认 9600 8N1)轮询解析后发布
#
# 协议帧: 发送 0x56[addr][host][rw][cmd][len2][data][crc16]
#         响应 0x57[addr][host][rw][cmd][len2][data][crc16]  (crc 覆盖地址..数据)
# 轮询指令: 0x06 基本信息1 / 0x07 单节温度 / 0x08 单节电压

std_msgs/Header header

# ===== 连接状态 =====
bool online                 # 串口链路是否在线(最近成功解析到有效响应)

# ===== 模块总压/总线 (0x06 基本信息1) =====
float32 pack_voltage        # 电池包总压 [V] (0.01V)
float32 pack_current        # 电池包总电流 [A] (0.01A, 充电为正, 32位有符号)
float32 soc                 # 荷电状态 [0-100 %] (0.1%)
float32 soh                 # 健康状态 [0-100 %] (0.1%)
float32 max_cell_voltage    # 最高单体电压 [V] (1mV)
float32 min_cell_voltage    # 最低单体电压 [V] (1mV)
float32 cell_voltage_diff   # 单体压差 [V] (1mV)
float32 max_cell_temp       # 最高单体温度 [℃] (0.1℃)
float32 min_cell_temp       # 最低单体温度 [℃] (0.1℃)
float32 avg_cell_temp       # 平均单体温度 [℃] (0.1℃)
float32 temp_diff           # 单体温差 [℃] (0.1℃)

# ===== 单体电压/温度(可变长, 长度等于实际电芯/温度点数量, 0x08/0x07) =====
float32[] cell_voltages     # 单体电压 [V] (1mV)
float32[] cell_temps        # 单体温度 [℃] (0.1℃, 有符号)

# ===== 状态字 (0x06, 均为 32 位, 位定义见北辰协议) =====
uint32 alarm_word           # 告警标志位 (Bit0..16, 见协议)
uint32 protect_word         # 保护标志位 (Bit0..16)
uint32 fault_word           # 故障标志位 (Bit0 放MOS失效/Bit1 充MOS失效/Bit4 短路/Bit30 保险丝/Bit31 掉线)
uint32 system_word          # 系统状态1 (Bit0 放电MOS/Bit1 充电MOS/Bit5 充电器接入/Bit6 负载连接/Bit9-10 工作状态)
