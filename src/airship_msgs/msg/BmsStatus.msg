# 锂电池BMS状态 (bms_ems_v01 协议)
# 由 airship_bms 节点解析 CAN 帧后发布

std_msgs/Header header

# ===== 连接状态 =====
bool online                 # CAN 链路是否在线(最近收到过有效帧)
uint32 can_id               # 本帧源 CAN ID(调试用)

# ===== 模块总压/总线 =====
float32 pack_voltage        # 电池包总压 [V]
float32 pack_current        # 电池包总电流 [A] (充电为正)
float32 soc                 # 荷电状态 [0-100 %]
float32 real_soc            # 真实 SOC [0-100 %]
float32 soh                 # 健康状态 [0-100 %]
float32 max_cell_voltage    # 最高单体电压 [V]
float32 min_cell_voltage    # 最低单体电压 [V]
float32 cell_voltage_diff   # 单体压差 [V]
float32 max_cell_temp       # 最高单体温度 [℃]
float32 min_cell_temp       # 最低单体温度 [℃]
float32 avg_cell_temp       # 平均单体温度 [℃]
float32 temp_diff           # 单体温差 [℃]
float32[8] pole_temps       # 极耳温度 [℃] (未用位置填 0)

# ===== 绝缘电阻 =====
uint16 positive_insulation_kohm   # 正极绝缘电阻 [kΩ]
uint16 negative_insulation_kohm   # 负极绝缘电阻 [kΩ]

# ===== 单体电压/温度(可变长, 长度等于实际电芯数, 未用位置填 NaN) =====
float32[] cell_voltages      # 单体电压 [V]
float32[] cell_temps         # 单体温度 [℃]

# ===== 故障与告警 (ErrorCode 帧 64 位故障/告警字, 见 BMS 协议 ErrorCode) =====
#   fault_word1: bit0-31 = 系统故障位(bit0-15) | 一级报警 Warn(bit16-31)
#   fault_word2: bit32-47 = 二级报警 Alarm
#   fault_word3: bit48-63 = 三级报警 CriticalAlarm
uint32 fault_word1          # 一级故障字 (32 位: 系统故障位 + 一级报警)
uint16 fault_word2          # 二级故障字 (16 位)
uint16 fault_word3          # 三级故障字 (16 位)
uint8 alarm_level           # 当前告警级别: 0正常 1故障 2严重