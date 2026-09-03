# 灵云01号伴飞电脑 — LoRa 温度/压力集中器单节点采样
# 由 airship_lora::lora_node 通过 485 Modbus RTU 轮询集中器生成

# 节点标识
int32 node_id            # LoRa 节点 ID (从 start_node_id 递增)
bool is_pressure         # true=压力传感器, false=温度传感器

# 采集时间
builtin_interfaces/Time timestamp

# 解析值
float32 temp_celsius     # 温度 (℃), 压力节点无效
float64 pressure_pa      # 压力 (Pa), 温度节点为 0
int32 raw                # 温度寄存器原始值(int16 补码, 负值=负温度)

# 状态
int32 online             # 1=在线(本轮读取成功或离线去抖沿用旧值), 0=读取失败/离线
int32 alarm              # 0=正常, 1=超上限, -1=超下限 (仅温度节点有效)
# 离线去抖标记: 1=本轮读取失败, 数值为离线去抖窗口内沿用的上一帧有效值(非本轮实测);
#               0=本轮实测有效。地面站可据此灰显/标注陈旧数据 (2026-09-03)
int32 stale
# 分量有效性: 区分"部分成功"轮次(如温度读失败但压力读成功), 防 0 值被当真值
int32 temp_valid           # 1=温度寄存器本轮读取成功(temp_celsius 为真实测量)
int32 press_valid          # 1=压力寄存器本轮读取成功(仅压力节点)
