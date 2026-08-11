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
int32 raw                # 温度寄存器原始值(qint16)

# 状态
int32 online             # 1=本次读取成功, 0=读取失败/离线
int32 alarm              # 0=正常, 1=超上限, -1=超下限 (仅温度节点有效)
