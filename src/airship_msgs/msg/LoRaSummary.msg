# 灵云01号伴飞电脑 — LoRa 温度统计汇总
builtin_interfaces/Time timestamp

bool serial_online       # 485 串口是否在线(用于区分串口掉线与节点离线)
int32 node_count         # 节点总数
int32 online_count       # 在线节点数
int32 alarm_count        # 报警节点数

float32 avg_temp          # 平均温度 (℃)
float32 max_temp          # 最高温度 (℃)
float32 min_temp          # 最低温度 (℃)
int32[] alarm_node_ids    # 报警节点 ID 列表
