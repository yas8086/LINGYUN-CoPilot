# 灵云01号 气囊压差同步消息 (树莓派 LoRa 采集 -> uXRCE-DDS -> PX4 飞控)
# 树莓派侧: bladder_bridge_node 发布到 /fmu/in/airship_bladder_pressure
# PX4 侧:   本文件复制到 PX4-Autopilot/msg/ 并在 uxrce_dds_client/dds_topics.yaml
#           的 subscriptions 段登记后, 控制模块以 uORB 话题 airship_bladder_pressure 订阅
#
# 槽位顺序固定为 [左副囊, 左主囊, 右主囊, 右副囊], 对应 LoRa node_id [6,13,14,15], 不可重排
# 压差为相对大气压的有符号值 (Pa), 0 Pa = 与大气压持平; 负值 = 低于大气压

uint64 timestamp			# [us] 树莓派系统时钟; uXRCE-DDS client timesync 自动换算为 PX4 时钟
uint64 timestamp_sample		# [us] LoRa 本轮采样时刻(树莓派时钟), 用于新鲜度判断

# ---- 槽位常量(数组下标) ----
uint8 BLADDER_LEFT_AUX = 0		# 左副囊 (LoRa node_id=6)
uint8 BLADDER_LEFT_MAIN = 1		# 左主囊 (LoRa node_id=13)
uint8 BLADDER_RIGHT_MAIN = 2		# 右主囊 (LoRa node_id=14)
uint8 BLADDER_RIGHT_AUX = 3		# 右副囊 (LoRa node_id=15)

# ---- 4 路压差 [Pa] (相对大气压, 有符号; bridge 从未收到有效数据时为 NaN) ----
float32[4] pressure_delta_pa

# ---- 4 路节点自带温度 [degC] (无效为 NaN) ----
float32[4] temperature_c

# ---- 槽位状态标志 ----
# valid: 1=本轮真实测量(LoRa 本轮 press_valid=1 且非沿用轮)
uint8[4] valid
# stale: 1=该槽位为沿用历史值(LoRa 离线去抖/读取失败/节点缺失), 数值非本轮实测
uint8[4] stale
