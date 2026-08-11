# 安全状态消息 (由 airship_safety 节点聚合各安全判据后发布)
# 用于决定"是否允许下发控制命令"(safe_to_control)。
# 设计原则(fail-safe): 任何关键判据异常时 safe_to_control=false。
# 注意: DCDC 控制当前由独立保活进程 dcdc_hold 维持, 不因本标志断电
#       (48V 为飞艇弱电主源, 有备用电池并联但撑不久)。本消息供后续
#       接入 DCDC/飞控等控制时使用。

std_msgs/Header header

# ===== 总开关 =====
bool safe_to_control        # 是否允许下发控制命令 (所有判据均正常时为 true)

# ===== 各判据 =====
bool dcdc_ok                # DCDC 自身无故障 (fault_word 无故障位)
bool battery_ok             # 主 BMS 电压在合理范围内
bool backup_battery_ok      # 12S 备用电源 BMS 在线且无故障/总压正常
bool flight_armed           # 飞控是否已解锁 (默认 false, 未启用判据)

# ===== 详情 =====
string reason               # 人类可读的不安全原因 (空串表示安全)