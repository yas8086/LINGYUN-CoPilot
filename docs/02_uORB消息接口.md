# 灵云01号 uORB 消息接口

> 本文档面向伴飞电脑开发者，列出飞控系统中所有关键uORB消息及其字段定义。
> 伴飞电脑通过 MAVLink 间接访问这些消息，MAVLink 消息与 uORB 消息存在对应关系。

## 1. 自定义消息

### ballast_setpoint — 浮力调节设定值

由 `ballast_control` 模块发布，Gazebo仿真插件订阅用于动态浮力调节。

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| timestamp | uint64 | us | 系统启动时间 |
| net_buoyancy | float32 | N | 净浮力调整量, 正=上升, 负=下降 |
| blower_left | float32 | [0,1] | 左鼓风机输出 |
| blower_right | float32 | [0,1] | 右鼓风机输出 |
| valve_left | float32 | [0,1] | 左阀门输出 |
| valve_right | float32 | [0,1] | 右阀门输出 |
| altitude_error | float32 | m | 当前高度误差 |

消息定义文件: `msg/ballast_setpoint.msg`

## 2. 飞控核心输入消息 (伴飞电脑可写入)

### vehicle_attitude_setpoint — 姿态设定值

Offboard模式下伴飞电脑通过此消息控制飞艇姿态。

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| timestamp | uint64 | us | 时间戳 |
| q_d[4] | float32 | - | 期望姿态四元数 |
| thrust_body[3] | float32 | [-1,1] | 机体坐标系推力 (X=前进, Z=升降) |
| yaw_sp_move_rate | float32 | rad/s | 偏航设定值变化率 |
| reset_integral | bool | - | 重置积分项 |

**飞艇约束**: `thrust_body[1]`(Y方向)始终为0, 无侧向控制。

### trajectory_setpoint — 轨迹设定值

Offboard/Position模式下设定目标位置/速度/加速度。

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| timestamp | uint64 | us | 时间戳 |
| position[3] | float32 | m | 本地坐标系目标位置 (NED) |
| velocity[3] | float32 | m/s | 目标速度 |
| acceleration[3] | float32 | m/s2 | 目标加速度 |
| yaw | float32 | rad | 目标偏航角 |
| yawspeed | float32 | rad/s | 目标偏航角速度 |

### offboard_control_mode — Offboard控制模式

告知飞控Offboard模式下哪些设定值有效。

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp | uint64 | 时间戳 |
| position | bool | 位置控制有效 |
| velocity | bool | 速度控制有效 |
| acceleration | bool | 加速度控制有效 |
| attitude | bool | 姿态控制有效 |
| body_rate | bool | 角速率控制有效 |
| thrust_and_torque | bool | 推力/力矩控制有效 |

### vehicle_command — 飞行器命令

用于发送模式切换、解锁/上锁等命令。

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp | uint64 | 时间戳 |
| command | uint16 | MAVLink命令ID |
| param1-7 | float32 | 命令参数 |

## 3. 飞控核心输出消息 (伴飞电脑可读取)

### vehicle_status — 飞行器状态

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp | uint64 | 时间戳 |
| arming_state | uint8 | 解锁状态 (0=DISARMED, 2=ARMED) |
| nav_state | uint8 | 导航模式 (见飞行模式文档) |
| nav_state_desc | uint8 | 导航模式描述 |
| failure_detector_status | uint8 | 故障检测状态 |
| hil_state | uint8 | HIL状态 |

### vehicle_attitude — 当前姿态

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| timestamp | uint64 | us | 时间戳 |
| q[4] | float32 | - | 姿态四元数 |
| rollspeed | float32 | rad/s | Roll角速度 |
| pitchspeed | float32 | rad/s | Pitch角速度 |
| yawspeed | float32 | rad/s | Yaw角速度 |

### vehicle_local_position — 本地位置

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| timestamp | uint64 | us | 时间戳 |
| x, y, z | float32 | m | 本地NED坐标 |
| vx, vy, vz | float32 | m/s | NED速度 |
| ax, ay, az | float32 | m/s2 | NED加速度 |
| heading | float32 | rad | 航向角 |
| ref_lat, ref_lon | double | deg | 参考点经纬度 |
| ref_alt | float32 | m | 参考点高度 |
| dist_bottom | float32 | m | 离地高度 |
| dist_bottom_valid | bool | - | 离地高度有效 |

### vehicle_land_detected — 着陆检测

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp | uint64 | 时间戳 |
| landed | bool | 是否着陆 |
| freefall | bool | 是否自由落体 |
| ground_contact | bool | 是否接地 |

**注意**: 飞艇中性浮力, 悬停静止时 `landed=true` 是正确行为, 不是bug。

### actuator_motors — 电机输出

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp | uint64 | 时间戳 |
| control[8] | float32 | 8个电机归一化输出 [0,1] |

电机映射:
- control[0] = M0 (前1上升)
- control[1] = M1 (前2下降)
- control[2] = M2 (后1下降)
- control[3] = M3 (后2上升)
- control[4] = M4 (左前推进)
- control[5] = M5 (左后推进)
- control[6] = M6 (右前推进)
- control[7] = M7 (右后推进)

### vehicle_control_mode — 控制模式

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp | uint64 | 时间戳 |
| flag_armed | bool | 是否解锁 |
| flag_control_manual_enabled | bool | 手动控制 |
| flag_control_auto_enabled | bool | 自动控制 |
| flag_control_offboard_enabled | bool | Offboard控制 |

### takeoff_status — 起飞状态

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp | uint64 | 时间戳 |
| takeoff_state | uint8 | 起飞状态 (DISARMED/SPAWNED/IN_AIR) |

## 4. uORB与MAVLink消息对应关系

伴飞电脑通过MAVLink访问飞控数据, 关键对应关系:

| uORB消息 | MAVLink消息 | 发送方向 |
|----------|------------|---------|
| vehicle_status | HEARTBEAT, SYS_STATUS | FC -> CC |
| vehicle_attitude | ATTITUDE, ATTITUDE_QUATERNION | FC -> CC |
| vehicle_local_position | LOCAL_POSITION_NED | FC -> CC |
| vehicle_global_position | GLOBAL_POSITION_INT | FC -> CC |
| vehicle_land_detected | EXTENDED_SYS_STATE | FC -> CC |
| trajectory_setpoint | SET_POSITION_TARGET_LOCAL_NED | CC -> FC |
| vehicle_attitude_setpoint | SET_ATTITUDE_TARGET | CC -> FC |
| offboard_control_mode | SET_POSITION_TARGET_LOCAL_NED (type_mask) | CC -> FC |
| vehicle_command | COMMAND_LONG / COMMAND_INT | 双向 |
| manual_control_setpoint | MANUAL_CONTROL | CC -> FC |

## 5. 伴飞电脑常用数据获取示例 (Python + MAVSDK)

```python
import asyncio
from mavsdk import System

async def monitor_airship():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # 位置
    async for position in drone.telemetry.position():
        print(f"Alt: {position.relative_altitude_m:.1f}m, "
              f"Lat: {position.latitude_deg:.6f}, "
              f"Lon: {position.longitude_deg:.6f}")
        break

    # 姿态
    async for attitude in drone.telemetry.attitude_euler():
        print(f"Pitch: {attitude.pitch_deg:.1f}, Yaw: {attitude.yaw_deg:.1f}")
        break

    # 速度
    async for velocity in drone.telemetry.velocity_ned():
        print(f"Vx: {velocity.north_m_s:.2f}, Vy: {velocity.east_m_s:.2f}, "
              f"Vz: {velocity.down_m_s:.2f}")
        break

    # 着陆状态
    async for landed in drone.telemetry.landed_state():
        print(f"Landed: {landed}")  # 飞艇悬停时为ON_GROUND, 正常
        break

asyncio.run(monitor_airship())
```
