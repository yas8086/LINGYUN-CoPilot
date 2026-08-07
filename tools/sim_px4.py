#!/usr/bin/env python3
# 灵云01号伴飞电脑 — PX4 飞控 telem2 串口模拟器
# 通过虚拟串口(见 tools/setup_sim_px4.sh)周期发送 MAVLink 数据流, 供 MAVROS 连接,
# 从而在本地无真实飞控时验证 fc_monitor -> cloud_node 的飞控数据上传链路。
#
# 发送的消息(覆盖 fc_monitor 全部输入, 也即 cloud_node 对飞控数据的解析来源):
#   HEARTBEAT              -> /mavros/state    (connected/armed/flight_mode)
#   ATTITUDE               -> /mavros/imu/data (roll/pitch/yaw -> 欧拉角)
#   GLOBAL_POSITION_INT    -> /mavros/global_position/global (lat/lon/alt)
#   LOCAL_POSITION_NED     -> /mavros/global_position/local (rel_alt/vx/vy/vz)
#   BATTERY_STATUS         -> /mavros/battery   (voltage/current/remaining)
#
# 飞行阶段状态机: 模拟一次完整起降, 随时间推进解锁/爬升/巡航/降落,
# 使各种飞行模式与高度/速度/姿态动态变化, 充分验证云上传的飞控数据。
#
# 用法:
#   # 先创建虚拟串口(见 setup_sim_px4.sh), 再:
#   python3 tools/sim_px4.py --device /tmp/ttySimPX4 --rate 10
"""PX4 飞控 telem2 串口 MAVLink 数据模拟器(带飞行阶段状态机)。"""

import argparse
import math
import time

from pymavlink import mavutil

# PX4 主模式(main mode): 见 px4_custom_mode, 编码在 custom_mode 的 bit16-23
PX4_MODE_STABILIZED = 1
PX4_MODE_ALTCTL = 5
PX4_MODE_POSCTL = 6
PX4_MODE_AUTO = 7

# 飞行阶段时间轴(秒), 模拟一次完整起降
PHASE_TAKEOFF = 0.0    # 待机, 未解锁
PHASE_ARM = 5.0        # 解锁(自稳)
PHASE_CLIMB = 10.0     # 爬升(定高/位置控制)
PHASE_CRUISE = 40.0    # 巡航(自动)
PHASE_DESCEND = 75.0   # 降落(自动)
PHASE_END = 100.0      # 结束

# 起飞点(GPS)
HOME_LAT = 30.2741
HOME_LON = 120.1551
HOME_ALT = 100.0
# 巡航高度(m)
CRUISE_ALT = 25.0


def phase_of(t):
    """返回当前阶段的 (armed, main_mode, 是否已起飞)。"""
    if t < PHASE_ARM:
        return False, PX4_MODE_STABILIZED, False
    if t < PHASE_CLIMB:
        return True, PX4_MODE_STABILIZED, False
    if t < PHASE_CRUISE:
        return True, PX4_MODE_POSCTL, True
    if t < PHASE_DESCEND:
        return True, PX4_MODE_AUTO, True
    if t < PHASE_END:
        return True, PX4_MODE_AUTO, True
    return False, PX4_MODE_STABILIZED, False


def rel_alt_at(t):
    """相对高度随阶段变化: 待机 0, 爬升段线性到巡航高度, 巡航保持, 降落段下降。"""
    if t < PHASE_CLIMB:
        return 0.0
    if t < PHASE_CRUISE:
        k = (t - PHASE_CLIMB) / (PHASE_CRUISE - PHASE_CLIMB)
        return CRUISE_ALT * k
    if t < PHASE_DESCEND:
        return CRUISE_ALT
    if t < PHASE_END:
        k = (t - PHASE_DESCEND) / (PHASE_END - PHASE_DESCEND)
        return CRUISE_ALT * (1.0 - k)
    return 0.0


def send_heartbeat(master, armed, main_mode):
    """HEARTBEAT: 供 MAVROS 判断连接/上锁/飞行模式。"""
    base = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        base |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        base |= mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
    custom_mode = main_mode << 16  # PX4: main_mode 在 bit16-23
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_PX4,
        base,
        custom_mode,
        mavutil.mavlink.MAV_STATE_ACTIVE)


def send_attitude(master, roll, pitch, yaw):
    """ATTITUDE: 姿态角(rad), MAVROS 内部转四元数。"""
    master.mav.attitude_send(
        int(time.time() * 1000) & 0xFFFFFFFF, roll, pitch, yaw, 0.0, 0.0, 0.0)


def send_global_pos_int(master, lat, lon, alt_msl, rel_alt, vx_cm, vy_cm, vz_cm):
    """GLOBAL_POSITION_INT: lat/lon(1e7), alt(mm), 速度(cm/s)。"""
    master.mav.global_position_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        int(lat * 1e7), int(lon * 1e7),
        int(alt_msl * 1000), int(rel_alt * 1000),
        int(vx_cm), int(vy_cm), int(vz_cm), 0)


def send_local_pos_ned(master, x, y, z, vx, vy, vz):
    """LOCAL_POSITION_NED: 机体系位置/速度(m, m/s, NED)。"""
    master.mav.local_position_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF, x, y, z, vx, vy, vz)


def send_battery(master, voltage, current, remaining):
    """BATTERY_STATUS: 电压(mV), 电流(cA), 剩余(0-100)。"""
    master.mav.battery_status_send(
        0,  # id
        mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
        mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,
        0x7FFF,  # 温度未知
        [int(voltage * 1000)] + [0] * 9,  # voltages[0] = 总电压 mV
        int(current * 100),   # current_battery 单位 cA
        -1, -1,               # current_consumed / energy_consumed 未知
        int(remaining))       # battery_remaining 0-100


def current_for(phase, rel_alt):
    """根据飞行阶段返回电池电流(A)与剩余电量(%)。"""
    # 爬升/巡航/降落电流不同, 剩余电量随时间下降
    if phase == 'takeoff':
        return 0.1, 100.0
    if phase == 'climb':
        return 8.5, max(1.0, 100.0 - 0.5 * time.time() % 100)
    if phase == 'cruise':
        return 4.0, max(1.0, 100.0 - 0.5 * time.time() % 100)
    if phase == 'descend':
        return 3.0, max(1.0, 100.0 - 0.5 * time.time() % 100)
    return 0.1, 87.0


def main():
    parser = argparse.ArgumentParser(description='PX4 telem2 串口 MAVLink 模拟器')
    parser.add_argument('--device', default='/tmp/ttySimPX4',
                        help='模拟串口设备路径(由 setup_sim_px4.sh 创建)')
    parser.add_argument('--baud', type=int, default=921600, help='波特率(pty 下无效)')
    parser.add_argument('--rate', type=float, default=10.0, help='消息组发送频率(Hz)')
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.device, baud=args.baud, force_mavlink20=True)
    print(f'[sim_px4] 已连接 {args.device}, 开始发送 MAVLink@{args.rate}Hz ...')

    t0 = time.time()
    while True:
        t = time.time() - t0

        armed, main_mode, flying = phase_of(t)
        r = rel_alt_at(t)

        # 姿态随飞行阶段动态变化
        roll = 0.10 * math.sin(2 * math.pi * 0.10 * t)
        pitch = 0.20 * math.sin(2 * math.pi * 0.05 * t)
        yaw = math.pi / 2.0 + 0.30 * math.sin(2 * math.pi * 0.02 * t)

        # 位置: 巡航时沿 x 方向前进, 高度随阶段变化
        # 速度: 爬升/降落有垂直速度, 巡航有水平速度
        if r > 0.05:
            vx, vy, vz = 5.0, 0.0, 0.0
        else:
            vx, vy, vz = 0.0, 0.0, 0.0

        # NED: z 向上为负, 高度 r 对应 z = -r
        send_heartbeat(master, armed, main_mode)
        send_attitude(master, roll, pitch, yaw)
        send_global_pos_int(master, HOME_LAT, HOME_LON, HOME_ALT, r, int(vx * 100), int(vy * 100), int(vz * 100))
        send_local_pos_ned(master, vx * (t % 100), 0.0, -r, vx, vy, vz)

        # 电池: 不同阶段电流不同, 电量随时间回落
        if armed:
            phase = 'climb' if r < CRUISE_ALT * 0.9 else ('descend' if t > PHASE_DESCEND else 'cruise')
            current, remaining = current_for(phase, r)
        else:
            current, remaining = current_for('takeoff', r)
        send_battery(master, 16.8, current, remaining)

        time.sleep(1.0 / args.rate)


if __name__ == '__main__':
    main()