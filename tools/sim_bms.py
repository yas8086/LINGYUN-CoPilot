#!/usr/bin/env python3
# 灵云01号伴飞电脑 — BMS 模拟器
# 通过虚拟 CAN 接口 vcan0 周期性发送 BMS 帧, 用于本地验证 bms_node 解析逻辑。
#
# 用法:
#   python3 tools/sim_bms.py                 # 默认 vcan0, 1Hz
#   python3 tools/sim_bms.py --iface vcan0 --rate 5
#
# 依赖: 内核 vcan 模块 + vcan0 接口(见 tools/setup_vcan.sh)。无需 can-utils。
"""BMS 虚拟设备模拟器, 向 vcan0 周期发送 BMS 协议帧。"""

import argparse
import socket
import struct
import time

# BMS 帧 ID(29 位扩展帧, 与 bms_protocol 一致)
BATT_INFO02 = 0x001400      # 总压/总电流/SOC
BATT_INFO01 = 0x001300      # 运行状态/报警级别
CELL_VOLT_BASE = 0x003000   # 单体电压, 每帧 +0x10
CELL_TEMP_STAT = 0x001500   # 温度统计
PACK_TEMP = 0x002110        # 极耳温度 8 点

CELLS_PER_FRAME = 5
TOTAL_CELLS = 102  # 三元锂 102 串


def open_can_socket(iface_name):
    """创建绑定到指定接口的 CAN_RAW 套接字。"""
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((iface_name,))
    return sock


def send(sock, can_id, data):
    """发送 8 字节扩展帧。"""
    assert len(data) == 8, "CAN 帧必须为 8 字节"
    flags = socket.CAN_EFF_FLAG
    sock.send(struct.pack("=IB3x8s", can_id | flags, 8, bytes(data)))


def pack_u16_le(value):
    """小端 u16。"""
    return (value & 0xFF, (value >> 8) & 0xFF)


def pack_cell_voltage_frame(cells):
    """按 DBC 的 12 位 Motorola 布局编码 5 节单体电压。

    布局(每节 12 位, raw = 电压*1000 - 1000):
      节0: b0<<4 | (b1>>4)    节1: (b1&0x0F)<<8 | b2
      节2: b3<<4 | (b4>>4)    节3: (b4&0x0F)<<8 | b5
      节4: (b6&0x0F)<<8 | b7
    """
    assert len(cells) == CELLS_PER_FRAME
    raw = [int(round(v * 1000.0 - 1000.0)) for v in cells]
    b = [0] * 8
    b[0] = (raw[0] >> 4) & 0xFF
    b[1] = ((raw[0] & 0x0F) << 4) | ((raw[1] >> 8) & 0x0F)
    b[2] = raw[1] & 0xFF
    b[3] = (raw[2] >> 4) & 0xFF
    b[4] = ((raw[2] & 0x0F) << 4) | ((raw[3] >> 8) & 0x0F)
    b[5] = raw[3] & 0xFF
    b[6] = (raw[4] >> 8) & 0x0F
    b[7] = raw[4] & 0xFF
    return bytes(b)


def build_cells():
    """生成 102 节电压, 设计高低压差以便验证 max/min/diff。"""
    cells = []
    for i in range(TOTAL_CELLS):
        # 正弦波动以覆盖 max/min, 末节压差明显
        v = 3.30 + 0.05 * ((i % 8) / 7.0)
        cells.append(round(v, 3))
    # 制造明显高低: 第 3 节偏低, 第 50 节偏高
    cells[2] = 3.10
    cells[49] = 3.60
    return cells


def main():
    parser = argparse.ArgumentParser(description="BMS 虚拟设备模拟器")
    parser.add_argument("--iface", default="vcan0", help="CAN 接口名(默认 vcan0)")
    parser.add_argument("--rate", type=float, default=1.0, help="发送频率 Hz(默认 1)")
    args = parser.parse_args()

    cells = build_cells()
    sock = open_can_socket(args.iface)
    period = 1.0 / args.rate
    print(f"[sim_bms] 绑定 {args.iface}, 频率 {args.rate}Hz, {TOTAL_CELLS} 节")
    print(f"[sim_bms] 预期: 第3节={cells[2]}V, 第50节={cells[49]}V(压差验证)")

    try:
        while True:
            # BattInfo02: 总压 = 单节均值*102, 电流 5.0A, SOC 88%
            pack_voltage = round(sum(cells) / TOTAL_CELLS * TOTAL_CELLS, 1)
            # 总压近似: 用均值*102 (模拟值)
            pack_v_raw = int(round(pack_voltage * 10.0))
            pack_i_raw = int(round((5.0 + 100.0) * 10.0))   # raw = (I+100)/0.1
            soc_raw = int(round(88.0 * 10.0))
            data02 = bytearray(
                pack_u16_le(pack_v_raw) + pack_u16_le(pack_i_raw) +
                pack_u16_le(soc_raw) + pack_u16_le(soc_raw))
            send(sock, BATT_INFO02, data02)

            # BattInfo01: run_state=1, conn=2, 绝缘 1000/1100kΩ, 报警 3
            data01 = bytearray(
                [0x21, 0x00] + list(pack_u16_le(1000)) + list(pack_u16_le(1100)) + [0x03, 0x00])
            send(sock, BATT_INFO01, data01)

            # CellVoltage_XX: 每帧 5 节
            for f in range((TOTAL_CELLS + CELLS_PER_FRAME - 1) // CELLS_PER_FRAME):
                start = f * CELLS_PER_FRAME
                frame_cells = cells[start:start + CELLS_PER_FRAME]
                if len(frame_cells) < CELLS_PER_FRAME:
                    frame_cells += [0.0] * (CELLS_PER_FRAME - len(frame_cells))
                send(sock, CELL_VOLT_BASE + f * 0x10, pack_cell_voltage_frame(frame_cells))

            # 温度统计: max=45, min=20, avg=32, diff=25
            data_temp = bytearray([45 + 50, 0, 0, 20 + 50, 0, 0, 32 + 50, 25 + 50])
            send(sock, CELL_TEMP_STAT, data_temp)

            # PACK 极耳温度 8 点: 30~37℃
            data_pack = bytearray([30 + 50 + i for i in range(8)])
            send(sock, PACK_TEMP, data_pack)

            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[sim_bms] 停止")
    finally:
        sock.close()


if __name__ == "__main__":
    main()