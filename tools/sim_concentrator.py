#!/usr/bin/env python3
"""LoRa 集中器 Modbus RTU 从机模拟器。
监听一个串口, 收到 lora_node 的读请求(功能码 0x04)后按寄存器地址回温度/压力帧。
支持串口断开后自动重连, 用于模拟集中器"掉线->恢复"。
"""
import argparse
import serial
import time

# 与 lora_node 一致的寄存器基址
TEMP_REG_BASE = 0x76C1
PRESSURE_REG_BASE = 0x8EF9


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_resp(slave, func, values):
    """values: 要回的数据字节 (不含从机/功能码/bytecount/CRC)
    按 Modbus RTU 标准: [slave][func][bytecount][data...][crcLo][crcHi]
    bytecount = 数据字节数。
    """
    payload = bytes([slave, func, len(values)]) + bytes(values)
    c = crc16(payload)
    return payload + bytes([c & 0xFF, (c >> 8) & 0xFF])


def handle_frame(frame, slave):
    """解析 Modbus RTU 请求并返回响应帧; 非法返回 None"""
    if len(frame) < 8 or frame[0] != slave or frame[1] != 0x04:
        return None
    req_crc = (frame[-2] | (frame[-1] << 8))
    if crc16(frame[:-2]) != req_crc:
        return None
    addr = (frame[2] << 8) | frame[3]
    qty = (frame[4] << 8) | frame[5]
    if qty == 1:  # 温度: raw = 295 (29.5℃)
        return build_resp(slave, 0x04, [0x01, 0x27])
    if qty == 2:  # 压力: 17 Pa
        return build_resp(slave, 0x04, [0x00, 0x00, 0x00, 0x11])
    return None


def main():
    parser = argparse.ArgumentParser(description="LoRa 集中器 Modbus 从机模拟器")
    parser.add_argument("--device", default="/tmp/ttyConcentrator", help="集中器端串口")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--slave", type=int, default=1)
    args = parser.parse_args()

    print(f"[sim_concentrator] 监听 {args.device} @ {args.baud}, 从机 {args.slave}")
    while True:
        try:
            ser = serial.Serial(args.device, args.baud, timeout=0.1)
            print(f"[sim_concentrator] 已连接 {args.device}")
        except Exception as e:  # noqa: BLE001
            print(f"[sim_concentrator] 连接失败: {e}, 1s 后重试")
            time.sleep(1)
            continue
        try:
            while True:
                # 读一帧 (从机地址开始, 最多 8 字节)
                head = ser.read(1)
                if not head:
                    continue
                if head[0] != args.slave:
                    continue
                rest = ser.read(7)
                if len(rest) < 7:
                    continue
                frame = head + rest
                print(f"[sim_concentrator] 收到请求: {frame.hex()}", flush=True)
                resp = handle_frame(frame, args.slave)
                if resp:
                    ser.write(resp)
                    print(f"[sim_concentrator] 回帧: {resp.hex()}", flush=True)
        except (serial.SerialException, OSError):
            print("[sim_concentrator] 串口断开, 尝试重连")
            time.sleep(1)
            continue
        finally:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
