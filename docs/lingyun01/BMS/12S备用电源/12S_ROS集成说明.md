# 12S 备用电源 BMS — ROS2 集成说明

> 协议原文：`北辰串口通迅协议1.4－2(7).pdf`（本目录）。
> 本文档为机载 ROS2 集成速查，供后续维护参考。

## 设备概述

12S 备用电源为飞艇**备用**锂电池 BMS，与主 BMS(CAN) 不同，走**串口**（北辰协议 V1.4）。

- 串口参数：**默认 9600 8N1，无校验位**
- 协议层：请求-响应（上位机轮询，从机应答）

## 帧格式

| 发送 | 0x56 | 地址 | 主机 | 读写 | 指令 | 长度H | 长度L | 数据(L) | CRC-H | CRC-L |
|------|------|------|------|------|------|-------|-------|---------|-------|-------|
| 响应 | 0x57 | 地址 | 主机 | 读写 | 指令 | 长度H | 长度L | 数据(L) | CRC-H | CRC-L |

- 地址默认 `0xFF`；主机信息（上位机）= `0x10`
- 读写：`0`=读，`1`=写
- 长度（2 字节，高在前）只包含数据长度 `L`
- **CRC16**（0xA001，初值 0xFFFF）覆盖"地址..数据"所有字节，高字节在前

读请求示例（读基本信息 0x06）：`56 FF 10 00 06 00 00 D6 35`

## 轮询指令

| 指令 | 内容 | 数据段要点 |
|------|------|-----------|
| 0x06 | 基本信息1 | 总压(0.01V u16)、总电流(0.01A s32)、SOC(0.1%)、SOH(0.1%)、最大/最小电压(1mV)、压差(1mV)、最大/最小/平均温度(0.1℃)、温差、MOS/工作温度、告警(32位)、保护(32位)、故障(32位)、系统状态(32位) |
| 0x07 | 单节温度 | 数量(u16) + 逐点温度(0.1℃ s16) |
| 0x08 | 单节电压 | 数量(u16) + 逐节电压(1mV) |

> 帧序号 7..N 对应数据段偏移 0..N-7。所有 16/32 位数值均为**大端**（高字节在前）。

## 状态字位定义（0x06）

- **告警/保护**：Bit0 单体电压高、Bit1 单体电压低、Bit2 总压高、Bit3 总压低、Bit4 放流1、Bit5 放流2、Bit6 充流、Bit7 充高温、Bit8 放高温、Bit9 充欠温、Bit10 放欠温、Bit11 MOS高温、Bit12 SOC低、Bit13 环境高温、Bit14 环境低温、Bit15 温差大、Bit16 压差大
- **故障**：Bit0 放MOS失效、Bit1 充MOS失效、Bit4 短路、Bit30 保险丝熔断、Bit31 掉线
- **系统状态1**：Bit0 放MOS、Bit1 充MOS、Bit2 预放MOS、Bit5 充电器接入、Bit6 负载连接、Bit9-10 工作状态(00空闲/01放电/10充电)

## ROS2 集成

- 包：`airship_backup_bms`
  - `backup_bms_protocol`（纯库，可 gtest）：CRC16 / 读请求构建 / 响应帧校验 / 0x06/0x07/0x08 解析
  - `backup_bms_node`：串口轮询 + 自动重连，发布 `/backup_bms/status`（`airship_msgs/BackupBmsStatus`）
- 串口：`/dev/airship_backup_bms`（udev 符号链接，规则见 `tools/70-airship-usb.rules`，VID/PID 待确认后启用）
- 接入链路：monitor 看门狗/告警、link JSON 下传(`backup` 字段)、cloud 上云、safety 安全仲裁
- 安全仲裁：`safety_node` 订阅 `/backup_bms/status`，判据为「在线 且 总压 ≥ `backup_bms_min_voltage` 且 `fault_word==0`」，
  任一不满足即 `safe_to_control=false`（fail-safe），结果写入 `/safety/status` 的 `backup_battery_ok`。

## 参数（airship_params.yaml）

```yaml
backup_bms_node:
  ros__parameters:
    serial_device: "/dev/airship_backup_bms"
    baud_rate: 9600
    addr: 255          # 0xFF
    host: 16           # 0x10
    sample_period_ms: 1000
    resp_timeout_ms: 200
    reconnect_ms: 2000
```

`safety_node` 相关参数：

```yaml
safety_node:
  ros__parameters:
    backup_bms_timeout_s: 3.0      # 备用电源判据超时 (s)
    backup_bms_min_voltage: 24.0   # 备用电源总压下限 (V) (12S 磷酸铁锂 27Ah, 过放保护 ~24V)
```

## 现场需确认

- 备用电源实际 USB 串口芯片的 **VID/PID**（用于启用 udev 固定符号链接）
- 若从机地址/主机信息非默认值，需按实测调整 `addr`/`host`
