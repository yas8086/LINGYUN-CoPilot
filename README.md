# 灵云01号伴飞电脑 (LINGYUN-CoPilot)

基于 **Raspberry Pi 5 + ROS2 Jazzy** 的飞艇机载数据中继节点，当前阶段聚焦**设备监控**（锂电池 BMS / 光伏 MPPT / DCDC 电源模块）。

## 系统定位

伴飞电脑作为**机载数据中继节点**，通过 CAN 总线接收各电源设备数据，解析后统一经串口数传下传给地面 Qt 上位机：

```
  MPPT ──┐
  BMS  ──┼── CAN总线 (SocketCAN can0, 250kbps, 扩展帧) ──► 树莓派5 ──► 串口数传 ──► Qt上位机
  DCDC ──┘                                               (airship_* 节点)   (JSON帧)
```

## 软件包结构

```
src/
├── airship_msgs/        # 设备监控消息定义 (BmsStatus/MpptStatus/DcdcStatus/DeviceAlert)
├── airship_can/         # SocketCAN 抽象层 (收发帧)
├── airship_utils/       # 可测工具库 (CAN解析/单位换算/限幅, 含 gtest 单测)
├── airship_bms/         # BMS 驱动节点
├── airship_mppt/        # MPPT 驱动节点
├── airship_dcdc/        # DCDC 驱动节点
├── airship_monitor/     # 设备监控聚合节点 (告警/看门狗)
├── airship_link/        # 串口数传链路节点 (JSON 下传)
└── airship_bringup/     # launch + 参数配置 (集中管理)
```

## 快速上手

### 环境

- 树莓派5 + Ubuntu 24.04 LTS + ROS2 Jazzy
- USB-CAN 适配器 → SocketCAN 接口 `can0`（250kbps）
- 串口数传模块（默认 `/dev/ttyUSB0`, 115200 baud）

### 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 一键启动

```bash
./tools/start_device_monitor.sh
```

脚本会自动 source 环境、配置 CAN 接口并启动全部监控节点。也可手动分步：

```bash
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
ros2 launch airship_bringup device_monitor.launch.py
```

### 查看设备状态

```bash
ros2 topic echo /bms/status
ros2 topic echo /mppt/status
ros2 topic echo /dcdc/status
ros2 topic echo /monitor/device_alert
```

### 测试

```bash
colcon test
colcon test-result --verbose
```

## 文档

| 文档 | 内容 |
|------|------|
| [伴飞电脑开发指南](docs/00_伴飞电脑开发指南.md) | 系统定位、环境、快速上手 |
| [设备监控架构](docs/01_设备监控架构.md) | 软硬件架构、节点拓扑、消息定义、数传协议 |
| [CAN协议汇总](docs/02_CAN协议汇总.md) | BMS/MPPT/DCDC 厂家 CAN 协议速查 |
| 厂家协议原文 | [docs/lingyun01/](docs/lingyun01/) 各设备厂家协议文件 |

## 工程规范

- **接口先行**：消息定义集中在 `airship_msgs`，避免循环依赖
- **逻辑与运输层分离**：核心业务逻辑抽为纯库（`airship_utils`）便于 gtest 单测
- **依赖单向 DAG**：自下而上 `airship_msgs → airship_can/utils → 驱动 → monitor/link → bringup`
- **lint 检查**：clang-format / cpplint / flake8，见 [.github/workflows/ci.yml](.github/workflows/ci.yml)
- **一键启动**：[tools/start_device_monitor.sh](tools/start_device_monitor.sh)

## 开发阶段

- **Phase-1（当前）**：设备监控 BMS/MPPT/DCDC，暂不接入飞控
- **Phase-2（规划）**：接入飞控与飞行控制