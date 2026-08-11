# LORA 采集 · ROS2 节点设计（lorach_ros）

## 背景

现有 LoRa 温度压力监测系统是一个 Qt6 桌面应用（`LoRaTemperature`），通过串口 Modbus RTU 轮询 LoRa 集中器，采集多节点温度/压力。用户需要在**树莓派**上以 **ROS2 节点**形式实现同样的采集能力，并对外提供**查询与汇总**接口，供上层（如飞艇控制系统）集成。

本文档是 ROS2 节点的设计规格，复用现有桌面应用的 Modbus 协议逻辑。

## 目标平台与环境

- 树莓派，ROS2 **Jazzy**（C++ / ament_cmake）
- 串口通过 **termios** 直接操作，**无 Qt 依赖**
  （树莓派环境纯净，Qt SerialBus 依赖过重，非 ROS 生态标准做法）
- 代码放在当前仓库内新建的 `lorach_ros/` 包目录，与 Qt 桌面应用共存

## 数据来源与协议（复用现有逻辑）

复用 [src/ModbusWorker.cpp](file:///home/hex/LINGYUN/Projects/LoRaTemperature/src/ModbusWorker.cpp) 与 [src/Sample.h](file:///home/hex/LINGYUN/Projects/LoRaTemperature/src/Sample.h) 的协议，纯 C++ 移植（无 Qt）：

| 项目 | 值 |
|------|-----|
| 功能码 | 0x04（读输入寄存器） |
| 温度寄存器 | 0x76C1 + (nodeId-1)，1 寄存器，`qint16(raw)/10.0` ℃ |
| 压力寄存器 | 0x8EF9 + (nodeId-1)*2，2 寄存器，`high<<16 \| low` Pa |
| 压力节点 | 默认 ID 6（逗号分隔可扩展） |
| CRC | CRC-16/MODBUS |
| 报警阈值 | 全局 [-10, 60]，`checkAlarm` |

默认参数（复用 AppConfig 默认值）：
- 串口 `/dev/ttyUSB0`，波特率 `9600`，8 数据位，1 停止位，无校验
- 从机地址 `1`，起始 ID `1`，节点数 `2`，采样周期 `2000ms`

## 架构

```
lorach_ros/
├── msg/
│   ├── LoRaSample.msg      # 单节点采样（结构化字段）
│   ├── LoRaSamples.msg     # 一轮采集的所有节点
│   └── LoRaSummary.msg     # 统计汇总
├── srv/
│   ├── QueryNode.srv       # 按节点ID查询最新值
│   └── QuerySummary.srv    # 主动触发汇总
├── src/
│   ├── LoraModbusClient.h/.cpp   # termios 串口 + Modbus RTU 帧（纯 C++，与 ROS 解耦）
│   └── lorach_query_node.cpp     # 主节点：采集 + 发布 + 服务
├── launch/lorach_query.launch.py
├── CMakeLists.txt
└── package.xml
```

### 核心组件

**LoraModbusClient**（纯 C++，独立可测）
- `open(port, baud, ...)`：termios 配置串口
- `readAllNodes(nodeCount, startId, pressureIds)`：构建并发送一轮读请求，同步等待响应，返回 `std::vector<Sample>`
- 内部实现 CRC、帧构建、温度/压力解析、报警判定
- 线程安全：单采集线程使用，通过互斥锁保护

**lorach_query_node**（ROS2 节点）
- 一个采集线程（`std::thread` + 原子停止标志），定时器驱动每 `samplePeriodMs` 采集一轮
- 维护 `node_id → Sample` 最新缓存（互斥锁保护）
- 发布：
  - `/lorach/samples`（`LoRaSamples`）：每轮采集后发布原始数据
  - `/lorach/summary`（`LoRaSummary`）：周期发布统计汇总
- 服务：
  - `/lorach/query_node`（`QueryNode`）：按 node_id 返回最新 Sample
  - `/lorach/query_summary`（`QuerySummary`）：主动触发一次性汇总

## 消息定义

### LoRaSample.msg
```
int32 node_id
builtin_interfaces/Time timestamp
float32 temp_celsius
float32 pressure_pa
int32 raw
int32 online        # 1=在线, 0=离线
int32 alarm         # 0 正常 / 1 超上限 / -1 超下限
bool is_pressure
```

### LoRaSamples.msg
```
builtin_interfaces/Time timestamp
LoRaSample[] samples
```

### LoRaSummary.msg
```
builtin_interfaces/Time timestamp
int32 node_count          # 节点总数
int32 online_count        # 在线节点数
int32 alarm_count         # 报警节点数
float32 avg_temp
float32 max_temp
float32 min_temp
int32 alarm_node_ids[]    # 报警节点ID列表
```

### QueryNode.srv
```
int32 node_id
---
bool success
string message
LoRaSample sample
```

### QuerySummary.srv
```
---
bool success
string message
LoRaSummary summary
```

## 服务/话题接口汇总

| 类型 | 名称 | 方向 |
|------|------|------|
| Publisher | `/lorach/samples` | 每轮原始数据 |
| Publisher | `/lorach/summary` | 周期统计汇总 |
| Service | `/lorach/query_node` | 按节点查询最新值 |
| Service | `/lorach/query_summary` | 主动触发汇总 |

## 错误处理

- 串口打不开：节点启动失败，LOG_ERROR 并退出
- 单节点读超时/失败：该节点标 `online=0`，不影响其他节点，持续采集
- 连续失败计数：超过阈值（如 3 次）LOG_WARN 提示通讯异常，采集不中断
- 服务查询时节点不在缓存：返回 `success=false` + 错误消息

## 测试策略

- **单元测试**：`LoraModbusClient` 的协议解析（CRC、温度/压力解析、报警判定）可脱离硬件测试，用 `ament_add_gtest`
- **集成**：树莓派连接真实集中器验证采集；无硬件时用 `socat` 虚拟串口对 + 模拟 Modbus 从机回帧验证
- **ROS 冒烟**：`ros2 node list`、`ros2 topic echo /lorach/samples`、`ros2 service call` 验证四条接口

## 与现有应用的边界

- ROS2 节点**独立于** Qt 桌面应用，两者共用同一集中器/协议，但代码独立
- 若未来需要两套同时读同一串口，需注意串口独占冲突；本设计假定同一时刻仅一个进程占用串口