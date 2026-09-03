# 08 气囊压差 uXRCE-DDS 同步协议文档

| 项目 | 内容 |
|------|------|
| 文档目的 | 树莓派 4 路气囊压差数据实时同步至 PX4 飞控（供二次开发控制逻辑消费） |
| 适用固件 | PX4-Autopilot v1.17（二次开发版） |
| 传输链路 | uXRCE-DDS（MicroXRCEAgent + uxrce_dds_client），UDP 8888 |
| 数据频率 | 0.5 Hz（LoRa 采集周期 2s，物理上限） |
| 编写日期 | 2026-09-03 |

---

## 1. 概述

### 1.1 链路架构

```
[LoRa 压力传感器 x4]                [树莓派 10.41.10.100]              [PX4 飞控 10.41.10.2]
 6=左副囊 13=左主囊   ──485 Modbus──▶ lora_node                          uxrce_dds_client
 14=右主囊 15=右副囊                 │  └─ /lora/samples (0.5Hz)         │  uORB: airship_bladder_pressure
                                    │ bladder_bridge_node                │  └─ 控制逻辑模块订阅
                                    │  └─ /fmu/in/airship_bladder_pressure
                                    │ MicroXRCEAgent (UDP 8888) ◀────────┘
                                    └────────────── 交换机 ──────────────┘
```

- 与现有 MAVROS（UDP 14550，只读监控）链路**独立共存**，端口不冲突。
- 飞控 eth 转 RJ45 网口与树莓派网口统一接入交换机，同网段 10.41.10.0/24。
- 压差语义：**相对大气压的有符号值**（0 Pa=与大气压持平，负值=低于大气压）。

### 1.2 数据流职责划分

| 端 | 职责 |
|----|------|
| 树莓派（本工程，已全部完成） | LoRa 采集 → 槽位映射/沿用语义 → px4_msgs 发布 → Agent 承载 |
| PX4（配合项） | 复制 msg + dds_topics.yaml 登记条目 + 重编固件 + 启动 client + 控制逻辑订阅 |

---

## 2. 消息定义

### 2.1 AirshipBladderPressure.msg（全文）

> 树莓派侧文件：`src/px4_msgs/msg/AirshipBladderPressure.msg`
> PX4 侧复制到：`PX4-Autopilot/msg/AirshipBladderPressure.msg`（**一字不改**）

```
# 灵云01号 气囊压差同步消息 (树莓派 LoRa 采集 -> uXRCE-DDS -> PX4 飞控)
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
```

### 2.2 槽位语义表（消费逻辑必读）

| valid | stale | pressure_delta_pa 含义 | 建议 |
|-------|-------|------------------------|------|
| 1 | 0 | 本轮真实测量（2s 内新鲜） | 正常使用 |
| 0 | 1 | 沿用最近一次有效值（**非本轮实测**，可能已过期数秒） | 谨慎使用/灰显，叠加时间超时判断 |
| 0 | 0 | `NaN`，该槽位自 bridge 启动以来从未有效 | 不可用 |

**注意**：消息以 0.5Hz 持续发布，即使数据源故障期间也是（沿用值/NaN）。
**消费逻辑不得以"持续收到消息"作为数据新鲜判据**，必须检查 `valid/stale` 并叠加时间超时
（参考阈值：>10s 无 valid=1 槽位停用该囊逻辑，>15s 整体 failsafe）。

### 2.3 时间戳机制

PX4 v1.17 的 `uxrce_dds_client` 内置 XRCE PING/PONG 时间同步（`UXRCE_DDS_SYNCT` 默认开启），
收到 TOFC 消息时自动把树莓派系统时钟（`timestamp` 字段，微秒）换算为 PX4 时钟。
树莓派侧无需运行任何 timesync 应答节点。
新鲜度建议用 `timestamp_sample` 与 PX4 本地时刻之差（树莓派无 RTC，重启后时钟依赖 NTP）。

---

## 3. PX4 侧配合步骤（完整可执行）

### 3.1 复制消息定义

```bash
cp <树莓派工程>/src/px4_msgs/msg/AirshipBladderPressure.msg  PX4-Autopilot/msg/
```

文件内容一字不改（DDS 序列化按字段完全一致对通）。

### 3.2 登记 uXRCE-DDS 话题条目

编辑 `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`，
在 **`subscriptions:`** 列表（ROS2→飞控方向）末尾追加：

```yaml
  - topic: /fmu/in/airship_bladder_pressure
    type: px4_msgs::msg::AirshipBladderPressure
```

> **重要**：v1.17 的映射文件是 `src/modules/uxrce_dds_client/dds_topics.yaml`。
> 网上很多资料（含 PX4 main 分支文档）写的 `msg/metadata/Uxrce-dds.yaml`（带 `dir:` 字段）
> 是 v1.18+ 的新格式，**不适用于 v1.17**。

### 3.3 重新编译飞控固件

正常执行你们的固件编译流程即可。构建系统自动为 `msg/AirshipBladderPressure.msg`
生成 uORB 代码：头文件 `uORB/topics/airship_bladder_pressure.h`、结构体 `airship_bladder_pressure_s`。

### 3.4 启动 uxrce_dds_client（飞控侧）

**方式 A（推荐）：SD 卡 `etc/extras.txt`**

```
uxrce_dds_client start -t udp -p 8888 -h 10.41.10.100
```

**方式 B：QGC 参数**（需重启生效）

| 参数 | 值 | 说明 |
|------|-----|------|
| UXRCE_DDS_CFG | Ethernet | 走以太网口 |
| UXRCE_DDS_PRT | 8888 | Agent 端口 |
| UXRCE_DDS_AG_IP | 170461796 | Agent IP 的 int32 形式（=10.41.10.100，可用 `Tools/convert_ip.py` 核算） |
| **UXRCE_DDS_DOM_ID** | **5** | **必须！与树莓派 ROS_DOMAIN_ID=5 一致；默认 0 不改则双方互相看不见（最大隐性坑）** |

> 注意：若以太网口当前被 TELEM MAVLink 配置占用，需先释放该口。

### 3.5 控制逻辑模块订阅示例

```cpp
#include <uORB/Subscription.hpp>
#include <uORB/topics/airship_bladder_pressure.h>

class AirshipBladderController : public ModuleBase<AirshipBladderController>
{
  // ...
private:
  uORB::Subscription _bladder_sub{ORB_ID(airship_bladder_pressure)};
  hrt_abstime _last_fresh_sample{0};   // 最近一次 valid=1 的 timestamp_sample

  void Run() final
  {
    airship_bladder_pressure_s bd{};
    if (_bladder_sub.updated() && _bladder_sub.copy(&bd)) {
      if (bd.valid[airship_bladder_pressure_s::BLADDER_LEFT_MAIN] == 1 &&
          bd.stale[airship_bladder_pressure_s::BLADDER_LEFT_MAIN] == 0) {
        const float dp_pa = bd.pressure_delta_pa[airship_bladder_pressure_s::BLADDER_LEFT_MAIN];
        _last_fresh_sample = bd.timestamp_sample;
        // ... 正常控制逻辑
      }
      // NaN 检查: std::isnan(dp_pa) -> 该槽从未有效
    }
    // 时间超时 failsafe (0.5Hz 低频, 最坏 ~10s 无真实更新):
    if (hrt_elapsed_time(&_last_fresh_sample) > 10_s) {
      // 停用气囊控制 / 进入安全态
    }
  }
};
```

### 3.6 飞控侧验证

QGC MAVLink Console（或调试串口 nsh）：

```
listener airship_bladder_pressure 5     # 连续 5 条, 应每 2s 一条
uorb top                                # 确认 airship_bladder_pressure ~0.5Hz
uxrce_dds_client status                 # 确认 client running / agent 已连接
```

---

## 4. 树莓派侧部署（已完成的部分 + Agent 安装）

### 4.1 工程侧（已交付）

| 组件 | 位置 | 说明 |
|------|------|------|
| px4_msgs 包 | `src/px4_msgs/` | 包名 `px4_msgs` 是 DDS 类型名匹配硬约束，**不可改名**；将来若需官方 PX4 消息必须合入本包，禁止引入官方 px4_msgs 仓库 |
| 桥接逻辑库 | `src/airship_fc/include/airship_fc/bladder_bridge_logic.hpp` | 纯逻辑（无 ROS 依赖），槽位映射+沿用语义 |
| 桥接节点 | `src/airship_fc/src/bladder_bridge_node.cpp` | 订阅 `/lora/samples`，发布 `/fmu/in/airship_bladder_pressure` |
| 单元测试 | `src/airship_fc/test/test_bladder_bridge_logic.cpp` | 14 用例全过（含去抖沿用轮不被拷贝的 press_valid=1 迷惑的关键语义用例） |
| 参数/编排 | `airship_bringup`（airship_params.yaml `bladder_bridge_node` 段 + launch） | 随 `airship-device-monitor` 服务自启 |

### 4.2 MicroXRCEAgent 安装（一次性，需手动执行）

`ros-jazzy-micro-xrce-dds-agent` apt 包**不存在**，须源码编译 **v2.4.3**
（v3.x 与 PX4 v1.17 client 不兼容）：

```bash
cd ~ && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. -DUAGENT_P2P_PROFILE=OFF     # Ubuntu 24.04 必须关 P2P profile, 否则编译报错
make -j1                              # 4核低配机, 强制单线程
sudo make install
sudo ldconfig /usr/local/lib/         # 必须! 否则运行期 BadParamException 崩溃
```

### 4.3 Agent systemd 服务

```ini
# /etc/systemd/system/airship-xrce-agent.service
[Unit]
Description=Lingyun Micro XRCE-DDS Agent (PX4 bridge)
After=network-online.target

[Service]
Type=simple
Restart=always
RestartSec=3
ExecStart=/usr/local/bin/MicroXRCEAgent udp4 -p 8888 -v 4

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now airship-xrce-agent
```

说明：Agent 是独立进程，不需要 source ROS、不需要设 ROS_DOMAIN_ID
（DDS domain 由 PX4 client 请求在 Agent 侧创建，见 3.4 的 UXRCE_DDS_DOM_ID）。

---

## 5. 端到端验证步骤（分层递进）

| 层 | 命令/操作 | 预期 |
|----|-----------|------|
| ① 桥节点输出（无飞控可验） | 树莓派：`unset RMW_IMPLEMENTATION CYCLONEDDS_URI; export ROS_DOMAIN_ID=5; source install/setup.bash; ros2 topic echo /fmu/in/airship_bladder_pressure` | 0.5Hz 输出 4 槽位数据（2026-09-03 台架实测：0/+58/+37/-41 Pa，全 valid=1） |
| ② Agent 连接 | 飞控启动 client 后：`journalctl -u airship-xrce-agent -f` | 出现 `create_participant` / session established（client_key）；有 `create_topic ... rt/fmu/in/airship_bladder_pressure` 即条目生效 |
| ③ PX4 uORB | 飞控 nsh/QGC Console：`listener airship_bladder_pressure 5` | 每 2s 一条，槽位值与 ① 一致，valid/stale 字段正确 |
| ④ 负压差 | 台架对传感器吹气/吸气 | 负值正确传递（历史教训：曾按无符号解析，-17Pa 被显示为 4294967279，已修复并有单测锁定） |

> 调试陷阱：交互 shell 默认是 CycloneDDS（`.bashrc` 导出 + `~/.ros/cyclonedds.xml` 静态 peer），
> 与生产环境（systemd 无 RMW 变量 → Fast DDS）不同族，直接 `ros2 topic echo` 会看不到话题。
> **调试命令前先 `unset RMW_IMPLEMENTATION CYCLONEDDS_URI`**。

---

## 6. 已知限制与注意事项

1. **0.5Hz 低频**：LoRa 2s 轮询周期是采集层物理上限。飞控消费逻辑必须做
   `valid/stale` 检查 + 时间超时 failsafe（见 2.2），不可用消息到达率当新鲜度。
2. **沿用值"假新鲜"**：LoRa 离线去抖（连续 3 轮失败才判离线）+ 串口重连期间，
   消息仍以 0.5Hz 发布沿用历史值——靠 `stale=1` 标记区分，单测已锁定该语义。
3. **Agent 单点**：Agent 进程崩溃则飞控收不到数据。已配 systemd Restart=always 自愈；
   PX4 侧超时 failsafe 兜底。
4. **传感器未实装**（截至 2026-09-03）：当前为台架数据，实装前 PX4 侧槽位值
   可能为噪声/零漂量级，联调时"全槽 NaN"（传感器全断）是正常初始态。
5. **与 MAVROS 共存**：uXRCE-DDS 走 UDP 8888，MAVROS 走 UDP 14550，链路独立无冲突。
6. **本工程 px4_msgs 是灵云定制包**（仅含 AirshipBladderPressure），将来若需官方
   PX4 消息（SensorCombined 等），必须把官方 msg 文件合入本包，**禁止 clone 官方
   px4_msgs 仓库**（同名包冲突）。

---

## 修订记录

| 日期 | 内容 |
|------|------|
| 2026-09-03 | 初版：树莓派端全链路交付（px4_msgs/桥接节点/单测/编排），PX4 侧配合步骤与验证方案 |
