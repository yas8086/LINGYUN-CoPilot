# 链路诊断报告 — MAVLink 遥测断流与 uXRCE-DDS 状态

| 项目 | 内容 |
|------|------|
| 诊断日期 | 2026-09-06 20:00 ~ 21:45 |
| 触发背景 | 用户反馈 QGC 地面站读不到气囊压差数据；QGC 主页"气囊浮力监控"面板四囊字段全部显示 "—" |
| 诊断范围 | 树莓派侧全链路（LoRa → bridge → Agent）+ 树莓派↔飞控两条通讯链路 |
| 诊断结论 | **机载计算机侧全部正常；两条链路的断点均在飞控侧**（详见 §4） |
| 关联文档 | 08_气囊压差uXRCE-DDS同步协议.md、09_PX4侧配合开发指南.md |

---

## 1. 诊断环境快照（诊断时点状态）

| 项 | 状态 | 备注 |
|----|------|------|
| 树莓派系统 | 当日 19:41 重启过 | 与飞控同期重启 |
| airship-device-monitor | active | bladder_bridge_node 随服务运行 |
| airship-xrce-agent | active (running) | UDP 8888 监听确认 |
| 飞控 10.41.10.2 | ping 通（0.17ms） | 网络层完好 |

## 2. 测试内容与结果

### 2.1 LoRa 集中器 → 树莓派数据（✅ 正常）

命令：
```bash
ros2 topic echo --once /fmu/in/airship_bladder_pressure
```
结果（2026-09-06 20:22 实测）：
```
pressure_delta_pa: [+37.0, +58.0, -40.0, -40.0]   # 左副囊/左主囊/右主囊/右副囊
temperature_c:     [22.1, 22.6, 21.5, 21.9]
valid: [1, 1, 1, 1]   stale: [0, 0, 0, 0]
```
- 频率稳定 0.5Hz（`ros2 topic hz` 实测 average rate: 0.500）
- 负压差传递正常，全槽 valid

### 2.2 uXRCE-DDS 树莓派侧（✅ 就绪，但 PX4 client 未连入）

| 检查项 | 命令 | 结果 |
|--------|------|------|
| Agent 进程 | `ps -eo args \| grep MicroXRCE` | `/usr/local/bin/MicroXRCEAgent udp4 -p 8888 -v 4` 运行中 |
| 监听端口 | `ss -ulnp \| grep 8888` | 0.0.0.0:8888 监听中 |
| PX4 client 连入 | `journalctl -u airship-xrce-agent` | **零 client 连入记录** |
| 飞控→树莓派方向话题 | `ros2 topic list \| grep /fmu/out` | **0 个**（client 未连入的必然结果） |

**重要发现**：本机 Agent 二进制（8月5日编译的 v2.4.3）运行时**零日志输出**（连启动
banner 都没有，日志特性编译时被裁剪）。实测验证：
```bash
timeout 6 /usr/local/bin/MicroXRCEAgent udp4 -p 18888 -v 4 > /tmp/agent_v4_test.log 2>&1
# 退出后 /tmp/agent_v4_test.log 为 0 字节
```
**影响**：不能用 journalctl 判断 client 是否连入。正确验证手段（已写入文档 08/09）：
```bash
sudo tcpdump -i eth0 'udp port 8888' -c 20
# client 连入后可见 10.41.10.2 ↔ 10.41.10.100:8888 双向小包
```

### 2.3 PX4 侧固件状态（部分完成，据 QGC Console 推断）

用户在 QGC MAVLink Console 执行：
```
listener airship_bladder_pressure 5
→ never published
```
**解读**：返回 "never published" 而非 "unknown topic"——
- ✅ msg + dds_topics.yaml 已编入固件（任务①②③完成）
- ❌ uORB 无发布者 = client 收不到数据 = **client 未连接**（任务④未完成）

### 2.4 MAVLink 遥测链路诊断（❌ 断流，逐层定位）

| 层 | 命令 | 结果 | 判定 |
|----|------|------|------|
| 网络层 | `ping -c 2 10.41.10.2` | 0% loss, 0.17ms | ✅ 通 |
| 进程层 | `ps` / `ss -ulnp` | mavros_node 存活，监听 0.0.0.0:14550 | ✅ 正常 |
| 握手层 | `ros2 topic echo /mavros/state` | **connected: false** | ❌ |
| 数据层 | `ros2 topic hz /mavros/imu/data`、`/mavros/battery` | **均无数据** | ❌ |
| **铁证** | `sudo tcpdump -i eth0 'host 10.41.10.2' -c 20` | **只有出、没有进** | ❌ |

抓包原始输出（21:42:24，抓 20 包全为出向心跳）：
```
21:42:24.652197 IP lingyun01.14550 > 10.41.10.2.14550: UDP, length 28   # mavros 心跳
21:42:24.752101 IP lingyun01.14550 > 10.41.10.2.14550: UDP, length 28
21:42:24.852430 IP lingyun01.14550 > 10.41.10.2.14550: UDP, length 28
... (20 包全部为 lingyun01 → 10.41.10.2，无一回包)
```

**MAVLink 工作机制说明**：mavros 每秒向 `10.41.10.2:14550` 发心跳；飞控的以太网
MAVLink 实例收到后，把遥测回发到收包源地址（10.41.10.100:14550）。现在飞控
**零回包** = 飞控侧 MAVLink 以太网实例没有工作。

**易误判点**：`/fc/status` 仍以 10Hz 发布——那是 fc_monitor_node 自己的心跳包，
字段为空值，**不代表遥测正常**。判断遥测要看 `/mavros/imu/data` 是否流动。

## 3. QGC "气囊浮力监控"面板显示 "—" 的原因

该面板是定制 QGC 的 MAVLink 消费组件（bal_p0~p3/blower/valve 字段）。当前双重原因：
1. MAVLink 链路断流（§2.4），面板无数据来源
2. 即使链路恢复，气囊压差目前走 uXRCE-DDS 进 PX4 uORB，**不在 MAVLink 遥测流里**，
   面板仍不会显示——需 PX4 侧追加"uORB → MAVLink 桥接"开发（文档 09 待补任务⑥，
   具体消息类型取决于面板订阅的 MAVLink 消息定义）

## 4. 结论

| # | 结论 | 依据 |
|---|------|------|
| 1 | 机载计算机（树莓派）侧全部组件工作正常 | §2.1/§2.2 各项检查通过 |
| 2 | 气囊压差数据正在产生并以 0.5Hz 发布 | §2.1 实测 |
| 3 | uXRCE-DDS 链路断点 = **PX4 侧 client 未启动** | §2.2/§2.3 互证 |
| 4 | MAVLink 遥测链路断点 = **飞控侧以太网 MAVLink 实例未工作** | §2.4 单向流量铁证 |
| 5 | QGC 面板显示 "—" = 链路①断流 + 压差未接入 MAVLink 流 | §3 |

两条链路在飞控侧恢复后，**树莓派侧无需任何改动**，数据流自动恢复。

## 5. 待办（均在飞控侧）

- [ ] 检查飞控以太网 MAVLink 实例参数（MAV_1_CFG/MAV_1_MODE/端口绑定），恢复遥测回发
- [ ] 启动 `uxrce_dds_client start -t udp -p 8888 -h 10.41.10.100`（**UXRCE_DDS_DOM_ID=5 必须设置**，文档 09 §5）
- [ ] 树莓派上 `sudo tcpdump -i eth0 'udp port 8888' -c 20` 确认 client 连入
- [ ] QGC Console `listener airship_bladder_pressure 5` 验证 uORB 数据
- [ ] （后续）确认 QGC 面板订阅的 MAVLink 消息类型 → PX4 侧 uORB→MAVLink 桥接（文档 09 任务⑥）

## 修订记录

| 日期 | 内容 |
|------|------|
| 2026-09-06 | 初版：MAVLink 断流分层诊断 + uXRCE-DDS 状态核查 + Agent 无日志发现 + QGC 面板原因分析 |
