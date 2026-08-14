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
├── airship_msgs/        # 设备监控消息定义 (BmsStatus/MpptStatus/DcdcStatus/BackupBmsStatus/DeviceAlert)
├── airship_can/         # SocketCAN 抽象层 (收发帧)
├── airship_utils/       # 可测工具库 (CAN解析/单位换算/限幅, 含 gtest 单测)
├── airship_bms/         # 主锂电池 BMS 驱动节点 (CAN)
├── airship_backup_bms/  # 12S 备用电源 BMS 驱动节点 (串口, 北辰协议 V1.4)
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
ros2 topic echo /backup_bms/status
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
| [高价值字段补齐报告](docs/lingyun01/0字段补齐报告.md) | 字段补齐详情、CSV 对照表核对结果、待补下传字段清单 |
| 厂家协议原文 | [docs/lingyun01/](docs/lingyun01/) 各设备厂家协议文件 (含 [12S备用电源](docs/lingyun01/BMS/12S备用电源/)) |

## 工程规范

- **接口先行**：消息定义集中在 `airship_msgs`，避免循环依赖
- **逻辑与运输层分离**：核心业务逻辑抽为纯库（`airship_utils`）便于 gtest 单测
- **依赖单向 DAG**：自下而上 `airship_msgs → airship_can/utils → 驱动 → monitor/link → bringup`
- **lint 检查**：clang-format / cpplint / flake8，见 [.github/workflows/ci.yml](.github/workflows/ci.yml)
- **一键启动**：[tools/start_device_monitor.sh](tools/start_device_monitor.sh)

## 开发阶段

- **Phase-1（当前）**：设备监控 BMS/MPPT/DCDC，暂不接入飞控
- **Phase-2（规划）**：接入飞控与飞行控制

## 变更与优化记录

### 高价值字段补齐 (2026-08-13)
- **BMS 故障字 ErrorCode(0x001FE0)**：解析 64 位故障/告警字 -> `fault_word1/2/3`；`BmsStatus.msg` `fault_word1` 由 uint16 修正为 uint32(原会截断 bit16-31 一级报警)；monitor 告警 code 显式 cast 低 16 位；JSON 下传 `fault1/fault2/fault3`
- **BMS SOH(0x005FF0)**：soh 健康状态补 JSON 下传(容量/循环次数/额定电压留在协议层, 见报告)
- **FC 航向角**：`/mavros/vfr_hud.heading` 采集 -> `FlightStatus.heading_deg`, JSON 下传 `hdg`
- **FC 真空速**：以 `VfrHud.airspeed` 填充 `true_airspeed`(安装版 mavros_msgs 无 Airspeed.msg, 暂未分离 IAS/TAS), JSON 下传 `tas/airspd/gs/climb/thr`
- **单测**：bms_protocol 新增 ParseErrorCode/ParseSoh/ParseSop/ParseCellVoltStatistic/ParsePoleTempStatistic + 短帧用例; json_packer 新增 BMS 故障字/SOH 与 FC 新字段断言
- **CSV 字段对照表核对**：`docs/lingyun01/0字段解析详情/` 下 6 个 CSV 逐行对照代码修正(01/02/03/04/06), 详见 [0字段补齐报告](docs/lingyun01/0字段补齐报告.md)

### 全面代码审查修复·低优先级 (2026-08-11)
- **MPPT 查询帧注释澄清**：`build_query_frame` 当前按 8 字节数据帧发送, 修正头文件注释不再称"远程帧", 并标注现场如无响应需按设备改为 RTR(见 docs/02)
- **backup_bms 掉线不清零**：维护 `last_good_data_`, 失败轮次沿用上一帧有效数值仅置 `online=false`, 避免发布清零值被误判为真实测量 0
- **BMS 重连清零旧数据**：断连恢复时 `bms_data_` 整体清零, 避免重连后首帧未到期间沿用陈旧值
- **fc_monitor 异常检测移入锁内**：`run_anomaly_detection` 在锁内执行, 消除 MultiThreadedExecutor 下共享状态读-写数据竞争
- **backup_bms 枚举分派**：`query` 第三参由魔法数字 1/2/3 改为 `PollKind` 枚举, 消除可读性/写错隐患
- **fc_monitor CSV 时间戳补零**：纳秒不足 9 位补零, 保证时间列宽一致便于解析
- **lora 掉线日志节流 + summary 刷新**：RX 超时/读错误改 `WARN_THROTTLE`; 串口掉线时 summary 的 `online_count` 归零(不再停留在断线前旧值)
- **dcdc_node 控制值单一来源**：上报的 set_voltage/set_current 优先取 `DCDC_SET_VOLTAGE/CURRENT` 环境变量(与 dcdc_hold 一致), 避免监控值与实际下发值不一致
- **协议边界单测确认**：bms/mppt/dcdc 已具 `ShortFrameIgnored` 用例覆盖各解析函数短帧安全返回, 无需新增

### 全面代码审查修复 (2026-08-11)
- **link_node 串口掉线自愈修复**：写失败时主动 close 串口, 使 `is_open()==false` 落入重连分支, 解决数传/USB 物理拔插后链路永不恢复的缺陷
- **CAN 节点 online 掉线兜底**：bms/mppt/dcdc 三节点新增 `link_timeout_s` 无数据超时, 超时周期兜底发布 `online=false`, 让下游感知设备失联(此前 online 恒为 true 且断帧后静默)
- **遥测精度修复**：`fmt_float` 由 `%.3g`(3 位有效数字, 大值截断) 改为 `%.4g`, 修复 pack_v=512.6→513 等精度丢失
- **串口健壮性**：`serial_interface` 设 `CLOCAL|CREAD`(防 USB 串口 hangup/接收失效); `write` 处理 `EAGAIN`(非阻塞缓冲满时 poll 等待重试)
- **频率除零防护补齐**：monitor/fc_monitor 频率参数 `<=0` 时回退默认值, 与其他节点一致, 消除 `static_cast<int>(inf)` UB
- **safety 初始化补齐**：`last_backup_time_` 构造时初始化, 消除未初始化成员隐患
- **socketcan 并发优化**：`receive` 的 poll 移出全局锁(fd 快照+锁内校验), 避免接收线程阻塞 send 造成发送抖动
- **math_utils 归一化优化**：`normalize_angle` 改用 `std::remainder`, O(1) 且对任意大角正确, 与 `angle_diff` 一致

### 12S 备用电源集成 (2026-08-11)
- **新增 airship_backup_bms 包**：12S 备用电源 BMS 驱动，经串口按北辰协议 V1.4 (9600 8N1) 轮询
  `0x06 基本信息1 / 0x07 单节温度 / 0x08 单节电压`，解析总压/总电流/SOC/SOH/电压温度统计/告警/保护/故障/系统状态与逐节电压温度
- **协议纯库可测**：`backup_bms_protocol`(CRC16/读请求构建/响应帧校验/多指令解析) 无 ROS 依赖，gtest 单测覆盖文档示例帧
- **驱动健壮性**：串口掉线自动重连、连续多轮失败判定掉线、响应帧起始/地址/主机/指令/长度/CRC 全量校验、单字节起始同步
- **消息**：`airship_msgs/BackupBmsStatus`；`DeviceAlert` 新增 `DEVICE_BACKUP_BMS=5`；`SafetyStatus` 新增 `backup_battery_ok`
- **系统集成**：接入 monitor 看门狗与异常告警、link JSON 下传(`backup` 字段)、cloud 上云、**safety 安全仲裁**(`backup_battery_judge`: 在线+总压≥`backup_bms_min_voltage`+`fault_word==0`, 参数 `backup_bms_timeout_s`/`backup_bms_min_voltage`)、launch/参数、udev 符号链接占位

### 第一优先（严重/崩溃/安全修复）
- **CAN send 缓冲区越界**：`socketcan.cpp` send 前钳制 DLC 到 8 字节，防止 `memcpy` 越界
- **JSON 精度丢失**：`json_packer` 新增 `fmt_double`（`%.6f`）用于经纬度/压力等字段，替代 `%.3g` 3 位有效数字
- **MQTT 数据竞争**：`mqtt_client` 的 `connected_` 改为 `std::atomic<bool>`
- **飞控启动 EMERGENCY 误报**：`fc_monitor_node` 异常检测改为 `mavros_connected_ && has_valid_data_` 门控，新增电池数据标志跳过未初始化判据
- **频率参数除零 UB**：mppt/dcdc/link/cloud 四节点统一校验 `rate<=0` 回退默认值
- **Dashboard DOM XSS**：`index.html` LoRa 面板改用 `textContent` 渲染，不再用 `innerHTML` 拼接

### 中级（优化/规范/边界）
- **协议帧长度校验**：bms/mppt/dcdc 各 parse 函数新增 `len` 参数，短帧安全返回
- **MPPT 源地址校验**：回应帧校验源设备地址，防多设备总线串扰
- **LoRa node_id 边界校验**：寄存器地址用 uint32 计算并校验不溢出 uint16，越界节点跳过
- **JSON 健壮性**：非有限值(NaN/Inf)输出 `null`；`flight_mode` 字符串转义
- **angle_diff 用 std::remainder**：O(1) 且对任意大角正确
- **BmsStatus 可变长数组**：`cell_voltages/cell_temps` 只携带实际电芯数，单帧从 2KB 降到 ~400B
- **MpptStatus.can_id 改 uint32**：与 BMS/DCDC 一致，避免 29 位扩展帧 ID 溢出
- **link_node 写失败日志**：串口写入失败节流告警
- **dcdc_hold 加固**：`strtod/strtol` 校验非法环境变量回退默认；send 返回值检查
- **MQTT 密码防暴露**：`cloud_node` 支持 `MQTT_PASSWORD` 环境变量，脚本不再经命令行传密码
- **install_production.sh 参数 bug**：`--can 250000` 现可正确消费波特率
- **airship_bringup package.xml**：补充 `airship_safety/fc/cloud/mavros` 的 exec_depend

### 高级（健壮性/一致性修复）
- **BMS cell_count 上限防护**：`bms_node` 配置电芯数超过协议上限(256)时截断，防 `cell_voltages` 数组越界
- **can_utils 边界契约**：`can_utils.hpp` 明确前置条件（调用方须按 DLC 校验长度），协议库已在 parse 入口保证
- **DCDC 控制帧钳制**：`build_control_frame` 对电压/电流用 `clampf` 钳制，防负值/超范围转 `uint16` 未定义行为
- **Modbus 从机地址校验**：`parse_temp/pressure_response` 校验响应从机地址，防 485 总线串扰/残留帧误解析
- **串口 write 部分写入/EINTR**：`SerialInterface::write` 改为循环写，处理部分写入与信号打断，确保整帧写完成
- **MQTT 明文密码去除**：`airship_params.yaml` 密码置空，改由 `MQTT_PASSWORD` 环境变量经 systemd `EnvironmentFile`(root 600) 注入；`install_production.sh` 新增 `install_secrets` 生成 env 文件
- **safety 接入 BMS 判据**：`safety_logic` 新增 `battery_judge`(在线+总压下限+无告警)，`safety_node` 订阅 BMS 并接入聚合，BMS 异常/掉线即置 `safe_to_control=false`
- **monitor DCDC 告警去重+解除**：DCDC 故障按 safety 一致掩码(排除 Bit2 输出位)状态机触发，跳变时告警/恢复时解除，不再刷屏
- **quat_to_euler 归一化**：入口先归一化四元数并处理零范数回退，避免非单位四元数导致姿态失真
- **throttle 单位修正**：`FlightStatus.msg` 油门注释由 0~1 修正为 %(MAVROS VfrHud.throttle 0~100)
- **高度坐标系现场标注**：`fc_monitor` 高度取反逻辑加现场验证注释（需按实际 PX4 输出确认符号），见 docs/05
- **dcdc_node 消除与 dcdc_hold 双发**：`dcdc_node` 改为仅监控（接收+发布），控制帧/查询帧保活完全交由独立 `dcdc_hold`，职责清晰
- **fc 告警阈值**：`publish_alert` 按实际触发等级填阈值，替代恒填 warning 阈值
- **CSV 磁盘 IO 优化**：`fc_monitor` CSV 每帧 flush 改为节流 ~1Hz，降低高频磁盘写入
- **airship_can package.xml**：移除多余 `rclcpp` 依赖，补 `ament_cmake_gtest` test_depend
- **Dashboard 安全提示**：`index.html` 注明默认公共 broker 仅限联调，生产须用私有+鉴权 broker

> 测试覆盖：`colcon test` 共 **1192 个用例，0 失败**（含本次新增 BMS 故障字/SOH/SOP/统计类解析与 JSON 下传用例）。