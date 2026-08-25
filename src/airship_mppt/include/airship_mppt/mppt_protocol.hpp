// 灵云01号伴飞电脑 — MPPT 光伏控制器 CAN 协议解析
// 协议: YQPV_SPC/SMC 系列 MPPT-CAN通信协议 V1.2
//
// 本模块为纯 C++ 协议解析库(无 ROS 依赖),便于 gtest 单元测试。
// 帧 ID 结构(29位扩展帧), 主机查询帧与从机回应帧的位域排布【不对称】
// (勘误版协议 3.2 节, 2026-08 实测验证: 按对称排布发送从机不响应):
//   主机查询帧: 0x14 [code] [设备地址] 0xA1   —— 设备地址在前(如 140301A1 查 01 号机)
//   从机回应帧: 0x14 [code] 0xA1 [设备地址]   —— A1 在前(如 1403A101 为 01 号机回应)
// 其中 [28:24] 为类型(0x14 只读 / 0x13 配置), [23:16] 为报文代码 code,
// 其余两字节依帧方向分别为 0xA1 协议标志与目标/源设备地址。
#ifndef AIRSHIP_MPPT__MPPT_PROTOCOL_HPP_
#define AIRSHIP_MPPT__MPPT_PROTOCOL_HPP_

#include <cstdint>
#include <optional>

#include "airship_can/can_interface.hpp"

namespace airship_mppt
{

// ===== 帧 ID 结构 =====
// 只读数据帧类型高5位
constexpr uint32_t kReadType = 0x14U;
// 目标地址(协议固定)
constexpr uint8_t kTargetAddr = 0xA1;

// 只读地址段报文代码 (code)
enum ReadCode : uint8_t
{
  kCodeRated = 0x02,        // 额定参数: 电池额定电压/充电额定电流
  kCodeRealtime = 0x03,     // 实时: 光伏电压/电池电压/充电电流
  kCodeState = 0x04,        // 充电状态/故障状态
  kCodeEnergyDay = 0x05,    // 日/月发电量
  kCodeEnergyTotal = 0x06,  // 总发电量
  kCodeTemp = 0x08,         // 运行时间/机内空气温度/模块温度
  kCodeControl = 0x0A,      // 设备控制模式/充电开关/充电限流
};

// 充电状态码 (充电状态表)
enum ChargeState : uint16_t
{
  kChargeBoot = 0,      // 启动阶段 (开机软起动)
  kChargeBoost = 1,     // 快充阶段 (SPC: PWM, SMC: MPPT/恒流)
  kChargeEqualize = 2,  // 均充阶段 (定时恒压)
  kChargeFloat = 3,     // 浮充阶段 (恒压, 锂电池系统无此状态)
  kChargeDone = 4,      // 结束充电
};

// 故障状态寄存器位 (故障状态表)
namespace fault
{
constexpr uint16_t kBatteryUndervolt = 0x0001;  // BIT0 电池欠压
constexpr uint16_t kBatteryOvervolt = 0x0002;   // BIT1 电池过压
constexpr uint16_t kPvUndervolt = 0x0004;       // BIT2 光伏欠压
constexpr uint16_t kPvOvervolt = 0x0008;        // BIT3 光伏过压
constexpr uint16_t kOvercurrent = 0x0010;       // BIT4 充电过流
constexpr uint16_t kOvertemp = 0x0020;          // BIT5 设备过温
constexpr uint16_t kModuleDamage = 0x0040;      // BIT6 模块损坏
}  // namespace fault

// 设备控制模式码 (设备控制模式表)
enum ControlMode : uint16_t
{
  kModeStandalone = 0,     // 独立运行(默认, 开机自动充电)
  kModeEmsRs485 = 1,       // EMS控制-RS485 (开机不充电)
  kModeEmsCan = 2,         // EMS控制-CAN (开机不充电)
  kModeFactoryReset = 255, // 恢复出厂设置
};

// 充电开关状态值 (FF00H 开 / 0000H 关)
constexpr uint16_t kChargeSwitchOn = 0xFF00;

// ===== 解析结果聚合 (与 MpptStatus 字段对应, 但无 ROS 依赖) =====
struct MpptData
{
  // 实时 (0x03)
  float pv_voltage = 0.0f;        // 光伏电压 [V]
  float battery_voltage = 0.0f;   // 电池电压 [V]
  float charge_current = 0.0f;    // 充电电流 [A]
  // 额定 (0x02)
  float rated_voltage = 0.0f;     // 电池额定电压 [V]
  float rated_current = 0.0f;     // 充电额定电流 [A]
  // 状态 (0x04)
  uint16_t charge_state = 0;      // 充电状态码
  uint16_t fault_state = 0;       // 故障状态位
  // 发电量 (0x05/0x06)
  float energy_today = 0.0f;      // 日发电量 [kWh]
  float energy_month = 0.0f;      // 月发电量 [kWh]
  float energy_total = 0.0f;      // 总发电量 [kWh]
  // 温度 (0x08)
  float air_temp = 0.0f;          // 机内空气温度 [℃]
  float module_temp = 0.0f;       // 模块温度 [℃]
  // 控制 (0x0A)
  uint16_t control_mode = 0;      // 设备控制模式
  bool charging_enabled = false;  // 充电开关是否开启
};

// 构造只读查询帧: 0x14[code][device_addr]A1 (设备地址在前, 与回应帧排布相反)
// device_addr: 目标从机(MPPT)的设备地址——双机部署时主=0x01/副=0x02,
//   勿用 0x00 广播地址发送(实测广播可能不应答, 故不提供默认值)。
// 注: 按"8 字节数据帧(数据全 0)"发送查询(实测可触发响应, 对 RTR 不敏感)。
airship_can::CanFrame build_query_frame(uint8_t code, uint8_t device_addr);

// 解析从机回应帧 ID(0x14[code]A1[dev])为只读段 code。
// 纯函数, 供接收线程作"帧 ID 匹配"判定——返回 nullopt 表示非本设备/非法帧,
// 应被忽略(如: type 非 0x14、target 非 0xA1、src 设备地址不匹配、未知 code)。
// 提炼自 mppt_node::receive_loop, 使历史 BUG 集中的帧匹配逻辑可单元测试。
std::optional<ReadCode> match_response_id(uint32_t frame_id, uint8_t device_addr);

// 解析 0x03 实时电压/电流帧
void parse_realtime(const uint8_t * data, uint32_t len, MpptData & out);
// 解析 0x02 额定参数帧
void parse_rated(const uint8_t * data, uint32_t len, MpptData & out);
// 解析 0x04 充电/故障状态帧
void parse_state(const uint8_t * data, uint32_t len, MpptData & out);
// 解析 0x05 日/月发电量帧
void parse_energy_day(const uint8_t * data, uint32_t len, MpptData & out);
// 解析 0x06 总发电量帧
void parse_energy_total(const uint8_t * data, uint32_t len, MpptData & out);
// 解析 0x08 温度帧
void parse_temp(const uint8_t * data, uint32_t len, MpptData & out);
// 解析 0x0A 控制模式/充电开关帧
void parse_control(const uint8_t * data, uint32_t len, MpptData & out);

}  // namespace airship_mppt

#endif  // AIRSHIP_MPPT__MPPT_PROTOCOL_HPP_
