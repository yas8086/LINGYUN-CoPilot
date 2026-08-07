// 灵云01号伴飞电脑 — MPPT 光伏控制器 CAN 协议解析
// 协议: YQPV_SPC/SMC 系列 MPPT-CAN通信协议 V1.2
//
// 本模块为纯 C++ 协议解析库(无 ROS 依赖),便于 gtest 单元测试。
// 帧 ID 结构(29位扩展帧):
//   [28:24] 0x14 (只读) / 0x13 (配置)
//   [23:16] 报文代码 code
//   [15:8]  目标地址, 固定 0xA1
//   [7:0]   源地址
// 只读查询: 主机发远程帧 0x14[code]A1[src], 从机回 0x14[code]A1[dev]
#ifndef AIRSHIP_MPPT__MPPT_PROTOCOL_HPP_
#define AIRSHIP_MPPT__MPPT_PROTOCOL_HPP_

#include <cstdint>

#include "airship_can/can_interface.hpp"

namespace airship_mppt
{

// ===== 帧 ID 结构 =====
// 只读数据帧类型高5位
constexpr uint32_t kReadType = 0x14U;
// 目标地址(协议固定)
constexpr uint8_t kTargetAddr = 0xA1;

// 只读地址段报文代码 (code)
enum ReadCode : uint8_t {
  kCodeRated = 0x02,        // 额定参数: 电池额定电压/充电额定电流
  kCodeRealtime = 0x03,     // 实时: 光伏电压/电池电压/充电电流
  kCodeState = 0x04,        // 充电状态/故障状态
  kCodeEnergyDay = 0x05,    // 日/月发电量
  kCodeEnergyTotal = 0x06,  // 总发电量
  kCodeTemp = 0x08,         // 运行时间/机内空气温度/模块温度
  kCodeControl = 0x0A,      // 设备控制模式/充电开关/充电限流
};

// 充电状态码 (充电状态表)
enum ChargeState : uint16_t {
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
enum ControlMode : uint16_t {
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

// 构造只读查询远程帧: 0x14[code]A1[src]
// src: 本机(主机)源地址, 默认 0x00
airship_can::CanFrame build_query_frame(uint8_t code, uint8_t src = 0x00);

// 解析 0x03 实时电压/电流帧
void parse_realtime(const uint8_t * data, MpptData & out);
// 解析 0x02 额定参数帧
void parse_rated(const uint8_t * data, MpptData & out);
// 解析 0x04 充电/故障状态帧
void parse_state(const uint8_t * data, MpptData & out);
// 解析 0x05 日/月发电量帧
void parse_energy_day(const uint8_t * data, MpptData & out);
// 解析 0x06 总发电量帧
void parse_energy_total(const uint8_t * data, MpptData & out);
// 解析 0x08 温度帧
void parse_temp(const uint8_t * data, MpptData & out);
// 解析 0x0A 控制模式/充电开关帧
void parse_control(const uint8_t * data, MpptData & out);

}  // namespace airship_mppt

#endif  // AIRSHIP_MPPT__MPPT_PROTOCOL_HPP_