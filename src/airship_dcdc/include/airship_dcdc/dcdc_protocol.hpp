// 灵云01号伴飞电脑 — DCDC 电源模块 CAN 协议解析
// 协议: 昊瑞昌 HRC-GFD360-48-4K (280-500V输入, 4KW, 输出48V), J1939 扩展帧
//
// 本模块为纯 C++ 协议库(无 ROS 依赖),便于 gtest 单元测试。
// 注意: 控制帧必须持续周期下发(200ms), 否则 DCDC 可能停机。
#ifndef AIRSHIP_DCDC__DCDC_PROTOCOL_HPP_
#define AIRSHIP_DCDC__DCDC_PROTOCOL_HPP_

#include <cstdint>

#include "airship_can/can_interface.hpp"

namespace airship_dcdc
{

// 报文帧 ID (29 位扩展帧)
constexpr uint32_t kPowerControlId = 0x18EF3010;  // VMS->DCDC 电源控制帧
constexpr uint32_t kStatusId = 0x18FF3247;        // DCDC->VMS 电源状态帧
constexpr uint32_t kAnalogQueryId = 0x18D80047;   // VMS->DCDC 模拟量查询
constexpr uint32_t kAnalogRespId = 0x18D84700;    // DCDC->VMS 模拟量回应

// 电源控制帧 Byte2 控制位: Bit5~4
constexpr uint8_t kControlOn = 0x10;   // 01 = 开机
constexpr uint8_t kControlOff = 0x00;  // 00 = 关机

// 状态帧故障字节位 (故障状态表)
namespace fault
{
constexpr uint8_t kInputUndervolt = 0x01;  // BIT0 输入欠压
constexpr uint8_t kInputOvervolt = 0x02;   // BIT1 输入过压
constexpr uint8_t kOutputOn = 0x04;        // BIT2 输出状态 (1=开机)
constexpr uint8_t kCanLost = 0x08;         // BIT3 CAN 中断
constexpr uint8_t kOvertemp = 0x10;        // BIT4 过热
constexpr uint8_t kShortCircuit = 0x20;    // BIT5 短路
constexpr uint8_t kFault = 0x80;           // BIT7 总故障
}  // namespace fault

// 解析结果聚合 (与 DcdcStatus 字段对应, 但无 ROS 依赖)
struct DcdcData
{
  float input_voltage = 0.0f;     // 输入电压 [V] (母线)
  float output_voltage = 0.0f;    // 输出电压 [V]
  float output_current = 0.0f;    // 输出电流 [A]
  float ambient_temp = 0.0f;      // 环境温度 [℃]
  float heatsink_temp = 0.0f;     // 散热器温度 [℃]
  uint8_t fault_word = 0;         // 故障状态字节
  bool output_enabled = false;    // 输出是否开启
};

// 构造电源控制帧: 开机(或关机) + 输出电压 + 限流
// set_voltage [V], set_current [A], 均按 0.1 分辨率编码
airship_can::CanFrame build_control_frame(bool enabled, float set_voltage, float set_current);

// 构造模拟量查询帧
airship_can::CanFrame build_analog_query_frame();

// 解析电源状态帧 (0x18FF3247), len 为 DLC
void parse_status(const uint8_t * data, uint32_t len, DcdcData & out);
// 解析模拟量回应帧 (0x18D84700)
void parse_analog(const uint8_t * data, uint32_t len, DcdcData & out);

}  // namespace airship_dcdc

#endif  // AIRSHIP_DCDC__DCDC_PROTOCOL_HPP_