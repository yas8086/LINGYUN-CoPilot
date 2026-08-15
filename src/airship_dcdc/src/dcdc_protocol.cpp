// 灵云01号伴飞电脑 — DCDC 电源模块 CAN 协议解析实现
#include "airship_dcdc/dcdc_protocol.hpp"

#include <cstdint>

#include "airship_utils/can_utils.hpp"
#include "airship_utils/math_utils.hpp"

namespace airship_dcdc
{

using airship_can::CanFrame;
using airship_utils::get_u16_le;
using airship_utils::scale_u16;
using airship_utils::temp_with_offset;

CanFrame build_control_frame(bool enabled, float set_voltage, float set_current)
{
  CanFrame frame{};
  frame.id = kPowerControlId;
  frame.extended = true;
  frame.len = 8;
  // Byte2: Bit5~4 = 01(开机) or 00(关机)
  frame.data[2] = enabled ? kControlOn : kControlOff;
  // Byte3~4: 输出电压 (0.1V/BIT, 小端), 钳制到 [0,600]V 防负值/超范围转 uint16 UB
  const uint16_t v = static_cast<uint16_t>(
    airship_utils::clampf(set_voltage, 0.0f, 600.0f) * 10.0f);
  frame.data[3] = static_cast<uint8_t>(v & 0xFF);
  frame.data[4] = static_cast<uint8_t>((v >> 8) & 0xFF);
  // Byte5~6: 输出限流 (0.1A/BIT, 小端), 钳制到 [0,100]A 防负值/超范围转 uint16 UB
  const uint16_t i = static_cast<uint16_t>(
    airship_utils::clampf(set_current, 0.0f, 100.0f) * 10.0f);
  frame.data[5] = static_cast<uint8_t>(i & 0xFF);
  frame.data[6] = static_cast<uint8_t>((i >> 8) & 0xFF);
  return frame;
}

CanFrame build_analog_query_frame()
{
  CanFrame frame{};
  frame.id = kAnalogQueryId;
  frame.extended = true;
  frame.len = 8;
  return frame;
}

// 电源状态帧: Byte0 故障字节, Bit2=输出状态;
// 状态帧同时携带模拟量(与模拟量回应同格式, 实测 DCDC 不回应模拟量查询 0x18D84700,
// 故电压/电流/温度均需由此帧解析):
//   Byte1~2 输入电压(1V/BIT, 小端), Byte3~4 输出电压(0.1V/BIT),
//   Byte5~6 输出电流(0.1A/BIT), Byte7 温度(1℃/BIT, -40 偏移)
void parse_status(const uint8_t * data, uint32_t len, DcdcData & out)
{
  if (len < 1) {
    return;
  }
  out.fault_word = data[0];
  out.output_enabled = (data[0] & fault::kOutputOn) != 0;
  if (len >= 8) {
    out.input_voltage = scale_u16(get_u16_le(data, 1), 1.0f);
    out.output_voltage = scale_u16(get_u16_le(data, 3), 0.1f);
    out.output_current = scale_u16(get_u16_le(data, 5), 0.1f);
    out.heatsink_temp = temp_with_offset(static_cast<int8_t>(data[7]));
  }
}

// 模拟量回应帧: 输入电压(1V)/输出电压(0.1V)/输出电流(0.1A)/温度(1℃,-40)
void parse_analog(const uint8_t * data, uint32_t len, DcdcData & out)
{
  if (len < 8) {
    return;
  }
  out.input_voltage = scale_u16(get_u16_le(data, 0), 1.0f);
  out.output_voltage = scale_u16(get_u16_le(data, 2), 0.1f);
  out.output_current = scale_u16(get_u16_le(data, 4), 0.1f);
  out.ambient_temp = temp_with_offset(static_cast<int8_t>(data[6]));
  out.heatsink_temp = temp_with_offset(static_cast<int8_t>(data[7]));
}

}  // namespace airship_dcdc
