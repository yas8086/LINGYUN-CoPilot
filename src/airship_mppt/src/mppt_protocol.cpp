// 灵云01号伴飞电脑 — MPPT 光伏控制器 CAN 协议解析实现
#include "airship_mppt/mppt_protocol.hpp"

#include "airship_utils/can_utils.hpp"

namespace airship_mppt
{

using airship_can::CanFrame;
using airship_utils::get_i16_le;
using airship_utils::get_u16_le;
using airship_utils::get_u32_le;

CanFrame build_query_frame(uint8_t code, uint8_t src)
{
  CanFrame frame{};
  frame.id = (kReadType << 24) | (static_cast<uint32_t>(code) << 16) |
    (static_cast<uint32_t>(kTargetAddr) << 8) | src;
  frame.extended = true;
  frame.len = 8;
  return frame;
}

// 0x03: 光伏电压(0.1V) / 电池电压(0.1V) / 充电电流(0.1A) / 保留
void parse_realtime(const uint8_t * data, uint32_t len, MpptData & out)
{
  if (len < 6) {
    return;
  }
  out.pv_voltage = airship_utils::scale_u16(get_u16_le(data, 0), 0.1f);
  out.battery_voltage = airship_utils::scale_u16(get_u16_le(data, 2), 0.1f);
  out.charge_current = airship_utils::scale_u16(get_u16_le(data, 4), 0.1f);
}

// 0x02: 光伏额定电压 / 电池额定电压(0.1V) / 充电额定电流(A) / 保留
void parse_rated(const uint8_t * data, uint32_t len, MpptData & out)
{
  if (len < 6) {
    return;
  }
  out.rated_voltage = airship_utils::scale_u16(get_u16_le(data, 2), 0.1f);
  out.rated_current = airship_utils::scale_u16(get_u16_le(data, 4), 1.0f);
}

// 0x04: 充电状态 / 保留 / 故障状态 / 保留
void parse_state(const uint8_t * data, uint32_t len, MpptData & out)
{
  if (len < 6) {
    return;
  }
  out.charge_state = get_u16_le(data, 0);
  out.fault_state = get_u16_le(data, 4);
}

// 0x05: 日发电量(U32, 0.1kWh) / 月发电量(U32, 0.1kWh)
void parse_energy_day(const uint8_t * data, uint32_t len, MpptData & out)
{
  if (len < 8) {
    return;
  }
  out.energy_today = static_cast<float>(get_u32_le(data, 0)) * 0.1f;
  out.energy_month = static_cast<float>(get_u32_le(data, 4)) * 0.1f;
}

// 0x06: 总发电量(U32, 0.1kWh) / 保留
void parse_energy_total(const uint8_t * data, uint32_t len, MpptData & out)
{
  if (len < 4) {
    return;
  }
  out.energy_total = static_cast<float>(get_u32_le(data, 0)) * 0.1f;
}

// 0x08: 运行时间 / 机内空气温度(S16, 0.1℃) / 模块温度(S16, 0.1℃) / 保留
void parse_temp(const uint8_t * data, uint32_t len, MpptData & out)
{
  if (len < 6) {
    return;
  }
  out.air_temp = airship_utils::scale_i16(get_i16_le(data, 2), 0.1f);
  out.module_temp = airship_utils::scale_i16(get_i16_le(data, 4), 0.1f);
}

// 0x0A: 保留 / 设备控制模式 / 充电开关 / 充电限流
void parse_control(const uint8_t * data, uint32_t len, MpptData & out)
{
  if (len < 6) {
    return;
  }
  out.control_mode = get_u16_le(data, 2);
  const uint16_t charge_switch = get_u16_le(data, 4);
  out.charging_enabled = (charge_switch == kChargeSwitchOn);
}

}  // namespace airship_mppt