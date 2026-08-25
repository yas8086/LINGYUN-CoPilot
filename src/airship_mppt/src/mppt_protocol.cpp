// 灵云01号伴飞电脑 — MPPT 光伏控制器 CAN 协议解析实现
#include "airship_mppt/mppt_protocol.hpp"

#include "airship_utils/can_utils.hpp"

namespace airship_mppt
{

using airship_can::CanFrame;
using airship_utils::get_i16_le;
using airship_utils::get_u16_le;
using airship_utils::get_u32_le;

CanFrame build_query_frame(uint8_t code, uint8_t device_addr)
{
  CanFrame frame{};
  // 主机查询帧排布: 0x14 | code | 设备地址 | 0xA1 (设备地址在 A1 之前)
  // 注: 与从机回应帧 0x14|code|A1|设备地址 (A1 在前) 排布相反, 二者不对称,
  //     说明书"主机发送 0x1401xxA1, 从机回应 0x1401A1xx" 即此含义(勘误版 3.2 节)。
  frame.id = (kReadType << 24) | (static_cast<uint32_t>(code) << 16) |
    (static_cast<uint32_t>(device_addr) << 8) | kTargetAddr;
  frame.extended = true;
  frame.len = 8;
  return frame;
}

std::optional<ReadCode> match_response_id(uint32_t frame_id, uint8_t device_addr)
{
  // 从机回应帧位域: [28:24] type=0x14(只读), [23:16] code, [15:8] A1, [7:0] 设备地址
  const uint8_t type = static_cast<uint8_t>((frame_id >> 24) & 0xFF);
  const uint8_t target = static_cast<uint8_t>((frame_id >> 8) & 0xFF);
  if (type != kReadType || target != kTargetAddr) {
    return std::nullopt;
  }
  // 源地址(回应帧末尾 8 位)须与本设备地址一致, 防多设备总线串扰误解析
  const uint8_t src = static_cast<uint8_t>(frame_id & 0xFF);
  if (src != device_addr) {
    return std::nullopt;
  }
  const uint8_t code = static_cast<uint8_t>((frame_id >> 16) & 0xFF);
  switch (static_cast<ReadCode>(code)) {
    case kCodeRated:
    case kCodeRealtime:
    case kCodeState:
    case kCodeEnergyDay:
    case kCodeEnergyTotal:
    case kCodeTemp:
    case kCodeControl:
      return static_cast<ReadCode>(code);
    default:
      return std::nullopt;  // 未知/保留 code, 忽略
  }
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
