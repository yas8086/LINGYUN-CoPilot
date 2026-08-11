// 灵云01号伴飞电脑 — 锂电池 BMS CAN 协议解析实现
#include "airship_bms/bms_protocol.hpp"

#include <cstdint>

#include "airship_utils/can_utils.hpp"

namespace airship_bms
{

using airship_utils::get_u16_le;

// BattInfo02: 总压(0.1V)/总电流(0.1A,-100A偏移)/SOC(0.1%)/RealSoc(0.1%)
void parse_batt_info(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  out.pack_voltage = airship_utils::scale_u16(get_u16_le(data, 0), 0.1f);
  out.pack_current = airship_utils::scale_u16(get_u16_le(data, 2), 0.1f) - 100.0f;
  out.soc = airship_utils::scale_u16(get_u16_le(data, 4), 0.1f);
  out.real_soc = airship_utils::scale_u16(get_u16_le(data, 6), 0.1f);
}

// BattInfo01: 运行状态(低4位)/连接状态(高4位)/绝缘电阻(kΩ)/报警级别
void parse_batt_status(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 7) {
    return;
  }
  out.run_state = data[0] & 0x0F;
  out.connection_status = (data[0] >> 4) & 0x0F;
  out.positive_insulation_kohm = get_u16_le(data, 2);
  out.negative_insulation_kohm = get_u16_le(data, 4);
  out.alarm_level = data[6];
}

// CellVoltage_XX: 每帧5节, 12位信号 (0.001V, +1V 偏移)
// 12位 Motorola(big-endian) 布局(由 DBC start 位推导, 已用 cantools 实测校验):
//   节0: b0<<4 | (b1>>4)    节1: (b1&0x0F)<<8 | b2
//   节2: b3<<4 | (b4>>4)    节3: (b4&0x0F)<<8 | b5
//   节4: (b6&0x0F)<<8 | b7
void parse_cell_voltage(uint32_t frame_id, const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  const uint32_t frame_idx = (frame_id - kCellVoltageBase) / 0x10;
  const uint32_t cell_start = frame_idx * kCellPerVoltFrame;

  const uint16_t v[5] = {
    static_cast<uint16_t>(((data[0] << 4) & 0xFF0) | (data[1] >> 4)),
    static_cast<uint16_t>(((data[1] & 0x0F) << 8) | data[2]),
    static_cast<uint16_t>(((data[3] << 4) & 0xFF0) | (data[4] >> 4)),
    static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]),
    static_cast<uint16_t>(((data[6] & 0x0F) << 8) | data[7]),
  };
  for (uint32_t i = 0; i < kCellPerVoltFrame; ++i) {
    const uint32_t idx = cell_start + i;
    if (idx >= kMaxCells) {
      break;
    }
    out.cell_voltages[idx] = static_cast<float>(v[i]) * 0.001f + 1.0f;
  }
}

// CellTempStatistic: 最高/最低/平均温度(1℃, -50偏移) 与温差
void parse_cell_temp_statistic(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  out.max_cell_temp = static_cast<float>(data[0]) - 50.0f;
  out.min_cell_temp = static_cast<float>(data[3]) - 50.0f;
  out.avg_cell_temp = static_cast<float>(data[6]) - 50.0f;
  out.temp_diff = static_cast<float>(data[7]) - 50.0f;
}

// PACKTemp: 极耳温度 8 点 (1℃, -50偏移)
void parse_pack_temp(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < kMaxPoleTemps) {
    return;
  }
  for (uint32_t i = 0; i < kMaxPoleTemps; ++i) {
    out.pole_temps[i] = static_cast<float>(data[i]) - 50.0f;
  }
}

}  // namespace airship_bms