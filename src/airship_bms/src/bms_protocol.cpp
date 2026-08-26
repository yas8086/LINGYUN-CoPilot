// 灵云01号伴飞电脑 — 锂电池 BMS CAN 协议解析实现
#include "airship_bms/bms_protocol.hpp"

#include <cstdint>

#include "airship_utils/can_utils.hpp"

namespace airship_bms
{

using airship_utils::get_u16_le;

namespace
{
// BattInfo02 的 16 位信号为 Motorola(大端)布局: 高字节在前。
// (与 parse_cell_voltage 的 12-bit Motorola 同源; 曾误用 get_u16_le 小端导致
//  总压/SOC 解析 ×1.53 错误, 2026-08-26 实机对照 PDU 面板 386V/43% 确认大端。)
uint16_t u16_be(const uint8_t * data, size_t offset)
{
  return static_cast<uint16_t>((data[offset] << 8) | data[offset + 1]);
}
}  // namespace

// BattInfo02: 总压(0.1V)/总电流(0.1A,-1000A偏移)/SOC(0.1%)/RealSoc(0.1%) — 大端
// (DBC L474: BattCurr : 23|16@0+ (0.1,-1000) [-1000|1000] "A" — 偏移为 -1000 非 -100;
//  实机静置帧 0F 16 27 10 01 AC 01 8A: raw=0x2710=10000, 10000×0.1-1000=0A ✓;
//  误用 -100 会得到 900A — 正是 2026-08-26 地面站显示的 900A bug 根因)
void parse_batt_info(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  out.pack_voltage = airship_utils::scale_u16(u16_be(data, 0), 0.1f);
  out.pack_current = airship_utils::scale_u16(u16_be(data, 2), 0.1f) - 1000.0f;
  out.soc = airship_utils::scale_u16(u16_be(data, 4), 0.1f);
  out.real_soc = airship_utils::scale_u16(u16_be(data, 6), 0.1f);
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
  // 防非对齐帧 ID: 单体电压帧 ID 必须按 0x10 步进(0x3000, 0x3010, ...),
  // 若出现非法 ID(如 0x3005), 直接除会向下取整错位解析到错误节号(不越界但语义错)。
  if (frame_id < kCellVoltageBase || (frame_id - kCellVoltageBase) % 0x10 != 0) {
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

// ErrorCode: 64 个 1-bit 故障/告警位 (Intel 小端, bit0=byte0.bit0 ... bit63=byte7.bit7)
//   一级报警 Warn (bit16-31) + 系统故障位 (bit0-15) -> fault_word1
//   二级报警 Alarm (bit32-47)                        -> fault_word2
//   三级报警 CriticalAlarm (bit48-63)                -> fault_word3
void parse_error_code(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  uint64_t word = 0;
  for (uint32_t i = 0; i < 8; ++i) {
    word |= static_cast<uint64_t>(data[i]) << (i * 8);
  }
  out.fault_word1 = static_cast<uint32_t>(word & 0xFFFFFFFF);         // bit0-31
  out.fault_word2 = static_cast<uint32_t>((word >> 32) & 0xFFFF);     // bit32-47
  out.fault_word3 = static_cast<uint32_t>(word >> 48);                // bit48-63
}

// SOH: 总容量(0.1AH)/循环次数(1)/额定电压(0.1V)/SOH(0.1%) u16 小端
void parse_soh(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  out.pack_total_cap = airship_utils::scale_u16(get_u16_le(data, 0), 0.1f);
  out.charge_times = get_u16_le(data, 2);
  out.pack_rated_voltage = airship_utils::scale_u16(get_u16_le(data, 4), 0.1f);
  out.soh = airship_utils::scale_u16(get_u16_le(data, 6), 0.1f);
}

// SOP: 最大充电单体电压(0.1V)/最大放电电流(0.1A)/最小放电单体电压(0.1V)/最大充电电流(0.1A)
void parse_sop(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  out.charge_max_cell_volt = airship_utils::scale_u16(get_u16_le(data, 0), 0.1f);
  out.max_discharge_current = airship_utils::scale_u16(get_u16_le(data, 2), 0.1f);
  out.discharge_min_cell_volt = airship_utils::scale_u16(get_u16_le(data, 4), 0.1f);
  out.max_charge_current = airship_utils::scale_u16(get_u16_le(data, 6), 0.1f);
}

// CellVoltStatistic: 平均/最大/最小单体电压 (12-bit Intel 小端紧凑交错, 0.001V@+1V)
//   信号起始位: MaxCellVoltIndex=7, MaxCellVolt=11, MinCellVoltIndex=35,
//               MinCellVolt=31, AvgCellVolt=51, Slave_Version=55
// 12-bit Intel 布局需按位提取, 从起始位(含)取 12 位
namespace
{
// 小端位序: 从绝对位索引 start(含)起取 12 位 (Intel 小端, 逐位拼接)
uint16_t extract_12bit_intel(const uint8_t * data, uint8_t start)
{
  uint32_t v = 0;
  for (uint32_t i = 0; i < 12; ++i) {
    const uint32_t bit = start + i;
    const uint8_t byte = data[bit / 8];
    const uint8_t val = static_cast<uint8_t>((byte >> (bit % 8)) & 0x01);
    v |= static_cast<uint32_t>(val) << i;
  }
  return static_cast<uint16_t>(v);
}
}  // namespace

void parse_cell_volt_statistic(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  out.stat_avg_cell_volt = static_cast<float>(extract_12bit_intel(data, 51)) * 0.001f + 1.0f;
  out.stat_max_cell_volt = static_cast<float>(extract_12bit_intel(data, 11)) * 0.001f + 1.0f;
  out.stat_min_cell_volt = static_cast<float>(extract_12bit_intel(data, 31)) * 0.001f + 1.0f;
}

// PoleTempStatistic: 极柱最高温(1℃,-50)/最高温序号/最低温(1℃,-50)/最低温序号/温差(0.001V)
void parse_pole_temp_statistic(const uint8_t * data, uint32_t len, BmsData & out)
{
  if (len < 8) {
    return;
  }
  out.pole_max_temp = static_cast<float>(data[0]) - 50.0f;
  out.pole_min_temp = static_cast<float>(data[3]) - 50.0f;
  out.pole_temp_diff = airship_utils::scale_u16(get_u16_le(data, 6), 0.001f);
}

}  // namespace airship_bms
