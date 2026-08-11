// 灵云01号伴飞电脑 — 12S 备用电源 BMS 串口协议解析实现
#include "airship_backup_bms/backup_bms_protocol.hpp"

#include <cstddef>

namespace airship_backup_bms
{

uint16_t crc16(const uint8_t * buf, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(buf[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

std::vector<uint8_t> build_read_request(uint8_t addr, uint8_t host, uint8_t cmd)
{
  std::vector<uint8_t> v;
  v.reserve(9);
  v.push_back(kReqStart);   // 0x56
  v.push_back(addr);        // 地址
  v.push_back(host);        // 主机信息
  v.push_back(kCmdRead);    // 读写: 读
  v.push_back(cmd);         // 指令
  v.push_back(0x00);        // 长度 H (读请求无数据 => 0)
  v.push_back(0x00);        // 长度 L
  // CRC 覆盖 "地址..长度" (不含起始字节)
  const uint16_t c = crc16(&v[1], static_cast<uint32_t>(v.size() - 1));
  v.push_back(static_cast<uint8_t>((c >> 8) & 0xFF));  // CRC 高
  v.push_back(static_cast<uint8_t>(c & 0xFF));         // CRC 低
  return v;
}

// 读取大端 u16 (带边界检查)
static inline bool read_u16(const uint8_t * d, uint32_t len, uint32_t off, uint16_t & out)
{
  if (off + 2 > len) {
    return false;
  }
  out = static_cast<uint16_t>((static_cast<uint16_t>(d[off]) << 8) | d[off + 1]);
  return true;
}

// 读取大端 s16 (带边界检查)
static inline bool read_s16(const uint8_t * d, uint32_t len, uint32_t off, int16_t & out)
{
  if (off + 2 > len) {
    return false;
  }
  out = static_cast<int16_t>((static_cast<uint16_t>(d[off]) << 8) | d[off + 1]);
  return true;
}

// 读取大端 s32 (带边界检查)
static inline bool read_s32(const uint8_t * d, uint32_t len, uint32_t off, int32_t & out)
{
  if (off + 4 > len) {
    return false;
  }
  out = static_cast<int32_t>(
    (static_cast<uint32_t>(d[off]) << 24) |
    (static_cast<uint32_t>(d[off + 1]) << 16) |
    (static_cast<uint32_t>(d[off + 2]) << 8) |
    static_cast<uint32_t>(d[off + 3]));
  return true;
}

bool parse_response_frame(
  const uint8_t * frame, uint32_t len, uint8_t addr, uint8_t host,
  uint8_t expect_cmd, const uint8_t ** data, uint32_t * data_len)
{
  // 最小响应: 起始+地址+主机+读写+指令+长度(2)+CRC(2) = 9
  if (frame == nullptr || len < 9) {
    return false;
  }
  if (frame[0] != kRespStart) {
    return false;
  }
  if (frame[1] != addr) {
    return false;
  }
  if (frame[2] != host) {
    return false;
  }
  if (frame[3] != kCmdRead) {
    return false;  // 读响应读写标志应为 0
  }
  if (frame[4] != expect_cmd) {
    return false;
  }
  const uint32_t dlen = (static_cast<uint32_t>(frame[5]) << 8) | frame[6];
  if (7 + dlen + 2 != len) {
    return false;  // 长度必须与帧长度精确匹配
  }
  const uint16_t expected = crc16(&frame[1], 6 + dlen);  // 地址..数据
  const uint16_t got = static_cast<uint16_t>(
    (static_cast<uint16_t>(frame[7 + dlen]) << 8) | frame[8 + dlen]);
  if (expected != got) {
    return false;
  }
  *data = &frame[7];
  *data_len = dlen;
  return true;
}

bool parse_basic_info(const uint8_t * data, uint32_t len, BackupBmsData & out)
{
  // 需读到"系统状态1"(偏移54..57)才视为核心数据可用; 不足则安全返回 false
  if (data == nullptr || len < 58) {
    return false;
  }

  uint16_t u16 = 0;
  int16_t s16 = 0;
  int32_t s32 = 0;

  // 总电压 0.01V
  if (read_u16(data, len, 0, u16)) {out.pack_voltage = u16 * 0.01f;}
  // 总电流 0.01A (32 位有符号, 高 16 位在前)
  if (read_s32(data, len, 2, s32)) {out.pack_current = s32 * 0.01f;}
  // SOC 0.1%
  if (read_u16(data, len, 6, u16)) {out.soc = u16 * 0.1f;}
  // SOH 0.1%
  if (read_u16(data, len, 8, u16)) {out.soh = u16 * 0.1f;}
  // 最大电压 1mV
  if (read_u16(data, len, 10, u16)) {out.max_cell_voltage = u16 * 0.001f;}
  // 最小电压 1mV
  if (read_u16(data, len, 14, u16)) {out.min_cell_voltage = u16 * 0.001f;}
  // 压差 1mV
  if (read_u16(data, len, 18, u16)) {out.cell_voltage_diff = u16 * 0.001f;}
  // 最大温度 0.1℃ (有符号)
  if (read_s16(data, len, 20, s16)) {out.max_cell_temp = s16 * 0.1f;}
  // 最小温度 0.1℃ (有符号)
  if (read_s16(data, len, 24, s16)) {out.min_cell_temp = s16 * 0.1f;}
  // 温差 0.1℃
  if (read_s16(data, len, 28, s16)) {out.temp_diff = s16 * 0.1f;}
  // 平均温度 0.1℃
  if (read_s16(data, len, 30, s16)) {out.avg_cell_temp = s16 * 0.1f;}

  // 告警/保护/故障/系统状态 (各 32 位, 高字节在前)
  if (read_s32(data, len, 42, s32)) {out.alarm_word = static_cast<uint32_t>(s32);}
  if (read_s32(data, len, 46, s32)) {out.protect_word = static_cast<uint32_t>(s32);}
  if (read_s32(data, len, 50, s32)) {out.fault_word = static_cast<uint32_t>(s32);}
  if (read_s32(data, len, 54, s32)) {out.system_word = static_cast<uint32_t>(s32);}

  return true;
}

bool parse_cell_voltages(const uint8_t * data, uint32_t len, BackupBmsData & out)
{
  if (data == nullptr || len < 2) {
    return false;
  }
  const uint16_t count = static_cast<uint16_t>((data[0] << 8) | data[1]);
  // 需 len >= 2 + 2*count
  if (len < 2 + static_cast<uint32_t>(count) * 2u) {
    return false;
  }
  out.cell_voltages.clear();
  out.cell_voltages.reserve(count);
  for (uint16_t i = 0; i < count; ++i) {
    const uint16_t raw = static_cast<uint16_t>((data[2 + i * 2] << 8) | data[3 + i * 2]);
    out.cell_voltages.push_back(raw * 0.001f);  // 1mV
  }
  return true;
}

bool parse_cell_temps(const uint8_t * data, uint32_t len, BackupBmsData & out)
{
  if (data == nullptr || len < 2) {
    return false;
  }
  const uint16_t count = static_cast<uint16_t>((data[0] << 8) | data[1]);
  if (len < 2 + static_cast<uint32_t>(count) * 2u) {
    return false;
  }
  out.cell_temps.clear();
  out.cell_temps.reserve(count);
  for (uint16_t i = 0; i < count; ++i) {
    const int16_t raw = static_cast<int16_t>((data[2 + i * 2] << 8) | data[3 + i * 2]);
    out.cell_temps.push_back(raw * 0.1f);  // 0.1℃ 有符号
  }
  return true;
}

}  // namespace airship_backup_bms
