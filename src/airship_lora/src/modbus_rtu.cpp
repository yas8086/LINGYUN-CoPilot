// 灵云01号伴飞电脑 — LoRa 集中器 Modbus RTU 协议解析实现
#include "airship_lora/modbus_rtu.hpp"

namespace airship_lora
{

uint16_t modbus_crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 1) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

std::vector<uint8_t> build_read_request(
  uint8_t slave_addr, uint16_t start_addr, uint16_t quantity)
{
  std::vector<uint8_t> frame;
  frame.reserve(8);
  frame.push_back(slave_addr);
  frame.push_back(0x04);  // 读输入寄存器
  frame.push_back(static_cast<uint8_t>((start_addr >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(start_addr & 0xFF));
  frame.push_back(static_cast<uint8_t>((quantity >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(quantity & 0xFF));
  const uint16_t crc = modbus_crc16(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));       // CRC 低字节在前
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return frame;
}

bool check_crc(const std::vector<uint8_t> & frame)
{
  if (frame.size() < 4) {
    return false;
  }
  const uint16_t expected =
    static_cast<uint16_t>(frame[frame.size() - 2]) |
    static_cast<uint16_t>(frame[frame.size() - 1] << 8);
  const uint16_t calc = modbus_crc16(frame.data(), frame.size() - 2);
  return calc == expected;
}

bool parse_temp_response(
  const std::vector<uint8_t> & resp, uint8_t slave_addr, float & temp_celsius, int & raw)
{
  // 温度响应: [slave][0x04][0x02][dHi][dLo][crcLo][crcHi] = 7 字节
  if (resp.size() != 7 || resp[1] != 0x04 || resp[2] != 0x02) {
    return false;
  }
  // 校验从机地址, 防止 485 总线串扰/残留帧被误解析
  if (resp[0] != slave_addr) {
    return false;
  }
  if (!check_crc(resp)) {
    return false;
  }
  raw = static_cast<int16_t>(
    static_cast<int16_t>(static_cast<uint16_t>(resp[3] << 8) | resp[4]));
  temp_celsius = static_cast<float>(raw) / 10.0f;
  return true;
}

bool parse_pressure_response(
  const std::vector<uint8_t> & resp, uint8_t slave_addr, double & pressure_pa)
{
  // 压力响应: [slave][0x04][0x04][d3][d2][d1][d0][crcLo][crcHi] = 9 字节
  // 前 4 字节为大端两个寄存器: high=(d3<<8)|d2, low=(d1<<8)|d0
  if (resp.size() != 9 || resp[1] != 0x04 || resp[2] != 0x04) {
    return false;
  }
  // 校验从机地址, 防止 485 总线串扰/残留帧被误解析
  if (resp[0] != slave_addr) {
    return false;
  }
  if (!check_crc(resp)) {
    return false;
  }
  const uint32_t high = static_cast<uint32_t>((resp[3] << 8) | resp[4]);
  const uint32_t low = static_cast<uint32_t>((resp[5] << 8) | resp[6]);
  // 压力为相对大气压的压差, 是有符号 32 位 (实测 15 号节点 0xFFFFFFEF = -17 Pa,
  // 按无符号解析会得到 4294967279, 误判为正大值)
  const int32_t signed_pa = static_cast<int32_t>((high << 16) | low);
  pressure_pa = static_cast<double>(signed_pa);
  return true;
}

int check_temp_alarm(float temp_celsius, float low, float high)
{
  if (temp_celsius > high) {
    return 1;
  }
  if (temp_celsius < low) {
    return -1;
  }
  return 0;
}

}  // namespace airship_lora
