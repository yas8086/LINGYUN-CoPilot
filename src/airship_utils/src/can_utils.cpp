// 灵云01号伴飞电脑 — CAN 帧解析工具实现
#include "airship_utils/can_utils.hpp"

#include <cstring>

namespace airship_utils
{

uint16_t get_u16_le(const uint8_t * data, size_t offset)
{
  // 小端序: 低字节在前
  uint16_t v;
  std::memcpy(&v, data + offset, sizeof(v));
  return v;
}

uint32_t get_u32_le(const uint8_t * data, size_t offset)
{
  uint32_t v;
  std::memcpy(&v, data + offset, sizeof(v));
  return v;
}

int16_t get_i16_le(const uint8_t * data, size_t offset)
{
  int16_t v;
  std::memcpy(&v, data + offset, sizeof(v));
  return v;
}

uint8_t get_u8(const uint8_t * data, size_t offset)
{
  return data[offset];
}

uint8_t get_bits2(const uint8_t * data, size_t byte_offset, uint8_t bit_pos)
{
  if (bit_pos > 6) {
    return 0;
  }
  return static_cast<uint8_t>((data[byte_offset] >> bit_pos) & 0x03U);
}

float scale_i16(int16_t raw, float scale, float offset)
{
  return static_cast<float>(raw) * scale + offset;
}

float scale_u16(uint16_t raw, float scale, float offset)
{
  return static_cast<float>(raw) * scale + offset;
}

float temp_with_offset(uint8_t raw)
{
  // 协议约定: 1℃/BIT, -40 偏移; raw 为无符号字节(0~255 → -40~215℃)
  return static_cast<float>(raw) - 40.0f;
}

}  // namespace airship_utils
