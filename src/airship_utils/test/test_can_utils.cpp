// 灵云01号伴飞电脑 — can_utils 单元测试
#include "airship_utils/can_utils.hpp"

#include <gtest/gtest.h>

using airship_utils::get_bits2;
using airship_utils::get_i16_le;
using airship_utils::get_u16_le;
using airship_utils::get_u32_le;
using airship_utils::get_u8;
using airship_utils::scale_i16;
using airship_utils::scale_u16;
using airship_utils::temp_with_offset;

// ===== 小端序整数解析 =====
TEST(CapUtilsTest, U16LittleEndian)
{
  const uint8_t data[8] = {0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // 0x01E0 = 480
  EXPECT_EQ(get_u16_le(data, 0), 480U);
}

TEST(CapUtilsTest, U32LittleEndian)
{
  const uint8_t data[8] = {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  // 0x01000000 = 16777216
  EXPECT_EQ(get_u32_le(data, 0), 16777216U);
}

TEST(CapUtilsTest, I16LittleEndian)
{
  // 0xFFFE = -2 (补码)
  const uint8_t data[8] = {0xFE, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(get_i16_le(data, 0), -2);
}

TEST(CapUtilsTest, U8)
{
  const uint8_t data[8] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(get_u8(data, 0), 0x10U);
}

// ===== 位域提取 (DCDC 启停位 Bit5~4) =====
TEST(CapUtilsTest, Bits2)
{
  // 0x10 = 0b00010000 -> Bit5~4 = 01
  const uint8_t data[8] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(get_bits2(data, 0, 4), 1U);
}

// ===== 单位换算 =====
TEST(CapUtilsTest, ScaleVoltage)
{
  // 0.1V/BIT: raw=480 -> 48.0V
  EXPECT_FLOAT_EQ(scale_u16(480, 0.1f), 48.0f);
}

TEST(CapUtilsTest, ScaleCurrent)
{
  // 0.1A/BIT: raw=800 -> 80.0A
  EXPECT_FLOAT_EQ(scale_u16(800, 0.1f), 80.0f);
}

TEST(CapUtilsTest, ScaleSigned)
{
  // 有符号负值
  EXPECT_FLOAT_EQ(scale_i16(-100, 0.01f), -1.0f);
}

// ===== 温度 -40 偏移 =====
TEST(CapUtilsTest, TempOffset)
{
  // 0 -> -40℃, 40 -> 0℃, 80 -> 40℃
  EXPECT_FLOAT_EQ(temp_with_offset(0), -40.0f);
  EXPECT_FLOAT_EQ(temp_with_offset(40), 0.0f);
  EXPECT_FLOAT_EQ(temp_with_offset(80), 40.0f);
}

// ===== 边界: 模拟 DCDC 开机帧整体解析 =====
// 开机 + 48.0V + 80.0A: Data = 00 00 10 E0 01 20 03 00
TEST(CapUtilsTest, DcdcPowerOnFrame)
{
  const uint8_t data[8] = {0x00, 0x00, 0x10, 0xE0, 0x01, 0x20, 0x03, 0x00};
  EXPECT_EQ(get_bits2(data, 2, 4), 1U);                 // 开机
  EXPECT_FLOAT_EQ(scale_u16(get_u16_le(data, 3), 0.1f), 48.0f);  // 电压
  EXPECT_FLOAT_EQ(scale_u16(get_u16_le(data, 5), 0.1f), 80.0f);  // 电流
}
