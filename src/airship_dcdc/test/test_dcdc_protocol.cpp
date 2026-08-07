// 灵云01号伴飞电脑 — DCDC 协议解析单元测试
#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "airship_dcdc/dcdc_protocol.hpp"

using airship_dcdc::DcdcData;

TEST(DcdcProtocol, ControlFrameOn)
{
  // 开机 + 48.0V + 80.0A -> 文档示例: 00 00 10 E0 01 20 03 00
  const auto frame = airship_dcdc::build_control_frame(true, 48.0f, 80.0f);
  EXPECT_EQ(frame.id, 0x18EF3010U);
  EXPECT_TRUE(frame.extended);
  EXPECT_EQ(frame.len, 8U);
  EXPECT_EQ(frame.data[2], (uint8_t)0x10);  // Bit5~4 = 01 开机
  EXPECT_EQ(frame.data[3], (uint8_t)0xE0);  // 电压低字节 480=0x01E0
  EXPECT_EQ(frame.data[4], (uint8_t)0x01);
  EXPECT_EQ(frame.data[5], (uint8_t)0x20);  // 电流低字节 800=0x0320
  EXPECT_EQ(frame.data[6], (uint8_t)0x03);
  EXPECT_EQ(frame.data[7], (uint8_t)0x00);
}

TEST(DcdcProtocol, ControlFrameOff)
{
  // 关机: 全 0
  const auto frame = airship_dcdc::build_control_frame(false, 48.0f, 80.0f);
  EXPECT_EQ(frame.data[2], (uint8_t)0x00);
}

TEST(DcdcProtocol, AnalogQueryFrame)
{
  const auto frame = airship_dcdc::build_analog_query_frame();
  EXPECT_EQ(frame.id, 0x18D80047U);
  EXPECT_TRUE(frame.extended);
  EXPECT_EQ(frame.len, 8U);
}

TEST(DcdcProtocol, ParseStatus)
{
  // 故障字节: BIT2(输出开) + BIT5(短路) + BIT7(总故障) = 0x80|0x20|0x04 = 0xA4
  const std::array<uint8_t, 8> d = {0xA4, 0, 0, 0, 0, 0, 0, 0};
  DcdcData out;
  airship_dcdc::parse_status(d.data(), out);
  EXPECT_EQ(out.fault_word, 0xA4);
  EXPECT_TRUE(out.output_enabled);
  EXPECT_TRUE((out.fault_word & airship_dcdc::fault::kShortCircuit) != 0);
  EXPECT_TRUE((out.fault_word & airship_dcdc::fault::kFault) != 0);
}

TEST(DcdcProtocol, ParseStatusOutputOff)
{
  // 仅 BIT0 输入欠压
  const std::array<uint8_t, 8> d = {0x01, 0, 0, 0, 0, 0, 0, 0};
  DcdcData out;
  airship_dcdc::parse_status(d.data(), out);
  EXPECT_FALSE(out.output_enabled);
  EXPECT_TRUE((out.fault_word & airship_dcdc::fault::kInputUndervolt) != 0);
}

TEST(DcdcProtocol, ParseAnalog)
{
  // 输入 360.0V(360), 输出 48.0V(480), 电流 12.5A(125), 温度 45℃/55℃
  // temp_with_offset(raw) = raw - 40. 要 45℃ -> raw=85, 55℃ -> raw=95
  const std::array<uint8_t, 8> d = {0x68, 0x01, 0xE0, 0x01, 0x7D, 0x00, 85, 95};
  DcdcData out;
  airship_dcdc::parse_analog(d.data(), out);
  EXPECT_FLOAT_EQ(out.input_voltage, 360.0f);
  EXPECT_FLOAT_EQ(out.output_voltage, 48.0f);
  EXPECT_FLOAT_EQ(out.output_current, 12.5f);
  EXPECT_FLOAT_EQ(out.ambient_temp, 45.0f);
  EXPECT_FLOAT_EQ(out.heatsink_temp, 55.0f);
}