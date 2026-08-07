// 灵云01号伴飞电脑 — BMS 协议解析单元测试
#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "airship_bms/bms_protocol.hpp"

using airship_bms::BmsData;

TEST(BmsProtocol, QueryFrameIds)
{
  // 确认关键帧 ID 常量符合 DBC
  EXPECT_EQ(airship_bms::kBattInfo02, 0x001400U);
  EXPECT_EQ(airship_bms::kBattInfo01, 0x001300U);
  EXPECT_EQ(airship_bms::kCellVoltageBase, 0x003000U);
  EXPECT_EQ(airship_bms::kCellTempStatistic, 0x001500U);
  EXPECT_EQ(airship_bms::kPackTemp, 0x002110U);
}

TEST(BmsProtocol, ParseBattInfo)
{
  // 总压 367.2V, 电流 5.0A, SOC 85.0%, RealSoc 85.0%
  // 小端: v raw 0x0E58, c raw 0x041A, soc raw 0x0352
  const std::array<uint8_t, 8> d = {
    0x58, 0x0E, 0x1A, 0x04, 0x52, 0x03, 0x52, 0x03,
  };
  BmsData out;
  airship_bms::parse_batt_info(d.data(), out);
  EXPECT_FLOAT_EQ(out.pack_voltage, 367.2f);
  EXPECT_FLOAT_EQ(out.pack_current, 5.0f);
  EXPECT_FLOAT_EQ(out.soc, 85.0f);
  EXPECT_FLOAT_EQ(out.real_soc, 85.0f);
}

TEST(BmsProtocol, ParseBattStatus)
{
  // run_state=1, connection_status=2, 正极绝缘 1000kΩ, 负极 1100kΩ, 报警 3
  const std::array<uint8_t, 8> d = {
    0x21, 0x00, 0xE8, 0x03, 0x4C, 0x04, 0x03, 0x00,
  };
  BmsData out;
  airship_bms::parse_batt_status(d.data(), out);
  EXPECT_EQ(out.run_state, 1);
  EXPECT_EQ(out.connection_status, 2);
  EXPECT_EQ(out.positive_insulation_kohm, 1000);
  EXPECT_EQ(out.negative_insulation_kohm, 1100);
  EXPECT_EQ(out.alarm_level, 3);
}

TEST(BmsProtocol, ParseCellVoltage)
{
  // 使用 cantools 实测编码: CellVoltage_01 (0x3000) 5 节 3.2/3.3/3.4/3.5/3.6V
  const std::array<uint8_t, 8> d = {
    0x89, 0x88, 0xFC, 0x96, 0x09, 0xC4, 0x0A, 0x28,
  };
  BmsData out;
  airship_bms::parse_cell_voltage(0x003000, d.data(), out);
  EXPECT_FLOAT_EQ(out.cell_voltages[0], 3.2f);
  EXPECT_FLOAT_EQ(out.cell_voltages[1], 3.3f);
  EXPECT_FLOAT_EQ(out.cell_voltages[2], 3.4f);
  EXPECT_FLOAT_EQ(out.cell_voltages[3], 3.5f);
  EXPECT_FLOAT_EQ(out.cell_voltages[4], 3.6f);
}

TEST(BmsProtocol, ParseCellVoltageSecondFrame)
{
  // CellVoltage_02 (0x3010) 对应节 5~9
  const std::array<uint8_t, 8> d = {
    0x89, 0x88, 0xFC, 0x96, 0x09, 0xC4, 0x0A, 0x28,
  };
  BmsData out;
  airship_bms::parse_cell_voltage(0x003010, d.data(), out);
  EXPECT_FLOAT_EQ(out.cell_voltages[5], 3.2f);
  EXPECT_FLOAT_EQ(out.cell_voltages[9], 3.6f);
}

TEST(BmsProtocol, ParseCellTempStatistic)
{
  // max=35, min=30, avg=32, diff=5 (byte = 温度+50)
  const std::array<uint8_t, 8> d = {
    85, 0, 0, 80, 0, 0, 82, 55,
  };
  BmsData out;
  airship_bms::parse_cell_temp_statistic(d.data(), out);
  EXPECT_FLOAT_EQ(out.max_cell_temp, 35.0f);
  EXPECT_FLOAT_EQ(out.min_cell_temp, 30.0f);
  EXPECT_FLOAT_EQ(out.avg_cell_temp, 32.0f);
  EXPECT_FLOAT_EQ(out.temp_diff, 5.0f);
}

TEST(BmsProtocol, ParsePackTemp)
{
  // 8 点极耳温度 25/30/35/40/45/50/55/60 (byte = 温度+50)
  const std::array<uint8_t, 8> d = {
    75, 80, 85, 90, 95, 100, 105, 110,
  };
  BmsData out;
  airship_bms::parse_pack_temp(d.data(), out);
  for (uint32_t i = 0; i < 8; ++i) {
    EXPECT_FLOAT_EQ(out.pole_temps[i], 25.0f + static_cast<float>(i) * 5.0f);
  }
}