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
  EXPECT_EQ(airship_bms::kErrorCode, 0x001FE0U);
  EXPECT_EQ(airship_bms::kSoh, 0x005FF0U);
  EXPECT_EQ(airship_bms::kSop, 0x001100U);
  EXPECT_EQ(airship_bms::kCellVoltStatistic, 0x001200U);
  EXPECT_EQ(airship_bms::kPoleTempStatistic, 0x001700U);
}

TEST(BmsProtocol, ParseBattInfo)
{
  // 2026-08-26 实机 candump 静置帧: 00001400 [8] 0F 16 27 10 01 AC 01 8A
  // 总压 0x0F16=3862 → 386.2V; 电流 0x2710=10000 → 1000.0-1000=0A;
  // SOC 0x01AC=428 → 42.8%; RealSoc 0x018A=394 → 39.4%
  // (DBC L474 确认 BattCurr offset=-1000; 旧代码 -100 算出 900A 即当日 900A bug)
  const std::array<uint8_t, 8> d = {
    0x0F, 0x16, 0x27, 0x10, 0x01, 0xAC, 0x01, 0x8A,
  };
  BmsData out;
  airship_bms::parse_batt_info(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.pack_voltage, 386.2f);
  EXPECT_FLOAT_EQ(out.pack_current, 0.0f);
  EXPECT_FLOAT_EQ(out.soc, 42.8f);
  EXPECT_FLOAT_EQ(out.real_soc, 39.4f);
}

TEST(BmsProtocol, ParseBattInfoDischarge5A)
{
  // 放电 +5A: raw = (5+1000)/0.1 = 10050 = 0x2742(BE); 其余保持 367.2V/85%/85%
  const std::array<uint8_t, 8> d = {
    0x0E, 0x58, 0x27, 0x42, 0x03, 0x52, 0x03, 0x52,
  };
  BmsData out;
  airship_bms::parse_batt_info(d.data(), 8, out);
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
  airship_bms::parse_batt_status(d.data(), 8, out);
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
  airship_bms::parse_cell_voltage(0x003000, d.data(), 8, out);
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
  airship_bms::parse_cell_voltage(0x003010, d.data(), 8, out);
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
  airship_bms::parse_cell_temp_statistic(d.data(), 8, out);
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
  airship_bms::parse_pack_temp(d.data(), 8, out);
  for (uint32_t i = 0; i < 8; ++i) {
    EXPECT_FLOAT_EQ(out.pole_temps[i], 25.0f + static_cast<float>(i) * 5.0f);
  }
}

// ErrorCode: 64 位故障/告警字 (小端)
//   bit0(byte0.0)=1 -> fault_word1 低 16 位系统故障位
//   bit16(byte2.0)=1 -> fault_word1 高 16 位一级报警 Warn
//   bit32(byte4.0)=1 -> fault_word2
//   bit48(byte6.0)=1 -> fault_word3
TEST(BmsProtocol, ParseErrorCode)
{
  const std::array<uint8_t, 8> d = {0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00};
  BmsData out;
  airship_bms::parse_error_code(d.data(), 8, out);
  EXPECT_EQ(out.fault_word1, 0x00010001U);  // bit0 + bit16
  EXPECT_EQ(out.fault_word2, 0x0001U);      // bit32
  EXPECT_EQ(out.fault_word3, 0x0001U);      // bit48
}

// SOH: 总容量(0.1AH)/循环次数(1)/额定电压(0.1V)/SOH(0.1%) u16 小端
TEST(BmsProtocol, ParseSoh)
{
  // 200.0AH, 150次, 374.4V, 95.0%
  const std::array<uint8_t, 8> d = {
    0xD0, 0x07, 0x96, 0x00, 0xA0, 0x0E, 0xB6, 0x03,
  };
  BmsData out;
  airship_bms::parse_soh(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.pack_total_cap, 200.0f);
  EXPECT_EQ(out.charge_times, 150);
  EXPECT_FLOAT_EQ(out.pack_rated_voltage, 374.4f);
  EXPECT_FLOAT_EQ(out.soh, 95.0f);
}

// SOP: 最大充电单节电压(0.1V)/最大放电电流(0.1A)/最小放电单节电压(0.1V)/最大充电电流(0.1A)
TEST(BmsProtocol, ParseSop)
{
  // 4.20V / 20.0A / 2.80V / 10.0A
  const std::array<uint8_t, 8> d = {
    0x2A, 0x00, 0xC8, 0x00, 0x1C, 0x00, 0x64, 0x00,
  };
  BmsData out;
  airship_bms::parse_sop(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.charge_max_cell_volt, 4.20f);
  EXPECT_FLOAT_EQ(out.max_discharge_current, 20.0f);
  EXPECT_FLOAT_EQ(out.discharge_min_cell_volt, 2.80f);
  EXPECT_FLOAT_EQ(out.max_charge_current, 10.0f);
}

// CellVoltStatistic: 平均/最大/最小单体电压 (12-bit Intel, 0.001V@+1V)
// 仅构造 AvgCellVolt 位(起始位 51), 其余位清零: raw 2200 -> 3.2V
TEST(BmsProtocol, ParseCellVoltStatistic)
{
  // raw 2200 (0b100010011000, bit3/4/7/11=1):
  //   位51-55(byte6 bit3-7): bit3,4=0 bit5,6=1 bit7=0 -> byte6=0xC0
  //   位56-62(byte7 bit0-6): bit0=0 bit1=0 bit2=1 bit3=0 bit4=0 bit5=0 bit6=1 -> byte7=0x44
  const std::array<uint8_t, 8> d = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x44,
  };
  BmsData out;
  airship_bms::parse_cell_volt_statistic(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.stat_avg_cell_volt, 3.2f);
}

// PoleTempStatistic: 极柱最高/最低温(1℃,-50) 与温差(0.001V)
TEST(BmsProtocol, ParsePoleTempStatistic)
{
  // 最高温 40℃(byte=90)/序号3, 最低温 25℃(byte=75)/序号2, 温差 0.5V(raw 500)
  const std::array<uint8_t, 8> d = {
    90, 3, 0, 75, 2, 0, 0xF4, 0x01,
  };
  BmsData out;
  airship_bms::parse_pole_temp_statistic(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.pole_max_temp, 40.0f);
  EXPECT_FLOAT_EQ(out.pole_min_temp, 25.0f);
  EXPECT_FLOAT_EQ(out.pole_temp_diff, 0.5f);
}

// 帧长不足时应忽略该帧, 不产生越界或假数据
TEST(BmsProtocol, ShortFrameIgnored)
{
  BmsData out;
  const std::array<uint8_t, 4> d = {0x58, 0x0E, 0x1A, 0x04};
  // 传入不足长度, 各解析函数应安全返回且不修改结果
  airship_bms::parse_batt_info(d.data(), 4, out);
  airship_bms::parse_batt_status(d.data(), 4, out);
  airship_bms::parse_cell_voltage(0x003000, d.data(), 4, out);
  airship_bms::parse_cell_temp_statistic(d.data(), 4, out);
  airship_bms::parse_pack_temp(d.data(), 4, out);
  airship_bms::parse_error_code(d.data(), 4, out);
  airship_bms::parse_soh(d.data(), 4, out);
  airship_bms::parse_sop(d.data(), 4, out);
  airship_bms::parse_cell_volt_statistic(d.data(), 4, out);
  airship_bms::parse_pole_temp_statistic(d.data(), 4, out);
  EXPECT_EQ(out.pack_voltage, 0.0f);
  EXPECT_EQ(out.alarm_level, 0);
  EXPECT_EQ(out.cell_voltages[0], 0.0f);
  EXPECT_EQ(out.pole_temps[0], 0.0f);
  EXPECT_EQ(out.fault_word1, 0U);
  EXPECT_EQ(out.soh, 0.0f);
  EXPECT_EQ(out.max_charge_current, 0.0f);
  EXPECT_EQ(out.stat_avg_cell_volt, 0.0f);
  EXPECT_EQ(out.pole_temp_diff, 0.0f);
}
