// 灵云01号伴飞电脑 — json_packer 单元测试
#include "airship_link/json_packer.hpp"

#include <gtest/gtest.h>

#include <string>

using airship_link::bms_to_json;
using airship_link::dcdc_to_json;
using airship_link::mppt_to_json;
using airship_link::pack_telemetry_json;

// ===== BMS =====
TEST(JsonPackerTest, BmsOnline)
{
  airship_msgs::msg::BmsStatus msg;
  msg.online = true;
  msg.pack_voltage = 48.0f;
  msg.pack_current = 10.5f;
  msg.soc = 85.0f;
  msg.real_soc = 82.0f;
  msg.max_cell_temp = 30.0f;
  msg.min_cell_temp = 25.0f;
  msg.avg_cell_temp = 27.5f;
  msg.temp_diff = 5.0f;
  msg.positive_insulation_kohm = 1000;
  msg.negative_insulation_kohm = 1100;
  msg.alarm_level = 1;

  const std::string s = bms_to_json(msg);
  EXPECT_NE(s.find("\"bms\""), std::string::npos);
  EXPECT_NE(s.find("\"pack_v\":48"), std::string::npos);
  EXPECT_NE(s.find("\"soc\":85"), std::string::npos);
  EXPECT_NE(s.find("\"rsoc\":82"), std::string::npos);
  EXPECT_NE(s.find("\"max_t\":30"), std::string::npos);
  EXPECT_NE(s.find("\"min_t\":25"), std::string::npos);
  EXPECT_NE(s.find("\"avg_t\":27.5"), std::string::npos);
  EXPECT_NE(s.find("\"diff_t\":5"), std::string::npos);
  EXPECT_NE(s.find("\"riso_p\":1000"), std::string::npos);
  EXPECT_NE(s.find("\"riso_n\":1100"), std::string::npos);
  EXPECT_NE(s.find("\"alarm\":1"), std::string::npos);
}

TEST(JsonPackerTest, MpptOnline)
{
  airship_msgs::msg::MpptStatus msg;
  msg.online = true;
  msg.pv_voltage = 300.0f;
  msg.energy_total = 123.4f;
  msg.fault_state = 0;

  const std::string s = mppt_to_json(msg);
  EXPECT_NE(s.find("\"mppt\""), std::string::npos);
  EXPECT_NE(s.find("\"pv_v\":300"), std::string::npos);
}

TEST(JsonPackerTest, DcdcOnline)
{
  airship_msgs::msg::DcdcStatus msg;
  msg.online = true;
  msg.output_voltage = 48.0f;
  msg.output_power = 2000.0f;

  const std::string s = dcdc_to_json(msg);
  EXPECT_NE(s.find("\"dcdc\""), std::string::npos);
  // 2000.0 在 %.4g 下输出 "2000"(4 位有效数字), 不再用 %.3g 的科学计数法 "2e+03"
  EXPECT_NE(s.find("\"out_p\":2000"), std::string::npos);
}

// ===== 完整打包 =====
TEST(JsonPackerTest, PackTelemetryWithAll)
{
  airship_msgs::msg::BmsStatus bms;
  bms.online = true;
  bms.soc = 50.0f;
  airship_msgs::msg::MpptStatus mppt;
  mppt.online = true;
  mppt.pv_voltage = 100.0f;
  airship_msgs::msg::DcdcStatus dcdc;
  dcdc.online = true;
  dcdc.output_power = 100.0f;

  const std::string s = pack_telemetry_json(1234.5, &bms, &mppt, &dcdc);
  EXPECT_NE(s.find("\"t\":1234.5"), std::string::npos);
  EXPECT_NE(s.find("\"bms\""), std::string::npos);
  EXPECT_NE(s.find("\"mppt\""), std::string::npos);
  EXPECT_NE(s.find("\"dcdc\""), std::string::npos);
}

TEST(JsonPackerTest, PackTelemetryOfflineOmitted)
{
  airship_msgs::msg::BmsStatus bms;
  bms.online = false;  // 离线设备不打包
  airship_msgs::msg::MpptStatus mppt;
  mppt.online = true;
  mppt.pv_voltage = 100.0f;

  const std::string s = pack_telemetry_json(1.0, &bms, &mppt, nullptr);
  EXPECT_NE(s.find("\"t\":1"), std::string::npos);
  EXPECT_EQ(s.find("\"bms\""), std::string::npos);   // 离线 BMS 不应出现
  EXPECT_NE(s.find("\"mppt\""), std::string::npos);
  EXPECT_EQ(s.find("\"dcdc\""), std::string::npos);  // nullptr 不应出现
}
