// 灵云01号伴飞电脑 — 12S 备用电源 BMS 串口协议解析单元测试
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "airship_backup_bms/backup_bms_protocol.hpp"

using airship_backup_bms::BackupBmsData;
using airship_backup_bms::build_read_request;
using airship_backup_bms::crc16;
using airship_backup_bms::kCmdBasicInfo;
using airship_backup_bms::kCmdCellTemp;
using airship_backup_bms::kCmdCellVoltage;
using airship_backup_bms::kDefaultAddr;
using airship_backup_bms::kHostUpper;
using airship_backup_bms::parse_basic_info;
using airship_backup_bms::parse_cell_temps;
using airship_backup_bms::parse_cell_voltages;
using airship_backup_bms::parse_response_frame;

// 协议文档示例: 读项目号 56 FF 10 00 01 00 00 17 84
TEST(BackupBmsProtocol, Crc16MatchesDocumentExample)
{
  const uint8_t frame[] = {0xFF, 0x10, 0x00, 0x01, 0x00, 0x00};
  // 文档校验字节为 17 84 (高字节在前) => crc = 0x1784
  EXPECT_EQ(crc16(frame, sizeof(frame)), 0x1784u);
}

TEST(BackupBmsProtocol, BuildReadRequestMatchesDocument)
{
  // 读基本信息1: 56 FF 10 00 06 00 00 D6 35 (文档示例)
  const auto req = build_read_request(kDefaultAddr, kHostUpper, kCmdBasicInfo);
  ASSERT_EQ(req.size(), 9u);
  EXPECT_EQ(req[0], 0x56);
  EXPECT_EQ(req[1], 0xFF);
  EXPECT_EQ(req[2], 0x10);
  EXPECT_EQ(req[3], 0x00);
  EXPECT_EQ(req[4], 0x06);
  EXPECT_EQ(req[5], 0x00);
  EXPECT_EQ(req[6], 0x00);
  EXPECT_EQ(req[7], 0xD6);  // CRC 高
  EXPECT_EQ(req[8], 0x35);  // CRC 低
}

// 用协议文档 0x06 响应示例构造帧并验证解析
TEST(BackupBmsProtocol, ParseBasicInfoFromDocumentExample)
{
  // 数据段: 15 CE 00 00 00 00 02 BC 03 E8 0A EB 00 13 0A E2 00 09 00 09 00 B6 00 01 00 B0
  //         00 02 00 06 00 B3 FE 0C 00 B3 0A E6 0B B8 00 00 00 00 00 0A 00 00 ...
  //  总电压=0x15CE=5582 *0.01 = 55.82V
  //  总电流(32位)=0x00000000 = 0A
  //  SOC=0x02BC=700 *0.1 = 70%
  //  SOH=0x03E8=1000 *0.1 = 100%
  //  最大电压=0x0AEB=2795 *0.001 = 2.795V
  //  最小电压=0x0AE2=2786 *0.001 = 2.786V
  //  压差=0x0009=9 *0.001 = 0.009V
  //  最大温度=0x00B6=182 *0.1 = 18.2℃
  //  最小温度=0x00B0=176 *0.1 = 17.6℃
  //  温差=0x0006=6 *0.1 = 0.6℃
  //  平均温度=0x00B3=179 *0.1 = 17.9℃
  const std::vector<uint8_t> data = {
    0x15, 0xCE, 0x00, 0x00, 0x00, 0x00, 0x02, 0xBC, 0x03, 0xE8,  // 总压/电流/SOC/SOH
    0x0A, 0xEB, 0x00, 0x13, 0x0A, 0xE2, 0x00, 0x09, 0x00, 0x09,  // 最大v/编号/最小v/编号/压差
    0x00, 0xB6, 0x00, 0x01, 0x00, 0xB0, 0x00, 0x02, 0x00, 0x06,  // 最大t/编号/最小t/编号/温差
    0x00, 0xB3, 0x0A, 0xE6, 0x0B, 0xB8, 0x00, 0x00, 0x00, 0x00,  // 平均t/MOS/工作/平均v
    0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 实际容量/循环/告警(0x0A)
    0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2C, 0x01, 0x2C,  // 保护/故障/系统状态
  };
  BackupBmsData d;
  ASSERT_TRUE(parse_basic_info(data.data(), static_cast<uint32_t>(data.size()), d));
  EXPECT_NEAR(d.pack_voltage, 55.82f, 1e-3f);
  EXPECT_NEAR(d.pack_current, 0.0f, 1e-3f);
  EXPECT_NEAR(d.soc, 70.0f, 1e-3f);
  EXPECT_NEAR(d.soh, 100.0f, 1e-3f);
  EXPECT_NEAR(d.max_cell_voltage, 2.795f, 1e-3f);
  EXPECT_NEAR(d.min_cell_voltage, 2.786f, 1e-3f);
  EXPECT_NEAR(d.cell_voltage_diff, 0.009f, 1e-3f);
  EXPECT_NEAR(d.max_cell_temp, 18.2f, 1e-3f);
  EXPECT_NEAR(d.min_cell_temp, 17.6f, 1e-3f);
  EXPECT_NEAR(d.temp_diff, 0.6f, 1e-3f);
  EXPECT_NEAR(d.avg_cell_temp, 17.9f, 1e-3f);
}

// 短帧(不足系统状态)应安全返回 false
TEST(BackupBmsProtocol, ParseBasicInfoShortFrameFails)
{
  const std::vector<uint8_t> data(40, 0x00);
  BackupBmsData d;
  EXPECT_FALSE(parse_basic_info(data.data(), static_cast<uint32_t>(data.size()), d));
  EXPECT_FALSE(parse_basic_info(nullptr, 0, d));
}

// 解析 0x08 单节电压: 文档示例 20 节
TEST(BackupBmsProtocol, ParseCellVoltages)
{
  std::vector<uint8_t> data;
  data.push_back(0x00);
  data.push_back(0x14);  // 20 节
  for (int i = 0; i < 20; ++i) {
    data.push_back(0x0A);
    data.push_back(0xDC);  // 0x0ADC = 2780mV = 2.78V
  }
  BackupBmsData d;
  ASSERT_TRUE(parse_cell_voltages(data.data(), static_cast<uint32_t>(data.size()), d));
  ASSERT_EQ(d.cell_voltages.size(), 20u);
  EXPECT_NEAR(d.cell_voltages[0], 2.78f, 1e-3f);
  EXPECT_NEAR(d.cell_voltages[19], 2.78f, 1e-3f);
}

TEST(BackupBmsProtocol, ParseCellVoltagesShortFails)
{
  const std::vector<uint8_t> data = {0x00, 0x14, 0x0A};  // 宣称 20 节但不足
  BackupBmsData d;
  EXPECT_FALSE(parse_cell_voltages(data.data(), static_cast<uint32_t>(data.size()), d));
}

// 解析 0x07 单节温度: 2 点 18.0℃/18.2℃ (0.1℃ 有符号)
TEST(BackupBmsProtocol, ParseCellTemps)
{
  const std::vector<uint8_t> data = {
    0x00, 0x02, 0x00, 0xB4, 0x00, 0xB6,  // count=2, 180, 182
  };
  BackupBmsData d;
  ASSERT_TRUE(parse_cell_temps(data.data(), static_cast<uint32_t>(data.size()), d));
  ASSERT_EQ(d.cell_temps.size(), 2u);
  EXPECT_NEAR(d.cell_temps[0], 18.0f, 1e-3f);
  EXPECT_NEAR(d.cell_temps[1], 18.2f, 1e-3f);
}

// 负温度: 0.1℃ 有符号补码, 0xFF38 = -200 -> -20.0℃ (协议明确支持负温度)
TEST(BackupBmsProtocol, ParseCellTempsNegative)
{
  const std::vector<uint8_t> data = {
    0x00, 0x01, 0xFF, 0x38,  // count=1, raw=-200
  };
  BackupBmsData d;
  ASSERT_TRUE(parse_cell_temps(data.data(), static_cast<uint32_t>(data.size()), d));
  ASSERT_EQ(d.cell_temps.size(), 1u);
  EXPECT_NEAR(d.cell_temps[0], -20.0f, 1e-3f);
}

// 4 个状态字(告警/保护/故障/系统)解析——safety 判据(backup_battery_judge)的关键输入
TEST(BackupBmsProtocol, ParseBasicInfoStatusWords)
{
  // 状态字区偏移: alarm@42 / protect@46 / fault@50 / system@54, 各 32 位大端
  std::vector<uint8_t> data(58, 0x00);
  data[45] = 0x0A;  // alarm_word   = 0x0000000A
  data[49] = 0x02;  // protect_word = 0x00000002
  data[53] = 0x05;  // fault_word   = 0x00000005
  data[57] = 0x01;  // system_word  = 0x00000001
  BackupBmsData d;
  ASSERT_TRUE(parse_basic_info(data.data(), static_cast<uint32_t>(data.size()), d));
  EXPECT_EQ(d.alarm_word, 0x0000000Au);
  EXPECT_EQ(d.protect_word, 0x00000002u);
  EXPECT_EQ(d.fault_word, 0x00000005u);
  EXPECT_EQ(d.system_word, 0x00000001u);
}

// 响应帧校验: 起始/地址/主机/指令/长度/CRC 均正确
TEST(BackupBmsProtocol, ParseResponseFrameValid)
{
  // 构造一个合法的 0x06 响应帧, 数据 58 字节(全 0), 再算 CRC
  const std::vector<uint8_t> payload(58, 0x00);
  std::vector<uint8_t> frame;
  frame.push_back(0x57);
  frame.push_back(kDefaultAddr);
  frame.push_back(kHostUpper);
  frame.push_back(0x00);
  frame.push_back(kCmdBasicInfo);
  frame.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint16_t c = crc16(&frame[1], static_cast<uint32_t>(frame.size() - 1));
  frame.push_back(static_cast<uint8_t>((c >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(c & 0xFF));

  const uint8_t * data = nullptr;
  uint32_t dlen = 0;
  ASSERT_TRUE(parse_response_frame(
    frame.data(), static_cast<uint32_t>(frame.size()),
    kDefaultAddr, kHostUpper, kCmdBasicInfo, &data, &dlen));
  EXPECT_EQ(dlen, payload.size());
  EXPECT_EQ(data, frame.data() + 7);
}

TEST(BackupBmsProtocol, ParseResponseFrameRejectsWrongStartOrCmd)
{
  const std::vector<uint8_t> payload(0, 0);
  std::vector<uint8_t> frame;
  frame.push_back(0x55);  // 错误起始
  frame.push_back(kDefaultAddr);
  frame.push_back(kHostUpper);
  frame.push_back(0x00);
  frame.push_back(kCmdBasicInfo);
  frame.push_back(0x00);
  frame.push_back(0x00);
  const uint16_t c = crc16(&frame[1], static_cast<uint32_t>(frame.size() - 1));
  frame.push_back(static_cast<uint8_t>((c >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(c & 0xFF));

  const uint8_t * data = nullptr;
  uint32_t dlen = 0;
  EXPECT_FALSE(parse_response_frame(
    frame.data(), static_cast<uint32_t>(frame.size()),
    kDefaultAddr, kHostUpper, kCmdBasicInfo, &data, &dlen));

  // 指令不匹配也应拒绝
  std::vector<uint8_t> frame2 = frame;
  frame2[0] = 0x57;
  frame2[4] = kCmdCellTemp;
  // 需要重算 CRC(改了 cmd)
  frame2.resize(7);
  const uint16_t c2 = crc16(&frame2[1], static_cast<uint32_t>(frame2.size() - 1));
  frame2.push_back(static_cast<uint8_t>((c2 >> 8) & 0xFF));
  frame2.push_back(static_cast<uint8_t>(c2 & 0xFF));
  EXPECT_FALSE(parse_response_frame(
    frame2.data(), static_cast<uint32_t>(frame2.size()),
    kDefaultAddr, kHostUpper, kCmdBasicInfo, &data, &dlen));
}

// 响应帧校验: CRC 错误/地址不匹配/主机不匹配/短帧 均应拒绝
// (串口残留帧/噪声帧误解析防线, 2026-08-26 补齐分支覆盖)
TEST(BackupBmsProtocol, ParseResponseFrameRejectsBadCrcAddrHostAndShort)
{
  const std::vector<uint8_t> payload(4, 0x00);
  std::vector<uint8_t> frame;
  frame.push_back(0x57);
  frame.push_back(kDefaultAddr);
  frame.push_back(kHostUpper);
  frame.push_back(0x00);
  frame.push_back(kCmdCellVoltage);
  frame.push_back(0x00);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint16_t c = crc16(&frame[1], static_cast<uint32_t>(frame.size() - 1));
  frame.push_back(static_cast<uint8_t>((c >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(c & 0xFF));

  const uint8_t * data = nullptr;
  uint32_t dlen = 0;
  // 1) CRC 破坏
  std::vector<uint8_t> bad_crc = frame;
  bad_crc[bad_crc.size() - 1] ^= 0xFF;
  EXPECT_FALSE(parse_response_frame(
    bad_crc.data(), static_cast<uint32_t>(bad_crc.size()),
    kDefaultAddr, kHostUpper, kCmdCellVoltage, &data, &dlen));

  // 2) 期望地址与帧地址不匹配(帧本身合法)
  EXPECT_FALSE(parse_response_frame(
    frame.data(), static_cast<uint32_t>(frame.size()),
    static_cast<uint8_t>(kDefaultAddr - 1), kHostUpper, kCmdCellVoltage, &data, &dlen));

  // 3) 期望主机与帧主机不匹配
  EXPECT_FALSE(parse_response_frame(
    frame.data(), static_cast<uint32_t>(frame.size()),
    kDefaultAddr, static_cast<uint8_t>(kHostUpper + 1), kCmdCellVoltage, &data, &dlen));

  // 4) 短帧(不足 7+4+2)
  EXPECT_FALSE(parse_response_frame(
    frame.data(), static_cast<uint32_t>(frame.size() - 1),
    kDefaultAddr, kHostUpper, kCmdCellVoltage, &data, &dlen));
}
