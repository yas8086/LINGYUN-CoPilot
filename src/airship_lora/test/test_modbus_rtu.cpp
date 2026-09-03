// 灵云01号伴飞电脑 — LoRa Modbus RTU 协议解析单元测试
#include <gtest/gtest.h>

#include <vector>

#include "airship_lora/modbus_rtu.hpp"

using airship_lora::build_read_request;
using airship_lora::check_crc;
using airship_lora::check_temp_alarm;
using airship_lora::modbus_crc16;
using airship_lora::parse_pressure_response;
using airship_lora::parse_temp_response;

namespace
{

// 为给定字节序列(不含 CRC)附加 CRC 低/高字节
std::vector<uint8_t> with_crc(const std::vector<uint8_t> & body)
{
  std::vector<uint8_t> f = body;
  const uint16_t crc = modbus_crc16(f.data(), f.size());
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return f;
}

}  // namespace

TEST(ModbusRtuTest, CrcKnownVector)
{
  // 标准 MODBUS CRC 校验向量: 0x01 0x03 0x00 0x00 0x00 0x0A -> 0xCDC5
  const uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
  EXPECT_EQ(modbus_crc16(data, sizeof(data)), 0xCDC5u);
}

TEST(ModbusRtuTest, BuildReadRequest)
{
  // 从机1, 温度寄存器 0x76C1, 数量1 -> [01 04 76 C1 00 01 crcLo crcHi]
  const auto f = build_read_request(1, 0x76C1, 1);
  ASSERT_EQ(f.size(), 8u);
  EXPECT_EQ(f[0], 0x01);
  EXPECT_EQ(f[1], 0x04);
  EXPECT_EQ(f[2], 0x76);
  EXPECT_EQ(f[3], 0xC1);
  EXPECT_EQ(f[4], 0x00);
  EXPECT_EQ(f[5], 0x01);
  // 自身 CRC 校验应通过
  EXPECT_TRUE(check_crc(f));
}

TEST(ModbusRtuTest, ParseTemperature)
{
  // raw=0x00BF(191) -> 19.1℃
  const auto resp = with_crc({0x01, 0x04, 0x02, 0x00, 0xBF});
  float temp = 0.0f;
  int raw = 0;
  ASSERT_TRUE(parse_temp_response(resp, 1, temp, raw));
  EXPECT_EQ(raw, 191);
  EXPECT_FLOAT_EQ(temp, 19.1f);
}

TEST(ModbusRtuTest, ParseTemperatureNegative)
{
  // raw=0xFF60(65376 补码=-160) -> -16.0℃
  const auto resp = with_crc({0x01, 0x04, 0x02, 0xFF, 0x60});
  float temp = 0.0f;
  int raw = 0;
  ASSERT_TRUE(parse_temp_response(resp, 1, temp, raw));
  EXPECT_EQ(raw, -160);
  EXPECT_FLOAT_EQ(temp, -16.0f);
}

TEST(ModbusRtuTest, ParsePressure)
{
  // high=0x000F, low=0x4240 -> 0x000F4240 = 1000000 Pa
  const auto resp = with_crc({0x01, 0x04, 0x04, 0x00, 0x0F, 0x42, 0x40});
  double pa = 0.0;
  ASSERT_TRUE(parse_pressure_response(resp, 1, pa));
  EXPECT_DOUBLE_EQ(pa, 1000000.0);
}

TEST(ModbusRtuTest, ParsePressureNegative)
{
  // 压差为有符号 32 位: 0xFFFFFFEF = -17 Pa (实测 15 号节点)
  const auto resp = with_crc({0x01, 0x04, 0x04, 0xFF, 0xFF, 0xFF, 0xEF});
  double pa = 0.0;
  ASSERT_TRUE(parse_pressure_response(resp, 1, pa));
  EXPECT_DOUBLE_EQ(pa, -17.0);
}

TEST(ModbusRtuTest, ParseRejectWrongSlaveAddr)
{
  // 从机地址不匹配(期望 1, 实际 2) -> 拒绝, 防 485 串扰
  const auto resp = with_crc({0x02, 0x04, 0x02, 0x00, 0xBF});
  float temp = 0.0f;
  int raw = 0;
  EXPECT_FALSE(parse_temp_response(resp, 1, temp, raw));
}

TEST(ModbusRtuTest, ParseRejectBadCrc)
{
  auto resp = with_crc({0x01, 0x04, 0x02, 0x00, 0xBF});
  resp.back() ^= 0xFF;  // 破坏 CRC
  float temp = 0.0f;
  int raw = 0;
  EXPECT_FALSE(parse_temp_response(resp, 1, temp, raw));
}

TEST(ModbusRtuTest, ParseRejectWrongLength)
{
  // 长度错误
  const auto resp = with_crc({0x01, 0x04, 0x02, 0x00});
  float temp = 0.0f;
  int raw = 0;
  EXPECT_FALSE(parse_temp_response(resp, 1, temp, raw));
}

TEST(ModbusRtuTest, TempAlarm)
{
  EXPECT_EQ(check_temp_alarm(25.0f, -10.0f, 60.0f), 0);
  EXPECT_EQ(check_temp_alarm(70.0f, -10.0f, 60.0f), 1);
  EXPECT_EQ(check_temp_alarm(-20.0f, -10.0f, 60.0f), -1);
}
