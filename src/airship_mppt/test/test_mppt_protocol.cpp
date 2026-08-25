// 灵云01号伴飞电脑 — MPPT 协议解析单元测试
#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "airship_mppt/mppt_protocol.hpp"

using airship_mppt::MpptData;
using airship_mppt::ReadCode;

// 构造 8 字节小端数据帧
static std::array<uint8_t, 8> make_frame(
  uint16_t w0, uint16_t w1, uint16_t w2, uint16_t w3)
{
  std::array<uint8_t, 8> d{};
  d[0] = static_cast<uint8_t>(w0 & 0xFF);
  d[1] = static_cast<uint8_t>((w0 >> 8) & 0xFF);
  d[2] = static_cast<uint8_t>(w1 & 0xFF);
  d[3] = static_cast<uint8_t>((w1 >> 8) & 0xFF);
  d[4] = static_cast<uint8_t>(w2 & 0xFF);
  d[5] = static_cast<uint8_t>((w2 >> 8) & 0xFF);
  d[6] = static_cast<uint8_t>(w3 & 0xFF);
  d[7] = static_cast<uint8_t>((w3 >> 8) & 0xFF);
  return d;
}

TEST(MpptProtocol, QueryFrameId)
{
  // 主机发送排布: 0x14[code][设备地址][0xA1] (目标标志 A1 在末尾)
  // 与从机回应 0x14[code]A1[dev] (A1 在设备地址之前) 非对称, 见勘误文档。
  const auto frame = airship_mppt::build_query_frame(ReadCode::kCodeRealtime, 0x01);
  EXPECT_EQ(frame.id, 0x140301A1U);
  EXPECT_TRUE(frame.extended);
  EXPECT_EQ(frame.len, 8U);
}

// 副 MPPT(设备地址 02, 双机部署形态): 查询帧 0x14 03 02 A1
TEST(MpptProtocol, QueryFrameIdSecondaryDevice)
{
  const auto frame = airship_mppt::build_query_frame(ReadCode::kCodeRealtime, 0x02);
  EXPECT_EQ(frame.id, 0x140302A1U);
  // 额定参数段(code=0x02) 副 MPPT: 0x14 02 02 A1
  const auto frame2 = airship_mppt::build_query_frame(ReadCode::kCodeRated, 0x02);
  EXPECT_EQ(frame2.id, 0x140202A1U);
}

TEST(MpptProtocol, ParseRealtime)
{
  // 光伏电压 3200 -> 320.0V, 电池电压 2600 -> 260.0V, 充电电流 100 -> 10.0A
  const auto d = make_frame(3200, 2600, 100, 0);
  MpptData out;
  airship_mppt::parse_realtime(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.pv_voltage, 320.0f);
  EXPECT_FLOAT_EQ(out.battery_voltage, 260.0f);
  EXPECT_FLOAT_EQ(out.charge_current, 10.0f);
}

TEST(MpptProtocol, ParseRated)
{
  // 电池额定电压 2600 -> 260.0V, 充电额定电流 60 -> 60A (分辨率 1A)
  const auto d = make_frame(0, 2600, 60, 0);
  MpptData out;
  airship_mppt::parse_rated(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.rated_voltage, 260.0f);
  EXPECT_FLOAT_EQ(out.rated_current, 60.0f);
}

TEST(MpptProtocol, ParseState)
{
  // 快充阶段(1) + 故障: 电池欠压|过温 (0x0001|0x0020=0x0021)
  const auto d = make_frame(1, 0, 0x0021, 0);
  MpptData out;
  airship_mppt::parse_state(d.data(), 8, out);
  EXPECT_EQ(out.charge_state, airship_mppt::kChargeBoost);
  EXPECT_EQ(out.fault_state, airship_mppt::fault::kBatteryUndervolt |
    airship_mppt::fault::kOvertemp);
}

TEST(MpptProtocol, ParseEnergyDay)
{
  // 日发电量 12345 -> 1234.5 kWh, 月发电量 45678 -> 4567.8 kWh
  std::array<uint8_t, 8> d{};
  d[0] = 0x39; d[1] = 0x30; d[2] = 0x00; d[3] = 0x00;  // 12345
  d[4] = 0x6E; d[5] = 0xB2; d[6] = 0x00; d[7] = 0x00;  // 45678
  MpptData out;
  airship_mppt::parse_energy_day(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.energy_today, 1234.5f);
  EXPECT_FLOAT_EQ(out.energy_month, 4567.8f);
}

TEST(MpptProtocol, ParseEnergyTotal)
{
  // 总发电量 200000 -> 20000.0 kWh
  std::array<uint8_t, 8> d{};
  d[0] = 0x40; d[1] = 0x0D; d[2] = 0x03; d[3] = 0x00;  // 200000
  MpptData out;
  airship_mppt::parse_energy_total(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.energy_total, 20000.0f);
}

TEST(MpptProtocol, ParseTemp)
{
  // 空气温度 320 -> 32.0℃, 模块温度 -50 -> -5.0℃ (S16 补码: 0xFFCE)
  const auto d = make_frame(0, static_cast<uint16_t>(320), static_cast<uint16_t>(0xFFCE), 0);
  MpptData out;
  airship_mppt::parse_temp(d.data(), 8, out);
  EXPECT_FLOAT_EQ(out.air_temp, 32.0f);
  EXPECT_FLOAT_EQ(out.module_temp, -5.0f);
}

TEST(MpptProtocol, ParseControl)
{
  // 控制模式 EMS-CAN(2), 充电开关 FF00H -> 开
  const auto d = make_frame(0, 2, 0xFF00, 0);
  MpptData out;
  airship_mppt::parse_control(d.data(), 8, out);
  EXPECT_EQ(out.control_mode, airship_mppt::kModeEmsCan);
  EXPECT_TRUE(out.charging_enabled);
}

TEST(MpptProtocol, ParseControlChargingOff)
{
  // 充电开关 0000H -> 关
  const auto d = make_frame(0, 0, 0x0000, 0);
  MpptData out;
  airship_mppt::parse_control(d.data(), 8, out);
  EXPECT_FALSE(out.charging_enabled);
}

// 帧长不足时应安全返回, 不产生越界或假数据
TEST(MpptProtocol, ShortFrameIgnored)
{
  const std::array<uint8_t, 4> d = {0x00, 0x00, 0x00, 0x00};
  MpptData out;
  airship_mppt::parse_realtime(d.data(), 4, out);
  airship_mppt::parse_rated(d.data(), 4, out);
  airship_mppt::parse_state(d.data(), 4, out);
  airship_mppt::parse_energy_day(d.data(), 4, out);
  airship_mppt::parse_energy_total(d.data(), 4, out);
  airship_mppt::parse_temp(d.data(), 4, out);
  airship_mppt::parse_control(d.data(), 4, out);
  EXPECT_EQ(out.pv_voltage, 0.0f);
  EXPECT_EQ(out.charge_state, 0);
  EXPECT_EQ(out.energy_today, 0.0f);
}

// ===== 回应帧 ID 匹配 (match_response_id, 从 receive_loop 提炼的纯函数) =====
TEST(MpptProtocol, MatchResponseId)
{
  // 01 号机实时段回应 0x1403A101 -> kCodeRealtime
  auto c = airship_mppt::match_response_id(0x1403A101U, 1);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(*c, ReadCode::kCodeRealtime);
  // 副 MPPT(02 号机)回应帧为其设备地址 -> 同样匹配
  auto c2 = airship_mppt::match_response_id(0x1403A102U, 2);
  ASSERT_TRUE(c2.has_value());
  EXPECT_EQ(*c2, ReadCode::kCodeRealtime);
}

TEST(MpptProtocol, MatchResponseIdRejects)
{
  // 设备地址不匹配: 期望 02, 帧实为 01 号机回应 -> 忽略(防串联误解析)
  EXPECT_FALSE(airship_mppt::match_response_id(0x1403A101U, 2).has_value());
  // type 非 0x14 (配置帧 0x13)
  EXPECT_FALSE(airship_mppt::match_response_id(0x1303A101U, 1).has_value());
  // target 非 0xA1
  EXPECT_FALSE(airship_mppt::match_response_id(0x14030001U, 1).has_value());
  // 未知/保留 code (0xF0)
  EXPECT_FALSE(airship_mppt::match_response_id(0x14F0A101U, 1).has_value());
}
