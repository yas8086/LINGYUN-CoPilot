// 灵云01号伴飞电脑 — 安全仲裁纯逻辑单元测试
#include <string>

#include <gtest/gtest.h>

#include "airship_safety/safety_logic.hpp"

using airship_safety::aggregate;
using airship_safety::backup_battery_judge;
using airship_safety::backup_battery_reason;
using airship_safety::battery_judge;
using airship_safety::battery_reason;
using airship_safety::dcdc_judge;
using airship_safety::dcdc_reason;
using airship_safety::kDcdcFaultMask;
using airship_safety::kDcdcOutputStatusBit;

// ===== DCDC 判据 =====
TEST(SafetyLogic, DcdcJudgeOnlineNoFault)
{
  // 在线且无故障 => 安全
  EXPECT_TRUE(dcdc_judge(true, 0x00));
}

TEST(SafetyLogic, DcdcJudgeOnlineFault)
{
  // 在线但有故障位 => 不安全
  EXPECT_FALSE(dcdc_judge(true, 0x80));  // BIT7 总故障
}

TEST(SafetyLogic, DcdcJudgeOffline)
{
  // 离线 => 不安全 (无论故障字)
  EXPECT_FALSE(dcdc_judge(false, 0x00));
  EXPECT_FALSE(dcdc_judge(false, 0x80));
}

TEST(SafetyLogic, DcdcJudgeOutputStatusBitExcluded)
{
  // BIT2 是输出状态位(非故障), 不该导致不安全
  EXPECT_TRUE(dcdc_judge(true, kDcdcOutputStatusBit));  // 0x04
}

TEST(SafetyLogic, DcdcJudgeOutputStatusPlusFault)
{
  // 输出状态位 + 真实故障位 => 仍应判不安全
  EXPECT_FALSE(dcdc_judge(true, kDcdcOutputStatusBit | 0x01));  // 0x05
}

// ===== 各故障位全覆盖 =====
TEST(SafetyLogic, DcdcJudgeAllFaultBits)
{
  // 遍历除输出状态位外的所有单个故障位, 均应判不安全
  const uint8_t fault_bits[] = {0x01, 0x02, 0x08, 0x10, 0x20, 0x80};
  for (const uint8_t bit : fault_bits) {
    EXPECT_FALSE(dcdc_judge(true, bit))
      << "故障位 0x" << std::hex << static_cast<int>(bit) << " 应判不安全";
  }
}

TEST(SafetyLogic, DcdcJudgeAllBitsSet)
{
  // 全部位都置1 (0xFF): 含输出状态位, 但仍有故障位 => 不安全
  EXPECT_FALSE(dcdc_judge(true, 0xFF));
}

TEST(SafetyLogic, DcdcJudgeMultipleFaults)
{
  // 多个故障位同时置位 => 不安全
  EXPECT_FALSE(dcdc_judge(true, 0x01 | 0x80 | 0x20));  // 欠压+总故障+短路
}

// ===== 主 BMS 判据 =====
TEST(SafetyLogic, BatteryJudgeOnlineOk)
{
  // 在线 且 总压达下限 且 无告警 => 安全
  EXPECT_TRUE(battery_judge(true, 300.0f, 0, 280.0f));
}

TEST(SafetyLogic, BatteryJudgeOffline)
{
  // 离线 => 不安全(无论电压/告警级), fail-safe
  EXPECT_FALSE(battery_judge(false, 300.0f, 0, 280.0f));
  EXPECT_FALSE(battery_judge(false, 0.0f, 2, 280.0f));
}

TEST(SafetyLogic, BatteryJudgeLowVoltage)
{
  // 总压低于下限 => 不安全
  EXPECT_FALSE(battery_judge(true, 279.9f, 0, 280.0f));
}

TEST(SafetyLogic, BatteryJudgeAlarmLevel)
{
  // 告警级 1(故障)/2(严重) => 不安全(即使电压正常)
  EXPECT_FALSE(battery_judge(true, 300.0f, 1, 280.0f));
  EXPECT_FALSE(battery_judge(true, 300.0f, 2, 280.0f));
}

// ===== 12S 备用电源判据 =====
TEST(SafetyLogic, BackupBatteryJudgeOnlineNoFault)
{
  // 在线 且 总压不低于下限 且 无故障 => 安全
  EXPECT_TRUE(backup_battery_judge(true, 40.0f, 0x00, 24.0f));
}

TEST(SafetyLogic, BackupBatteryJudgeOffline)
{
  // 离线 => 不安全 (无论电压/故障字)
  EXPECT_FALSE(backup_battery_judge(false, 40.0f, 0x00, 24.0f));
}

TEST(SafetyLogic, BackupBatteryJudgeLowVoltage)
{
  // 电压低于下限 => 不安全
  EXPECT_FALSE(backup_battery_judge(true, 20.0f, 0x00, 24.0f));
}

TEST(SafetyLogic, BackupBatteryJudgeFault)
{
  // 存在故障位 => 不安全 (即使电压正常)
  EXPECT_FALSE(backup_battery_judge(true, 40.0f, 0x01, 24.0f));
  EXPECT_FALSE(backup_battery_judge(true, 40.0f, 0x80000000, 24.0f));
}

// ===== 聚合判定: 组合穷举 =====
TEST(SafetyLogic, AggregateAllOk)
{
  const auto d = aggregate(true, true, true);
  EXPECT_TRUE(d.safe_to_control);
  EXPECT_TRUE(d.dcdc_ok);
  EXPECT_TRUE(d.backup_battery_ok);
  EXPECT_TRUE(d.reason.empty());
}

TEST(SafetyLogic, AggregateDcdcNotOk)
{
  const auto d = aggregate(false, true, true);
  EXPECT_FALSE(d.safe_to_control);
  EXPECT_FALSE(d.dcdc_ok);
  EXPECT_EQ(d.reason, dcdc_reason());
}

TEST(SafetyLogic, AggregateBatteryNotOk)
{
  // 主 BMS 判据失败时, reason 应描述主 BMS 而非 DCDC (回归: 曾错误地设为 DCDC 原因)
  const auto d = aggregate(true, false, true);
  EXPECT_FALSE(d.safe_to_control);
  EXPECT_TRUE(d.dcdc_ok);
  EXPECT_EQ(d.reason, battery_reason());
  EXPECT_NE(d.reason, dcdc_reason());
}

TEST(SafetyLogic, AggregateBackupBatteryNotOk)
{
  // 备用电源判据失败时, reason 应描述备用电源
  const auto d = aggregate(true, true, false);
  EXPECT_FALSE(d.safe_to_control);
  EXPECT_TRUE(d.dcdc_ok);
  EXPECT_FALSE(d.backup_battery_ok);
  EXPECT_EQ(d.reason, backup_battery_reason());
  EXPECT_NE(d.reason, dcdc_reason());
  EXPECT_NE(d.reason, battery_reason());
}

TEST(SafetyLogic, AggregateBothNotOk)
{
  // 双判据均失败 => 不控制; reason 优先描述 DCDC
  const auto d = aggregate(false, false, true);
  EXPECT_FALSE(d.safe_to_control);
  EXPECT_FALSE(d.dcdc_ok);
  EXPECT_EQ(d.reason, dcdc_reason());
}

TEST(SafetyLogic, AggregateBackupOverridesBattery)
{
  // 主 BMS 与备用电源均失败 => reason 优先主 BMS
  const auto d = aggregate(true, false, false);
  EXPECT_FALSE(d.safe_to_control);
  EXPECT_EQ(d.reason, battery_reason());
}

TEST(SafetyLogic, DcdcFaultMask)
{
  // 掩码应排除 BIT2(输出状态), 保留其余故障位
  EXPECT_EQ(kDcdcFaultMask & kDcdcOutputStatusBit, 0);
  EXPECT_EQ(kDcdcFaultMask & 0x80, 0x80);
  EXPECT_EQ(kDcdcFaultMask & 0x01, 0x01);
  EXPECT_EQ(kDcdcFaultMask & 0x02, 0x02);
  EXPECT_EQ(kDcdcFaultMask & 0x08, 0x08);
  EXPECT_EQ(kDcdcFaultMask & 0x10, 0x10);
  EXPECT_EQ(kDcdcFaultMask & 0x20, 0x20);
}
