// 灵云01号伴飞电脑 — 飞控告警规则引擎纯逻辑单元测试
#include <gtest/gtest.h>

#include "airship_fc/fc_alert_logic.hpp"

using fc_logic::Action;
using fc_logic::AlertState;
using fc_logic::Decision;
using fc_logic::Threshold;

// ===== rule_level: 阈值分级判定 =====
TEST(FcAlertLogic, RuleLevelUpperSide)
{
  Threshold t{10.0f, 20.0f, 30.0f, true};
  EXPECT_EQ(fc_logic::rule_level(t, 5.0f), fc_logic::level::kInfo);
  EXPECT_EQ(fc_logic::rule_level(t, 15.0f), fc_logic::level::kWarning);
  EXPECT_EQ(fc_logic::rule_level(t, 25.0f), fc_logic::level::kCritical);
  EXPECT_EQ(fc_logic::rule_level(t, 35.0f), fc_logic::level::kEmergency);
  // 精确等于阈值: 用严格 >, 恰等于 warning 不告警
  EXPECT_EQ(fc_logic::rule_level(t, 10.0f), fc_logic::level::kInfo);
}

TEST(FcAlertLogic, RuleLevelLowerSide)
{
  Threshold t{10.0f, 5.0f, 1.0f, false};  // <warning 告警, warning/critical/emergency 递减
  EXPECT_EQ(fc_logic::rule_level(t, 20.0f), fc_logic::level::kInfo);
  EXPECT_EQ(fc_logic::rule_level(t, 8.0f), fc_logic::level::kWarning);
  EXPECT_EQ(fc_logic::rule_level(t, 3.0f), fc_logic::level::kCritical);
  EXPECT_EQ(fc_logic::rule_level(t, 0.5f), fc_logic::level::kEmergency);
}

// ===== normalize_battery_remaining: 归一化到 0~100 =====
TEST(FcAlertLogic, NormalizeBatteryFracToPercent)
{
  EXPECT_FLOAT_EQ(fc_logic::normalize_battery_remaining(0.85f), 85.0f);
  EXPECT_FLOAT_EQ(fc_logic::normalize_battery_remaining(1.0f), 100.0f);
  EXPECT_FLOAT_EQ(fc_logic::normalize_battery_remaining(0.5f), 50.0f);
}

TEST(FcAlertLogic, NormalizeBatteryPercentPassthrough)
{
  // 已按 0~100 上报(0 或 >1)时原样透传
  EXPECT_FLOAT_EQ(fc_logic::normalize_battery_remaining(85.0f), 85.0f);
  EXPECT_FLOAT_EQ(fc_logic::normalize_battery_remaining(0.0f), 0.0f);
}

// ===== update_alert: 去抖状态机 =====
TEST(FcAlertLogic, UpdateAlertDebounceGating)
{
  Threshold t{10.0f, 20.0f, 30.0f, true};
  AlertState st;
  // 连续越界不足 debounce_n(3) 不触发
  EXPECT_EQ(fc_logic::update_alert(t, 15.0f, st, 3).action, Action::kNone);
  EXPECT_EQ(fc_logic::update_alert(t, 15.0f, st, 3).action, Action::kNone);
  // 第 3 次达阈值触发
  const auto d = fc_logic::update_alert(t, 15.0f, st, 3);
  EXPECT_EQ(d.action, Action::kActivate);
  EXPECT_EQ(d.level, fc_logic::level::kWarning);
  EXPECT_EQ(st.current_level, fc_logic::level::kWarning);
  // 同一等级持续越界不再重复触发(去重)
  EXPECT_EQ(fc_logic::update_alert(t, 16.0f, st, 3).action, Action::kNone);
}

TEST(FcAlertLogic, UpdateAlertEscalation)
{
  Threshold t{10.0f, 20.0f, 30.0f, true};
  AlertState st;
  // 稳定 warning
  (void)fc_logic::update_alert(t, 15.0f, st, 1);
  EXPECT_EQ(st.current_level, fc_logic::level::kWarning);
  // 升级到 emergency -> 触发
  const auto d = fc_logic::update_alert(t, 35.0f, st, 1);
  EXPECT_EQ(d.action, Action::kActivate);
  EXPECT_EQ(d.level, fc_logic::level::kEmergency);
}

TEST(FcAlertLogic, UpdateAlertClear)
{
  // lower_side: warning=30(值跌破 30 开始告警), critical=20, emergency=10(最危急)
  Threshold t{30.0f, 20.0f, 10.0f, false};
  AlertState st;
  // 值跌破 emergency(值 5 < 10) -> 进入 emergency 告警
  (void)fc_logic::update_alert(t, 5.0f, st, 1);
  EXPECT_EQ(st.current_level, fc_logic::level::kEmergency);
  // 恢复到 warning 之上(值 35: 35>=30) -> Clear
  const auto d = fc_logic::update_alert(t, 35.0f, st, 1);
  EXPECT_EQ(d.action, Action::kClear);
  EXPECT_EQ(st.current_level, fc_logic::level::kInfo);
  // 已正常再正常, 无动作
  EXPECT_EQ(fc_logic::update_alert(t, 35.0f, st, 1).action, Action::kNone);
}

TEST(FcAlertLogic, UpdateAlertStatePerRule)
{
  // 多条规则各自独立的去抖状态互不影响
  Threshold ta{10.0f, 20.0f, 30.0f, true};
  Threshold tb{100.0f, 200.0f, 300.0f, true};  // 独立的不同阈值
  AlertState sa;
  AlertState sb;
  // 各累积 1 次(未达 debounce=2), 均不触发
  (void)fc_logic::update_alert(ta, 15.0f, sa, 2);
  (void)fc_logic::update_alert(tb, 150.0f, sb, 2);
  EXPECT_EQ(sa.current_level, fc_logic::level::kInfo);
  EXPECT_EQ(sb.current_level, fc_logic::level::kInfo);
  // 各自第 2 次达阈值, 独立触发(无共享状态)
  EXPECT_EQ(fc_logic::update_alert(ta, 15.0f, sa, 2).action, Action::kActivate);
  EXPECT_EQ(fc_logic::update_alert(tb, 150.0f, sb, 2).action, Action::kActivate);
}
