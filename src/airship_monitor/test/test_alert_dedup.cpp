// 灵云01号伴飞电脑 — 告警去重状态机单元测试
#include <gtest/gtest.h>

#include "airship_monitor/alert_dedup.hpp"

using airship_monitor::update_online;

TEST(AlertDedup, FirstOfflineRaises)
{
  // 初始 last=false, 首次判定离线(false) => 无变化, 不应重复告警
  bool last = false;
  auto t = update_online(false, &last);
  EXPECT_FALSE(t.changed);
  EXPECT_FALSE(t.now_online);
  EXPECT_FALSE(last);
}

TEST(AlertDedup, OnlineTransitionRaises)
{
  // 从离线(false)转在线(true) => 跳变, 应发"恢复"告警
  bool last = false;
  auto t = update_online(true, &last);
  EXPECT_TRUE(t.changed);
  EXPECT_TRUE(t.now_online);
  EXPECT_TRUE(last);
}

TEST(AlertDedup, OfflineTransitionRaises)
{
  // 从在线(true)转离线(false) => 跳变, 应发"离线"告警
  bool last = true;
  auto t = update_online(false, &last);
  EXPECT_TRUE(t.changed);
  EXPECT_FALSE(t.now_online);
  EXPECT_FALSE(last);
}

TEST(AlertDedup, NoRepeatWhileOffline)
{
  // 持续离线期间, 每次调用不应重复触发 (去重核心)
  bool last = false;
  update_online(false, &last);  // 首次 (无变化, 但因初始即离线)
  for (int i = 0; i < 100; ++i) {
    auto t = update_online(false, &last);
    EXPECT_FALSE(t.changed);
  }
}

TEST(AlertDedup, NoRepeatWhileOnline)
{
  // 持续在线期间, 不应重复触发
  bool last = false;
  update_online(true, &last);  // 首次跳变
  for (int i = 0; i < 100; ++i) {
    auto t = update_online(true, &last);
    EXPECT_FALSE(t.changed);
  }
}

TEST(AlertDedup, ToggleBackAndForth)
{
  // 反复跳变: 每次跳变都应触发一次
  bool last = false;
  EXPECT_TRUE(update_online(true, &last).changed);   // 0->1
  EXPECT_FALSE(update_online(true, &last).changed);  // 稳定
  EXPECT_TRUE(update_online(false, &last).changed);  // 1->0
  EXPECT_FALSE(update_online(false, &last).changed); // 稳定
  EXPECT_TRUE(update_online(true, &last).changed);   // 0->1
}

TEST(AlertDedup, NullLastOnlineNoCrash)
{
  // 防御: last_online 为 nullptr 时不应崩溃, 无状态变化
  auto t0 = update_online(false, nullptr);
  EXPECT_FALSE(t0.changed);
  EXPECT_FALSE(t0.now_online);

  auto t1 = update_online(true, nullptr);
  EXPECT_FALSE(t1.changed);
  EXPECT_TRUE(t1.now_online);
}
