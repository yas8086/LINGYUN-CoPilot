// 灵云01号伴飞电脑 — 离线兜底发布时机判定纯函数单元测试
#include <gtest/gtest.h>

#include "airship_utils/offline_utils.hpp"

using airship_utils::should_publish_offline;

// 未超时(距上次有效数据 < timeout): 不发布
TEST(OfflineUtils, NotTimedOut)
{
  EXPECT_FALSE(should_publish_offline(1.0, 0.0, 0.0, 3.0));
}

// 首次超时(last_offline_pub 距 now 也 ≥ timeout): 发布
TEST(OfflineUtils, TimedOutFirstTime)
{
  EXPECT_TRUE(should_publish_offline(5.0, 0.0, 0.0, 3.0));
}

// 超时但距上次 offline 发布不足一个 timeout: 不重复发布(兜底节流)
TEST(OfflineUtils, PubGatedBeforeNextPeriod)
{
  // now=5, last_pub=3(距 2s < 3s timeout), last_data=0 -> 距数据超时但距发布不足, 不发布
  EXPECT_FALSE(should_publish_offline(5.0, 0.0, 3.0, 3.0));
  // now=6, last_pub=3(距 3s == timeout) -> 允许再次发布
  EXPECT_TRUE(should_publish_offline(6.0, 0.0, 3.0, 3.0));
}

// 数据仍新鲜但距上次发布很久: 不发布(fresh data 意味着在线)
TEST(OfflineUtils, FreshDataNeverOffline)
{
  EXPECT_FALSE(should_publish_offline(100.0, 100.0, 0.0, 3.0));
}

// 边界: now-last_data 恰等于 timeout 时不视为失联(严格 >)
TEST(OfflineUtils, DataAgeEqualsTimeoutBoundary)
{
  EXPECT_FALSE(should_publish_offline(3.0, 0.0, 0.0, 3.0));
  EXPECT_TRUE(should_publish_offline(3.0 + 1e-6, 0.0, 0.0, 3.0));
}
