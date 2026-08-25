// 灵云01号伴飞电脑 — MQTT 客户端最小单元测试
// 覆盖连接前状态与生命周期安全(lib_init/cleanup 配对, 未连接 publish 语义)。
#include <gtest/gtest.h>

#include "airship_cloud/mqtt_client.hpp"

// 未 connect 的客户端: is_connected 恒 false, publish 应失败(而非假装入队)
TEST(MqttClient, NotConnectedPublishFails)
{
  airship_cloud::MqttClient c("127.0.0.1", 1, "test_client");
  EXPECT_FALSE(c.is_connected());
  EXPECT_FALSE(c.publish("test/topic", "payload"));
  // 析构不崩溃(lib_inited_ 为 false 时跳过 cleanup, 配对逻辑正确)
}

// 构造后立即析构: 未 init 库也安全(曾依赖 lib_inited_ 守卫)
TEST(MqttClient, DtorWithoutConnectSafe)
{
  {airship_cloud::MqttClient c("192.0.2.1", 8883, "d");}
  SUCCEED();
}
