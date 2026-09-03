// 气囊压差桥接核心逻辑单测(header-only 纯逻辑, 无 rclcpp 依赖)
// 语义契约见 bladder_bridge_logic.hpp 与 px4_msgs/AirshipBladderPressure.msg 注释
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "airship_fc/bladder_bridge_logic.hpp"

using bladder_bridge::BladderBridgeState;
using bladder_bridge::SampleInput;
using bladder_bridge::kNumBladders;

namespace
{

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

// 构造一路有效压力样本
SampleInput make_press_sample(int32_t node_id, double pa, float temp = 26.0f)
{
  SampleInput s;
  s.node_id = node_id;
  s.pressure_pa = pa;
  s.temp_c = temp;
  s.online = 1;
  s.stale = 0;
  s.temp_valid = 1;
  s.press_valid = 1;
  return s;
}

// 构造默认映射 [6,13,14,15] 的 4 节点完整有效轮
std::vector<SampleInput> make_full_round(double pa0, double pa1, double pa2, double pa3)
{
  return {
    make_press_sample(6, pa0), make_press_sample(13, pa1),
    make_press_sample(14, pa2), make_press_sample(15, pa3)};
}

}  // namespace

TEST(BladderBridgeTest, InitialSlotsAreNaN)
{
  BladderBridgeState st({6, 13, 14, 15});
  const auto out = st.update({});
  for (std::size_t i = 0; i < kNumBladders; ++i) {
    EXPECT_TRUE(std::isnan(out[i].pressure_pa));
    EXPECT_TRUE(std::isnan(out[i].temp_c));
    EXPECT_FALSE(out[i].valid);
    EXPECT_FALSE(out[i].stale);
  }
}

TEST(BladderBridgeTest, NormalRoundMapsFixedSlots)
{
  BladderBridgeState st({6, 13, 14, 15});
  const auto out = st.update(make_full_round(10.0, 20.0, 30.0, 40.0));
  // 槽位顺序固定: 6->槽0(左副囊), 13->槽1(左主囊), 14->槽2(右主囊), 15->槽3(右副囊)
  EXPECT_FLOAT_EQ(out[0].pressure_pa, 10.0f);
  EXPECT_FLOAT_EQ(out[1].pressure_pa, 20.0f);
  EXPECT_FLOAT_EQ(out[2].pressure_pa, 30.0f);
  EXPECT_FLOAT_EQ(out[3].pressure_pa, 40.0f);
  for (std::size_t i = 0; i < kNumBladders; ++i) {
    EXPECT_TRUE(out[i].valid);
    EXPECT_FALSE(out[i].stale);
  }
}

TEST(BladderBridgeTest, OutOfOrderInputStillMapsCorrectly)
{
  BladderBridgeState st({6, 13, 14, 15});
  // 乱序输入: 15/6/14/13
  const auto round = std::vector<SampleInput>{
    make_press_sample(15, 44.0), make_press_sample(6, 11.0),
    make_press_sample(14, 33.0), make_press_sample(13, 22.0)};
  const auto out = st.update(round);
  EXPECT_FLOAT_EQ(out[0].pressure_pa, 11.0f);
  EXPECT_FLOAT_EQ(out[1].pressure_pa, 22.0f);
  EXPECT_FLOAT_EQ(out[2].pressure_pa, 33.0f);
  EXPECT_FLOAT_EQ(out[3].pressure_pa, 44.0f);
}

TEST(BladderBridgeTest, MissingNodeWithHistoryCarriesLastValue)
{
  BladderBridgeState st({6, 13, 14, 15});
  st.update(make_full_round(10.0, 20.0, 30.0, 40.0));
  // 第二轮缺 node_id=13
  const auto round2 = std::vector<SampleInput>{
    make_press_sample(6, 11.0), make_press_sample(14, 31.0), make_press_sample(15, 41.0)};
  const auto out = st.update(round2);
  EXPECT_FLOAT_EQ(out[1].pressure_pa, 20.0f);  // 沿用上轮值
  EXPECT_FALSE(out[1].valid);
  EXPECT_TRUE(out[1].stale);
  EXPECT_TRUE(out[0].valid);   // 其余槽位正常
}

TEST(BladderBridgeTest, MissingNodeWithoutHistoryIsNaN)
{
  BladderBridgeState st({6, 13, 14, 15});
  // 首轮即缺 node_id=15
  const auto round = std::vector<SampleInput>{
    make_press_sample(6, 10.0), make_press_sample(13, 20.0), make_press_sample(14, 30.0)};
  const auto out = st.update(round);
  EXPECT_TRUE(std::isnan(out[3].pressure_pa));
  EXPECT_FALSE(out[3].valid);
  EXPECT_FALSE(out[3].stale);
}

TEST(BladderBridgeTest, PressValidZeroCarriesLastValue)
{
  BladderBridgeState st({6, 13, 14, 15});
  st.update(make_full_round(10.0, 20.0, 30.0, 40.0));
  // 第二轮 14 号压力读失败(press_valid=0)但温度成功(temp_valid=1)
  auto s14 = make_press_sample(14, 999.0);
  s14.press_valid = 0;
  const auto round2 = std::vector<SampleInput>{
    make_press_sample(6, 11.0), make_press_sample(13, 21.0), s14,
    make_press_sample(15, 41.0)};
  const auto out = st.update(round2);
  EXPECT_FLOAT_EQ(out[2].pressure_pa, 30.0f);  // 压力沿用
  EXPECT_FALSE(out[2].valid);
  EXPECT_TRUE(out[2].stale);
  EXPECT_FLOAT_EQ(out[2].temp_c, 26.0f);       // 温度本轮有效
}

TEST(BladderBridgeTest, StaleDebounceRoundNotFooledByCopiedPressValid)
{
  // 最关键语义用例: LoRa 去抖沿用轮 stale=1 时 press_valid 是拷贝的旧值(仍为 1),
  // bridge 必须按 stale 先判, 不被拷贝来的 press_valid=1 迷惑
  BladderBridgeState st({6, 13, 14, 15});
  st.update(make_full_round(10.0, 20.0, 30.0, 40.0));
  auto s6 = make_press_sample(6, 555.0);  // 假新值, 若被误信则测试失败
  s6.stale = 1;
  const auto round2 = std::vector<SampleInput>{
    s6, make_press_sample(13, 21.0), make_press_sample(14, 31.0),
    make_press_sample(15, 41.0)};
  const auto out = st.update(round2);
  EXPECT_FLOAT_EQ(out[0].pressure_pa, 10.0f);  // 沿用 bridge 缓存, 非 555
  EXPECT_FALSE(out[0].valid);
  EXPECT_TRUE(out[0].stale);
}

TEST(BladderBridgeTest, OfflineNodeCarriesLastValue)
{
  BladderBridgeState st({6, 13, 14, 15});
  st.update(make_full_round(10.0, 20.0, 30.0, 40.0));
  auto s15 = make_press_sample(15, 999.0);
  s15.online = 0;
  const auto round2 = std::vector<SampleInput>{
    make_press_sample(6, 11.0), make_press_sample(13, 21.0), make_press_sample(14, 31.0), s15};
  const auto out = st.update(round2);
  EXPECT_FLOAT_EQ(out[3].pressure_pa, 40.0f);
  EXPECT_FALSE(out[3].valid);
  EXPECT_TRUE(out[3].stale);
}

TEST(BladderBridgeTest, RecoveryAfterOffline)
{
  BladderBridgeState st({6, 13, 14, 15});
  st.update(make_full_round(10.0, 20.0, 30.0, 40.0));
  // 多轮离线
  for (int i = 0; i < 3; ++i) {
    st.update({});
  }
  // 恢复实测
  const auto out = st.update(make_full_round(100.0, 200.0, 300.0, 400.0));
  EXPECT_FLOAT_EQ(out[0].pressure_pa, 100.0f);
  for (std::size_t i = 0; i < kNumBladders; ++i) {
    EXPECT_TRUE(out[i].valid);
    EXPECT_FALSE(out[i].stale);
  }
}

TEST(BladderBridgeTest, TemperatureIndependentFromPressure)
{
  BladderBridgeState st({6, 13, 14, 15});
  // 首轮: 温度失败(temp_valid=0)压力成功
  auto s6 = make_press_sample(6, 10.0);
  s6.temp_valid = 0;
  st.update({s6, make_press_sample(13, 20.0), make_press_sample(14, 30.0),
      make_press_sample(15, 40.0)});
  // 第二轮: 恢复 temp_valid=1 且温度新值
  const auto out = st.update(make_full_round(11.0, 21.0, 31.0, 41.0));
  EXPECT_FLOAT_EQ(out[0].temp_c, 26.0f);  // 本轮温度有效
  // 反向验证首轮: 重新构造检查(用新状态机)
  BladderBridgeState st2({6, 13, 14, 15});
  auto s6b = make_press_sample(6, 10.0);
  s6b.temp_valid = 0;
  const auto out2 = st2.update(
    {s6b, make_press_sample(13, 20.0), make_press_sample(14, 30.0),
      make_press_sample(15, 40.0)});
  EXPECT_TRUE(std::isnan(out2[0].temp_c));      // 首轮温度 NaN
  EXPECT_TRUE(out2[0].valid);                   // 但压力有效
}

TEST(BladderBridgeTest, NegativePressurePassesThrough)
{
  // 呼应历史教训: 负压差曾按无符号解析成 4294967279 巨值
  BladderBridgeState st({6, 13, 14, 15});
  const auto out = st.update(make_full_round(-123.4, 0.0, 56.0, -6.0));
  EXPECT_FLOAT_EQ(out[0].pressure_pa, -123.4f);
  EXPECT_FLOAT_EQ(out[3].pressure_pa, -6.0f);
}

TEST(BladderBridgeTest, CustomNodeMapping)
{
  // 参数化映射: [21,22,23,24]
  BladderBridgeState st({21, 22, 23, 24});
  const auto out = st.update(
    {make_press_sample(24, 4.0), make_press_sample(21, 1.0),
      make_press_sample(22, 2.0), make_press_sample(23, 3.0)});
  EXPECT_FLOAT_EQ(out[0].pressure_pa, 1.0f);
  EXPECT_FLOAT_EQ(out[3].pressure_pa, 4.0f);
}

TEST(BladderBridgeTest, DuplicateNodeIdTakesFirst)
{
  BladderBridgeState st({6, 13, 14, 15});
  auto dup = make_press_sample(6, 999.0);
  const auto out = st.update(
    {make_press_sample(6, 10.0), dup, make_press_sample(13, 20.0),
      make_press_sample(14, 30.0), make_press_sample(15, 40.0)});
  EXPECT_FLOAT_EQ(out[0].pressure_pa, 10.0f);  // 取第一条
}

TEST(BladderBridgeTest, IrrelevantNodesIgnored)
{
  BladderBridgeState st({6, 13, 14, 15});
  // 混入无关温度节点(node_id=1/7)不越界、不干扰
  const auto out = st.update(
    {make_press_sample(1, 100.0), make_press_sample(6, 10.0),
      make_press_sample(7, 200.0), make_press_sample(13, 20.0),
      make_press_sample(14, 30.0), make_press_sample(15, 40.0)});
  EXPECT_FLOAT_EQ(out[0].pressure_pa, 10.0f);
  EXPECT_TRUE(out[1].valid);
  EXPECT_TRUE(out[2].valid);
  EXPECT_TRUE(out[3].valid);
}
