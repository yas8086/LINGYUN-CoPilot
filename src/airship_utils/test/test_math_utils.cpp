// 灵云01号飞艇伴飞电脑 — gtest 单元测试示例
//
// 本文件演示 ROS2 中使用 gtest 写"真正的单元测试"的常见写法:
//   1. TEST()                      —— 最基本的独立测试用例
//   2. TEST_F() + TEST_FIXTURE     —— 测试夹具:SetUp/TearDown 复用公共数据
//   3. TEST_P() + 参数化           —— 同一逻辑跑多组输入数据
//   4. 断言宏                      —— EXPECT_* vs ASSERT_*(聚合 vs 中断)
//   5. 浮点比较                    —— 必须用 EXPECT_FLOAT_EQ / EXPECT_NEAR
//   6. 测试分组与命名规范
//
// 运行方式(在 colcon 工作区):
//   colcon test --packages-select airship_utils
//   colcon test-result --verbose
// 或直接运行单个测试可执行文件:
//   ./build/airship_utils/test_math_utils
//   ./build/airship_utils/test_math_utils --gtest_filter=ClampTest.*
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "airship_utils/math_utils.hpp"

using airship_utils::angle_diff;
using airship_utils::clampf;
using airship_utils::normalize_angle;

// ============================================================
// 1. 最基本的 TEST() 用例
//    语法: TEST(TestSuiteName, TestName)
//    - TestSuiteName 是分组名,TestName 是具体用例名
//    - 一个断言失败不会影响同一 TEST 内其他断言(默认继续执行)
// ============================================================
TEST(NormalizeAngleTest, WrapsPositiveOverflow)
{
  // 7.0 rad ≈ 401°,归一化后应约为 -0.28 rad(-16°)
  double result = normalize_angle(7.0f);
  EXPECT_NEAR(result, 7.0 - 2.0 * M_PI, 1e-4);
}

TEST(NormalizeAngleTest, WrapsNegativeOverflow)
{
  // -7.0 rad ≈ -401°,归一化后应约为 +0.28 rad(+16°)
  double result = normalize_angle(-7.0f);
  EXPECT_NEAR(result, -7.0 + 2.0 * M_PI, 1e-4);
}

TEST(NormalizeAngleTest, KeepsValueInRange)
{
  // 归一化结果必须始终落在闭区间 [-pi, pi]。
  // 注意1: 实现用严格条件(> M_PI / < -M_PI),因此 +pi 不会被翻转,返回范围是 [-pi, pi]。
  // 注意2: 函数返回 float,而 M_PI 是 double。float 版的 pi((float)M_PI)转成 double 后
  //        会略大于 double 的 M_PI(3.1415927410 > 3.1415926535),直接与 double 版 M_PI
  //        比较会误判越界。因此范围边界必须统一用 float 版 pi 比较。
  const float kPi = static_cast<float>(M_PI);
  for (double angle_deg = -720.0; angle_deg <= 720.0; angle_deg += 15.0) {
    double angle = angle_deg * M_PI / 180.0;
    float result = normalize_angle(static_cast<float>(angle));
    // 关键约束: 结果永远在 [-pi, pi]
    EXPECT_GE(result, -kPi);
    EXPECT_LE(result, kPi);
  }
}

// ============================================================
// 2. ASSERT_* 断言(失败即中断当前用例)
//    与 EXPECT_* 的区别: ASSERT_* 失败会立刻 return,后续断言不再执行。
//    适合"前置条件不满足就没必要继续"的场景。
// ============================================================
TEST(AngleDiffTest, DifferenceIsBounded)
{
  // angle_diff 结果必须永远在 (-pi, pi]
  for (size_t i = 0; i < 1000; ++i) {
    float target = static_cast<float>(i) * 0.1f;
    float current = -static_cast<float>(i) * 0.05f;
    float diff = angle_diff(target, current);
    // 若 diff 越界,ASSERT 直接失败并中断本用例,避免后续无意义计算
    ASSERT_GT(diff, -static_cast<float>(M_PI));
    ASSERT_LE(diff, static_cast<float>(M_PI));
  }
}

TEST(AngleDiffTest, ZeroWhenSameAngle)
{
  EXPECT_FLOAT_EQ(angle_diff(0.0f, 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(angle_diff(1.5f, 1.5f), 0.0f);
}

TEST(AngleDiffTest, ShortestPathAcrossPi)
{
  // 从 +170° 到 -170°,最短路径是逆时针跨过 +180° 到 -170°,即 +20°。
  // 计算: target-current = -170°-170° = -340°,加 360° 归一化得 +20°。
  float result = angle_diff(static_cast<float>(-170.0 * M_PI / 180.0),
                            static_cast<float>(170.0 * M_PI / 180.0));
  EXPECT_NEAR(result, 20.0 * M_PI / 180.0, 1e-4);
}

// ============================================================
// 3. TEST_F() + 测试夹具(test fixture)
//    当多个用例共享同一套初始化数据时,用夹具类封装 SetUp/TearDown。
//    每个 TEST_F 用例执行前会自动调用 SetUp,结束后调用 TearDown。
// ============================================================
class ClampTest : public ::testing::Test
{
protected:
  // 在每个用例执行前调用,初始化公共数据
  void SetUp() override
  {
    min_ = -10.0f;
    max_ = 10.0f;
  }

  // 在每个用例执行后调用(本示例无资源需要释放,仅演示)
  void TearDown() override {}

  float min_;
  float max_;
};

TEST_F(ClampTest, ClipsBelowMin)
{
  EXPECT_FLOAT_EQ(clampf(-100.0f, min_, max_), min_);
}

TEST_F(ClampTest, ClipsAboveMax)
{
  EXPECT_FLOAT_EQ(clampf(100.0f, min_, max_), max_);
}

TEST_F(ClampTest, KeepsValueInsideRangeUnchanged)
{
  EXPECT_FLOAT_EQ(clampf(0.0f, min_, max_), 0.0f);
  EXPECT_FLOAT_EQ(clampf(5.0f, min_, max_), 5.0f);
}

TEST_F(ClampTest, BoundaryValuesAreIdentity)
{
  // 边界值本身应保持不变
  EXPECT_FLOAT_EQ(clampf(min_, min_, max_), min_);
  EXPECT_FLOAT_EQ(clampf(max_, min_, max_), max_);
}

// ============================================================
// 4. TEST_P() + 参数化测试 (parameterized test)
//    同一段测试逻辑,对多组输入数据循环执行。
//    遇到输入-期望值对时,比写多个 TEST 更简洁、可维护。
// ============================================================
struct ClampTestCase
{
  float input;
  float lo;
  float hi;
  float expected;
};

class ClampParameterizedTest
  : public ::testing::TestWithParam<ClampTestCase>
{
};

// 把多组 (input, lo, hi, expected) 作为参数喂给同一个测试逻辑
TEST_P(ClampParameterizedTest, ClampsToExpected)
{
  const auto & tc = GetParam();
  EXPECT_FLOAT_EQ(clampf(tc.input, tc.lo, tc.hi), tc.expected);
}

INSTANTIATE_TEST_SUITE_P(
  ClampVariants,
  ClampParameterizedTest,
  ::testing::Values(
    ClampTestCase{-50.0f, -10.0f, 10.0f, -10.0f},
    ClampTestCase{50.0f, -10.0f, 10.0f, 10.0f},
    ClampTestCase{3.0f, -10.0f, 10.0f, 3.0f},
    ClampTestCase{-10.0f, -10.0f, 10.0f, -10.0f},   // 下边界
    ClampTestCase{10.0f, -10.0f, 10.0f, 10.0f},     // 上边界
    ClampTestCase{0.5f, 0.0f, 1.0f, 0.5f}           // 不同区间
));

// ============================================================
// 5. 浮点比较的"坑"
//    gtest 中浮点断言家族:
//    - EXPECT_FLOAT_EQ(a, b)  : 4 ULP 内相等(float 专用,推荐)
//    - EXPECT_DOUBLE_EQ(a, b) : 4 ULP 内相等(double 专用,推荐)
//    - EXPECT_NEAR(a, b, abs_error) : 绝对误差内相等(最常用)
//    不要用 EXPECT_EQ 直接比较浮点(二进制度不准会导致偶发失败)。
// ============================================================
TEST(FloatingPointTest, FloatEqHandlesTinyDifferences)
{
  // 0.1 + 0.2 在二进制浮点下不等于 0.3,但 EXPECT_FLOAT_EQ 能正确判定
  float a = 0.1f + 0.2f;
  EXPECT_FLOAT_EQ(a, 0.3f);
}

TEST(FloatingPointTest, NearAllowsTolerance)
{
  // 带容差的比较: 允许绝对误差在 1e-2 内
  EXPECT_NEAR(1.23456, 1.23, 1e-2);
}

// ============================================================
// 6. 测试"应当失败的输入" / 边界条件
// ============================================================
TEST(ClampTest2, InvalidRangeBehaviour)
{
  // 当 min > max 时(非法参数),实现行为不定——这里只验证不崩溃、不在断言范围外
  // 实际项目应对非法参数做防御性校验,并在此处断言该行为
  EXPECT_NO_THROW({clampf(0.0f, 10.0f, -10.0f);});
}

// gtest 主函数:ament_add_gtest 会注入 main,因此这里不需要写 main。
// 若你用原生 gtest 而非 ament_add_gtest,则需要:
//   int main(int argc, char ** argv) { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
