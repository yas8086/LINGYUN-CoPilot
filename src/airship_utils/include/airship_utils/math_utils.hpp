// 灵云01号飞艇伴飞电脑 — 通用数学工具
// 从 airship_control 提取的通用纯函数,便于单元测试与复用。
// 所有函数均为纯函数(无状态、无副作用),因此非常适合 gtest 单元测试。
#ifndef AIRSHIP_UTILS__MATH_UTILS_HPP_
#define AIRSHIP_UTILS__MATH_UTILS_HPP_

namespace airship_utils
{

// 角度差,归一化到 [-pi, pi]
// 例: target=3.0, current=0.0 -> ~3.0(若 >pi 则减 2pi)
float angle_diff(float target, float current);

// 角度归一化到 [-pi, pi]
float normalize_angle(float angle);

// 限幅: 将 val 限制在 [min_val, max_val] 区间内
float clampf(float val, float min_val, float max_val);

}  // namespace airship_utils

#endif  // AIRSHIP_UTILS__MATH_UTILS_HPP_
