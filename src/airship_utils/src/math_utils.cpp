// 灵云01号飞艇伴飞电脑 — 通用数学工具实现
#include "airship_utils/math_utils.hpp"

#include <cmath>

namespace airship_utils
{

float angle_diff(float target, float current)
{
  // std::remainder 返回 [-π, π] 区间余数, 比 while 循环更高效且对任意大角正确
  return std::remainder(static_cast<double>(target - current),
    2.0 * M_PI);
}

float normalize_angle(float angle)
{
  // 与 angle_diff 一致, 用 std::remainder 归一化到 [-π, π]:
  // 对任意大角 O(1) 且正确, 避免 while 循环在极端输入(如积分漂移)下卡顿
  return static_cast<float>(
    std::remainder(static_cast<double>(angle), 2.0 * M_PI));
}

float clampf(float val, float min_val, float max_val)
{
  // NaN 防护: NaN 与任何数比较均为 false, 会穿透下方两个 if 直接原样返回,
  // 下游 static_cast<uint16_t>(NaN) 属未定义行为(如 DCDC 控制帧构造)。
  // 非法输入(NaN)按区间下限处理(对物理量而言是安全侧)。
  if (std::isnan(val)) {
    return min_val;
  }
  if (val < min_val) {
    return min_val;
  }
  if (val > max_val) {
    return max_val;
  }
  return val;
}

void quat_to_euler(
  float w, float x, float y, float z, float * roll, float * pitch, float * yaw)
{
  // 先归一化, 避免非单位四元数导致姿态失真
  const float norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (norm < 1e-6f) {
    // 零范数(非法四元数) 安全回退
    *roll = 0.0f;
    *pitch = 0.0f;
    *yaw = 0.0f;
    return;
  }
  const float nw = w / norm;
  const float nx = x / norm;
  const float ny = y / norm;
  const float nz = z / norm;
  // ZYX 顺序 (与 PX4/MAVROS 常用约定一致)
  // 万向节锁定: pitch=±90° 时 roll/yaw 退化(固有特性), asin 已用 clampf 限幅
  *roll = std::atan2(2.0f * (nw * nx + ny * nz), 1.0f - 2.0f * (nx * nx + ny * ny));
  *pitch = std::asin(clampf(2.0f * (nw * ny - nz * nx), -1.0f, 1.0f));
  *yaw = std::atan2(2.0f * (nw * nz + nx * ny), 1.0f - 2.0f * (ny * ny + nz * nz));
}

}  // namespace airship_utils
