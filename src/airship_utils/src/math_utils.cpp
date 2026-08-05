// 灵云01号飞艇伴飞电脑 — 通用数学工具实现
#include "airship_utils/math_utils.hpp"

#include <cmath>

namespace airship_utils
{

float angle_diff(float target, float current)
{
  float diff = target - current;
  while (diff > M_PI) {
    diff -= 2.0f * static_cast<float>(M_PI);
  }
  while (diff < -M_PI) {
    diff += 2.0f * static_cast<float>(M_PI);
  }
  return diff;
}

float normalize_angle(float angle)
{
  while (angle > M_PI) {
    angle -= 2.0f * static_cast<float>(M_PI);
  }
  while (angle < -M_PI) {
    angle += 2.0f * static_cast<float>(M_PI);
  }
  return angle;
}

float clampf(float val, float min_val, float max_val)
{
  if (val < min_val) {
    return min_val;
  }
  if (val > max_val) {
    return max_val;
  }
  return val;
}

}  // namespace airship_utils
