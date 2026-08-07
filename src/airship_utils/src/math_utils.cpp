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

void quat_to_euler(
  float w, float x, float y, float z, float * roll, float * pitch, float * yaw)
{
  // ZYX 顺序 (与 PX4/MAVROS 常用约定一致)
  const float r = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  const float p = std::asin(clampf(2.0f * (w * y - z * x), -1.0f, 1.0f));
  const float yw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
  *roll = r;
  *pitch = p;
  *yaw = yw;
}

}  // namespace airship_utils
