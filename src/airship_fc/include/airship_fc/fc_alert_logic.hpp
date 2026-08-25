// 灵云01号飞艇伴飞电脑 — 飞控告警规则引擎(纯逻辑, 无 rclcpp 依赖)
//
// 从 fc_monitor_node.cpp 中提取可单测的告警判定核心:
//   - 阈值分级判定 rule_level()
//   - 去抖状态机 update_alert()(累积 N 次越界才发, 等级变化去重, 恢复解除)
//   - 电池剩余量归一化 normalize_battery_remaining()
// 节点层保留: 采集适配(rule_value 读取节点状态)、告警发布(ROS 消息)。
#ifndef AIRSHIP_FC__FC_ALERT_LOGIC_HPP_
#define AIRSHIP_FC__FC_ALERT_LOGIC_HPP_

#include <cstdint>

namespace fc_logic
{

// 告警等级(与 FcAlert.msg 的 LEVEL_* 保持一致)
namespace level
{
constexpr uint8_t kInfo = 0;
constexpr uint8_t kWarning = 1;
constexpr uint8_t kCritical = 2;
constexpr uint8_t kEmergency = 3;
}  // namespace level

// 规则阈值(纯参数, 无状态)
struct Threshold
{
  float warning = 0.0f;    // 警告阈值
  float critical = 0.0f;   // 严重阈值
  float emergency = 0.0f;  // 危急阈值
  bool upper_side = true;  // true=超过阈值告警(>); false=低于阈值告警(<)
};

// 去抖/告警状态(可变, 由节点持有)
struct AlertState
{
  uint8_t current_level = 0;  // 当前活跃等级(0=无告警)
  int debounce_count = 0;     // 连续越界计数
};

// update_alert 的动作
enum class Action
{
  kNone,       // 无变化, 无需发消息
  kActivate,   // 触发/升级告警(携带新 level 与触发值)
  kClear,      // 从告警态恢复正常(解除)
};

// 一次告警判定结果
struct Decision
{
  Action action = Action::kNone;
  uint8_t level = level::kInfo;  // Activate 时的等级
  float value = 0.0f;            // 触发的参数值
};

// 纯函数: 按规则与当前值计算告警等级(0=正常)。
inline uint8_t rule_level(const Threshold & t, float value)
{
  if (t.upper_side) {
    if (value > t.emergency) {return level::kEmergency;}
    if (value > t.critical) {return level::kCritical;}
    if (value > t.warning) {return level::kWarning;}
  } else {
    if (value < t.emergency) {return level::kEmergency;}
    if (value < t.critical) {return level::kCritical;}
    if (value < t.warning) {return level::kWarning;}
  }
  return level::kInfo;
}

// 纯函数: 电池剩余量归一化到 0~100 百分比。
// MAVROS 的 sensor_msgs/BatteryState.percentage 契约为 0~1, 但兼容直接透传 0~100
// 的实现(防御性): 值域恰好在 (0,1] 才乘 100。
inline float normalize_battery_remaining(float percentage)
{
  if (percentage > 0.0f && percentage <= 1.0f) {
    return percentage * 100.0f;
  }
  return percentage;
}

// 去抖状态机: 每采样周期对单条规则调用一次, 内部更新 state。
// debounce_n: 连续越界多少次才真正发声(防瞬时抖动误报)。
inline Decision update_alert(
  const Threshold & t, float value, AlertState & st, int debounce_n)
{
  const uint8_t lvl = rule_level(t, value);
  if (lvl != level::kInfo) {
    // 越界: 累积计数, 达阈值且等级变化(升级/切换)才触发
    ++st.debounce_count;
    if (st.debounce_count >= debounce_n && lvl != st.current_level) {
      st.current_level = lvl;
      return Decision{Action::kActivate, lvl, value};
    }
    return Decision{Action::kNone, level::kInfo, value};
  }
  // 恢复正常: 若曾在告警态则解除
  st.debounce_count = 0;
  if (st.current_level != level::kInfo) {
    st.current_level = level::kInfo;
    return Decision{Action::kClear, level::kInfo, value};
  }
  return Decision{Action::kNone, level::kInfo, value};
}

}  // namespace fc_logic

#endif  // AIRSHIP_FC__FC_ALERT_LOGIC_HPP_
