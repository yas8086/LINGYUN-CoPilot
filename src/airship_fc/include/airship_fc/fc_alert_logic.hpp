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
#include <ctime>
#include <string>
#include <vector>

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

// ===== 默认告警规则表 (2026-09-03 从 fc_monitor_node 提取, 便于单测方向断言) =====
// 布尔标志类规则(ekf_const_pos_mode/motor_stuck)必须 upper_side=true:
// flag=1(异常态)才告警。曾误写 false 导致正常态持续 EMERGENCY、真异常反被 Clear
// (2026-09-03 修复, 由 DefaultRulesBooleanDirection 单测锁定防回归)。
struct NamedRule
{
  std::string name;
  Threshold t;
};

// 阈值取自文档 07 安全阈值表; 与 fc_monitor_node::setup_alert_rules 保持同源。
inline std::vector<NamedRule> make_default_rules()
{
  return {
    {"roll_deg", {20.0f, 30.0f, 45.0f, true}},
    {"pitch_deg", {10.0f, 15.0f, 20.0f, true}},
    {"yaw_rate", {10.0f, 20.0f, 30.0f, true}},
    {"climb_rate", {3.0f, 5.0f, 8.0f, true}},
    {"altitude_agl_high", {140.0f, 148.0f, 150.0f, true}},
    {"altitude_agl_low", {3.0f, 2.0f, 1.0f, false}},
    {"battery_voltage", {48.0f, 46.0f, 44.0f, false}},
    {"battery_remaining", {30.0f, 20.0f, 10.0f, false}},
    // GPS 定位质量: fix_type < 3 (无 3D fix) 视为定位失效 (0=无GPS,1=NO_FIX,2=2D)
    {"gps_fix_loss", {3.0f, 3.0f, 3.0f, false}},
    // EKF 退化到位置锁定模式 (GPS 不可用) -> 定位精度骤降, 危急; flag=1 时告警
    {"ekf_const_pos_mode", {0.5f, 0.5f, 0.5f, true}},
    // 卫星数过少 (低于 6 颗警告)-> 影响定位可靠性
    {"gps_satellites_low", {6.0f, 5.0f, 4.0f, false}},
    // 电机堵转: 任一电机输出>0.5 但转速≈0 (需 ESC 遥测数据, 否则跳过); stuck=1 时告警
    {"motor_stuck", {0.5f, 0.5f, 0.5f, true}},
  };
}

// ===== CSV 按天切分工具 (2026-09-03 提取, 便于单测) =====
// 生成 YYYYMMDD 本地日期串(跨天切分判定键)
// 用 to_string 拼接而非 snprintf: 避免 gcc -Wformat-truncation 对
// tm_year/tm_mon 理论值域的告警(正常值恒为 4/2/2 位)
inline std::string csv_date_key(const std::tm & lt)
{
  const int y = lt.tm_year + 1900;
  const int mo = lt.tm_mon + 1;
  const int d = lt.tm_mday;
  std::string s;
  s.reserve(8);
  s += std::to_string(y);
  if (mo < 10) {s += '0';}
  s += std::to_string(mo);
  if (d < 10) {s += '0';}
  s += std::to_string(d);
  return s;
}

// 生成当天的 CSV 文件路径: <dir>/fc_status_YYYYMMDD.csv
inline std::string csv_path_for_date(const std::string & dir, const std::tm & lt)
{
  return dir + "/fc_status_" + csv_date_key(lt) + ".csv";
}

}  // namespace fc_logic

#endif  // AIRSHIP_FC__FC_ALERT_LOGIC_HPP_
