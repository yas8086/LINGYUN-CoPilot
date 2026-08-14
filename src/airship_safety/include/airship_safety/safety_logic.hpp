// 灵云01号伴飞电脑 — 安全仲裁纯逻辑 (无 ROS 依赖, 便于单元测试)
//
// 聚合各安全判据, 决定 safe_to_control 总开关。
// 设计原则(fail-safe): 任一关键判据异常 => 禁止控制。
#ifndef AIRSHIP_SAFETY__SAFETY_LOGIC_HPP_
#define AIRSHIP_SAFETY__SAFETY_LOGIC_HPP_

#include <cstdint>
#include <string>

namespace airship_safety
{

// DCDC 故障位: bit2 是"输出状态"(非故障), 其余位为故障
constexpr uint8_t kDcdcOutputStatusBit = 0x04;
constexpr uint8_t kDcdcFaultMask = ~kDcdcOutputStatusBit;

// 聚合判定结果
struct SafetyDecision
{
  bool safe_to_control = false;  // 是否允许下发控制命令
  bool dcdc_ok = false;          // DCDC 判据是否通过
  bool backup_battery_ok = false;  // 备用电源判据是否通过
  std::string reason;            // 不安全原因 (空串表示安全)
};

inline const char * dcdc_reason()
{
  return "DCDC 异常或离线, 禁止下发控制命令";
}

inline const char * battery_reason()
{
  return "BMS 电压判据不满足, 禁止下发控制命令";
}

inline const char * backup_battery_reason()
{
  return "备用电源异常或离线, 禁止下发控制命令";
}

// DCDC 判据: 在线(info_ok) 且 无故障位(排除输出状态位)
inline bool dcdc_judge(bool online, uint8_t fault_word)
{
  return online && ((fault_word & kDcdcFaultMask) == 0);
}

// BMS 判据: 在线 且 总压不低于下限 且 无告警级
inline bool battery_judge(
  bool online, float pack_voltage, uint8_t alarm_level, float min_voltage)
{
  return online && (pack_voltage >= min_voltage) && (alarm_level == 0);
}

// 12S 备用电源判据: 在线 且 总压不低于下限 且 无故障位
inline bool backup_battery_judge(
  bool online, float pack_voltage, uint32_t fault_word, float min_voltage)
{
  return online && (pack_voltage >= min_voltage) && (fault_word == 0);
}

// 聚合判定: 当前所有判据均通过 => safe_to_control=true
// 参数:
//   dcdc_ok: DCDC 判据结果 (在线且无故障)
//   battery_ok: 主 BMS 判据结果
//   backup_ok: 备用电源判据结果
// 返回: 聚合决策; reason 描述首个不满足的判据
inline SafetyDecision aggregate(bool dcdc_ok, bool battery_ok, bool backup_ok)
{
  SafetyDecision decision;
  decision.dcdc_ok = dcdc_ok;
  decision.backup_battery_ok = backup_ok;
  decision.safe_to_control = dcdc_ok && battery_ok && backup_ok;
  if (!decision.safe_to_control) {
    if (!dcdc_ok) {
      decision.reason = dcdc_reason();
    } else if (!battery_ok) {
      decision.reason = battery_reason();
    } else {
      decision.reason = backup_battery_reason();
    }
  }
  return decision;
}

}  // namespace airship_safety

#endif  // AIRSHIP_SAFETY__SAFETY_LOGIC_HPP_
