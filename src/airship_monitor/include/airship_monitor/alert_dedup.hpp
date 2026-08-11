// 灵云01号伴飞电脑 — 告警去重状态机 (无 ROS 依赖, 便于单元测试)
//
// 用于"离线/恢复"类告警去重: 仅在设备在线状态发生跳变时返回"需要发告警",
// 避免周期性看门狗在离线持续期间反复刷屏。
// 同时区分"离线告警"与"恢复告警", 供上层映射不同严重级别。
#ifndef AIRSHIP_MONITOR__ALERT_DEDUP_HPP_
#define AIRSHIP_MONITOR__ALERT_DEDUP_HPP_

namespace airship_monitor
{

// 一次"状态变化"的结果
struct AlertTransition
{
  bool changed = false;   // 状态是否发生跳变 (仅跳变时需发告警)
  bool now_online = false; // 当前在线状态
};

// 告警去重状态机
// online: 当前设备是否在线
// last_online: 上一次记录的在线状态 (由调用方维护, 初始 false; 不可为 null)
// 返回: 是否发生跳变及新状态; 若跳变则更新 *last_online
inline AlertTransition update_online(bool online, bool * last_online)
{
  AlertTransition t;
  t.now_online = online;
  if (last_online == nullptr) {
    return t;  // 防御: 空指针视为无状态, 不修改
  }
  if (*last_online == online) {
    return t;  // 无变化
  }
  t.changed = true;
  *last_online = online;
  return t;
}

}  // namespace airship_monitor

#endif  // AIRSHIP_MONITOR__ALERT_DEDUP_HPP_