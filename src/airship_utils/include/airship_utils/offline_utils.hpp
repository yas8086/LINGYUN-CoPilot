// 灵云01号飞艇伴飞电脑 — 设备离线判定工具(纯函数)
// 从 bms/mppt/dcdc 三个 CAN 节点的 receive_loop 中提取"离线兜底发布时机判定",
// 消除三处复制的逻辑差异, 并使该判定成为可单测的纯逻辑(历史 BUG 恰集中在此区)。
#ifndef AIRSHIP_UTILS__OFFLINE_UTILS_HPP_
#define AIRSHIP_UTILS__OFFLINE_UTILS_HPP_

namespace airship_utils
{

// 判断此刻是否应发布 offline 兜底消息。
// 语义: 距上次有效数据已超过 timeout(设备失联) 且 距上次 offline 发布已满一个 timeout
//       (维持 "超时后每 timeout 周期重发一次 offline" 的兜底节奏)。
// 参数均为秒; 纯计算无副作用, 调用方在返回 true 时自行更新 last_offline_pub 并发消息。
inline bool should_publish_offline(
  double now_s, double last_data_s, double last_offline_pub_s, double timeout_s)
{
  return (now_s - last_data_s) > timeout_s && (now_s - last_offline_pub_s) >= timeout_s;
}

}  // namespace airship_utils

#endif  // AIRSHIP_UTILS__OFFLINE_UTILS_HPP_
