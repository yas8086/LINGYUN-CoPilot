// 灵云01号伴飞电脑 — 设备状态 JSON 打包
// 将 BMS/MPPT/DCDC 状态序列化为单行 JSON, 供串口数传下传地面 Qt 上位机
// 无 ROS 依赖, 便于单元测试
#ifndef AIRSHIP_LINK__JSON_PACKER_HPP_
#define AIRSHIP_LINK__JSON_PACKER_HPP_

#include <string>

#include "airship_msgs/msg/backup_bms_status.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/flight_status.hpp"
#include "airship_msgs/msg/lo_ra_samples.hpp"
#include "airship_msgs/msg/mppt_status.hpp"

namespace airship_link
{

// BMS 状态 -> JSON 片段
std::string bms_to_json(const airship_msgs::msg::BmsStatus & msg);

// 12S 备用电源 BMS 状态 -> JSON 片段
std::string backup_bms_to_json(const airship_msgs::msg::BackupBmsStatus & msg);

// MPPT 状态 -> JSON 片段
std::string mppt_to_json(const airship_msgs::msg::MpptStatus & msg);

// DCDC 状态 -> JSON 片段
std::string dcdc_to_json(const airship_msgs::msg::DcdcStatus & msg);

// 飞控状态 -> JSON 片段
std::string fc_to_json(const airship_msgs::msg::FlightStatus & msg);

// LoRa 温度/压力采集 -> JSON 片段 (仅含在线节点)
std::string lora_to_json(const airship_msgs::msg::LoRaSamples & msg);

// 组装完整帧: {"t":<sec>, "bms":{...}, "backup":{...}, "mppt":{...}, "dcdc":{...}, "fc":{...}, "lora":{...}}
// 空设备(online=false)不出现在帧中; flight/lora/backup 可不传(为 nullptr)以保持向后兼容
std::string pack_telemetry_json(
  double timestamp_sec,
  const airship_msgs::msg::BmsStatus * bms,
  const airship_msgs::msg::MpptStatus * mppt,
  const airship_msgs::msg::DcdcStatus * dcdc,
  const airship_msgs::msg::FlightStatus * flight = nullptr,
  const airship_msgs::msg::LoRaSamples * lora = nullptr,
  const airship_msgs::msg::BackupBmsStatus * backup = nullptr);

}  // namespace airship_link

#endif  // AIRSHIP_LINK__JSON_PACKER_HPP_
