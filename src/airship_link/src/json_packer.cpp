// 灵云01号伴飞电脑 — 设备状态 JSON 打包实现
#include "airship_link/json_packer.hpp"

#include <cstdio>
#include <string>

namespace airship_link
{

// 浮点格式化: 保留最多 3 位小数, 去掉多余 0
std::string fmt_float(float v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3g", static_cast<double>(v));
  return buf;
}

std::string bms_to_json(const airship_msgs::msg::BmsStatus & msg)
{
  std::string s = "\"bms\":{";
  s += "\"online\":" + std::string(msg.online ? "true" : "false") + ",";
  s += "\"pack_v\":" + fmt_float(msg.pack_voltage) + ",";
  s += "\"pack_i\":" + fmt_float(msg.pack_current) + ",";
  s += "\"soc\":" + fmt_float(msg.soc) + ",";
  s += "\"max_v\":" + fmt_float(msg.max_cell_voltage) + ",";
  s += "\"min_v\":" + fmt_float(msg.min_cell_voltage) + ",";
  s += "\"diff_v\":" + fmt_float(msg.cell_voltage_diff) + ",";
  s += "\"max_t\":" + fmt_float(msg.max_cell_temp) + ",";
  s += "\"alarm\":" + std::to_string(msg.alarm_level);
  s += "}";
  return s;
}

std::string mppt_to_json(const airship_msgs::msg::MpptStatus & msg)
{
  std::string s = "\"mppt\":{";
  s += "\"online\":" + std::string(msg.online ? "true" : "false") + ",";
  s += "\"pv_v\":" + fmt_float(msg.pv_voltage) + ",";
  s += "\"pv_p\":" + fmt_float(msg.pv_power) + ",";
  s += "\"batt_v\":" + fmt_float(msg.battery_voltage) + ",";
  s += "\"charge_i\":" + fmt_float(msg.charge_current) + ",";
  s += "\"today\":" + fmt_float(msg.energy_today) + ",";
  s += "\"total\":" + fmt_float(msg.energy_total) + ",";
  s += "\"fault\":" + std::to_string(msg.fault_state);
  s += "}";
  return s;
}

std::string dcdc_to_json(const airship_msgs::msg::DcdcStatus & msg)
{
  std::string s = "\"dcdc\":{";
  s += "\"online\":" + std::string(msg.online ? "true" : "false") + ",";
  s += "\"in_v\":" + fmt_float(msg.input_voltage) + ",";
  s += "\"out_v\":" + fmt_float(msg.output_voltage) + ",";
  s += "\"out_i\":" + fmt_float(msg.output_current) + ",";
  s += "\"out_p\":" + fmt_float(msg.output_power) + ",";
  s += "\"temp\":" + fmt_float(msg.heatsink_temp) + ",";
  s += "\"enabled\":" + std::string(msg.output_enabled ? "true" : "false") + ",";
  s += "\"fault\":" + std::to_string(msg.fault_word);
  s += "}";
  return s;
}

std::string pack_telemetry_json(
  double timestamp_sec,
  const airship_msgs::msg::BmsStatus * bms,
  const airship_msgs::msg::MpptStatus * mppt,
  const airship_msgs::msg::DcdcStatus * dcdc)
{
  std::string s = "{\"t\":";
  char tb[32];
  std::snprintf(tb, sizeof(tb), "%.3f", timestamp_sec);
  s += tb;

  // 依次追加在线设备, 字段间用逗号分隔
  if (bms != nullptr && bms->online) {
    s += "," + bms_to_json(*bms);
  }
  if (mppt != nullptr && mppt->online) {
    s += "," + mppt_to_json(*mppt);
  }
  if (dcdc != nullptr && dcdc->online) {
    s += "," + dcdc_to_json(*dcdc);
  }

  s += "}";
  return s;
}

}  // namespace airship_link
