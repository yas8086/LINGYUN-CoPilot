// 灵云01号伴飞电脑 — 设备状态 JSON 打包实现
#include "airship_link/json_packer.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace airship_link
{

// 浮点格式化: 保留最多 4 位有效数字, 去掉多余 0
// 非有限值(NaN/Inf)输出合法 JSON null, 避免产生非法 JSON
// 注: 曾用 %.3g(仅 3 位有效数字)导致 pack_v=512.6 被截成 513 等大值精度丢失, 现放宽为 4 位
static std::string fmt_float(float v)
{
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.4g", static_cast<double>(v));
  return buf;
}

// 高精度浮点格式化(double): 用于经纬度/压力等需要精度的字段,
// 固定 6 位小数, 避免 %.3g 仅 3 位有效数字导致的精度灾难性丢失
static std::string fmt_double(double v)
{
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6f", v);
  return buf;
}

// JSON 字符串转义: 处理双引号/反斜杠/控制字符, 防止字段内嵌引号破坏 JSON
static std::string escape_json_string(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        } else {
          out += c;
        }
        break;
    }
  }
  return out;
}

std::string bms_to_json(const airship_msgs::msg::BmsStatus & msg)
{
  std::string s = "\"bms\":{";
  s += "\"online\":" + std::string(msg.online ? "true" : "false") + ",";
  s += "\"pack_v\":" + fmt_float(msg.pack_voltage) + ",";
  s += "\"pack_i\":" + fmt_float(msg.pack_current) + ",";
  s += "\"soc\":" + fmt_float(msg.soc) + ",";
  s += "\"rsoc\":" + fmt_float(msg.real_soc) + ",";
  s += "\"max_v\":" + fmt_float(msg.max_cell_voltage) + ",";
  s += "\"min_v\":" + fmt_float(msg.min_cell_voltage) + ",";
  s += "\"diff_v\":" + fmt_float(msg.cell_voltage_diff) + ",";
  s += "\"max_t\":" + fmt_float(msg.max_cell_temp) + ",";
  s += "\"min_t\":" + fmt_float(msg.min_cell_temp) + ",";
  s += "\"avg_t\":" + fmt_float(msg.avg_cell_temp) + ",";
  s += "\"diff_t\":" + fmt_float(msg.temp_diff) + ",";
  s += "\"riso_p\":" + std::to_string(msg.positive_insulation_kohm) + ",";
  s += "\"riso_n\":" + std::to_string(msg.negative_insulation_kohm) + ",";
  s += "\"alarm\":" + std::to_string(msg.alarm_level);
  s += "}";
  return s;
}

std::string backup_bms_to_json(const airship_msgs::msg::BackupBmsStatus & msg)
{
  std::string s = "\"backup\":{";
  s += "\"online\":" + std::string(msg.online ? "true" : "false") + ",";
  s += "\"pack_v\":" + fmt_float(msg.pack_voltage) + ",";
  s += "\"pack_i\":" + fmt_float(msg.pack_current) + ",";
  s += "\"soc\":" + fmt_float(msg.soc) + ",";
  s += "\"soh\":" + fmt_float(msg.soh) + ",";
  s += "\"max_v\":" + fmt_float(msg.max_cell_voltage) + ",";
  s += "\"min_v\":" + fmt_float(msg.min_cell_voltage) + ",";
  s += "\"diff_v\":" + fmt_float(msg.cell_voltage_diff) + ",";
  s += "\"max_t\":" + fmt_float(msg.max_cell_temp) + ",";
  s += "\"min_t\":" + fmt_float(msg.min_cell_temp) + ",";
  s += "\"avg_t\":" + fmt_float(msg.avg_cell_temp) + ",";
  s += "\"diff_t\":" + fmt_float(msg.temp_diff) + ",";
  s += "\"alarm\":" + std::to_string(msg.alarm_word) + ",";
  s += "\"protect\":" + std::to_string(msg.protect_word) + ",";
  s += "\"fault\":" + std::to_string(msg.fault_word) + ",";
  s += "\"sys\":" + std::to_string(msg.system_word);
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

std::string fc_to_json(const airship_msgs::msg::FlightStatus & msg)
{
  std::string s = "\"fc\":{";
  s += "\"online\":" + std::string(msg.online ? "true" : "false") + ",";
  s += "\"roll\":" + fmt_float(msg.roll_deg) + ",";
  s += "\"pitch\":" + fmt_float(msg.pitch_deg) + ",";
  s += "\"yaw\":" + fmt_float(msg.yaw_deg) + ",";
  s += "\"lat\":" + fmt_double(msg.lat) + ",";
  s += "\"lon\":" + fmt_double(msg.lon) + ",";
  s += "\"alt\":" + fmt_float(msg.alt_rel) + ",";
  s += "\"vx\":" + fmt_float(msg.vx) + ",";
  s += "\"vy\":" + fmt_float(msg.vy) + ",";
  s += "\"vz\":" + fmt_float(msg.vz) + ",";
  s += "\"mode\":\"" + escape_json_string(msg.flight_mode) + "\",";
  s += "\"armed\":" + std::string(msg.armed ? "true" : "false") + ",";
  s += "\"batt_v\":" + fmt_float(msg.battery_voltage) + ",";
  s += "\"batt_pct\":" + fmt_float(msg.battery_remaining);
  s += "}";
  return s;
}

std::string lora_to_json(const airship_msgs::msg::LoRaSamples & msg)
{
  std::string s = "\"lora\":{\"nodes\":[";
  bool first = true;
  for (const auto & smp : msg.samples) {
    if (smp.online == 0) {
      continue;  // 仅打包在线节点
    }
    if (!first) {
      s += ",";
    }
    first = false;
    s += "{\"id\":" + std::to_string(smp.node_id) + ",";
    s += "\"online\":1,";
    s += "\"temp\":" + fmt_float(smp.temp_celsius) + ",";
    s += "\"pressure\":" + fmt_double(smp.pressure_pa) + ",";
    s += "\"alarm\":" + std::to_string(smp.alarm);
    s += "}";
  }
  s += "]}";
  return s;
}

std::string pack_telemetry_json(
  double timestamp_sec,
  const airship_msgs::msg::BmsStatus * bms,
  const airship_msgs::msg::MpptStatus * mppt,
  const airship_msgs::msg::DcdcStatus * dcdc,
  const airship_msgs::msg::FlightStatus * flight,
  const airship_msgs::msg::LoRaSamples * lora,
  const airship_msgs::msg::BackupBmsStatus * backup)
{
  std::string s = "{\"t\":";
  char tb[32];
  std::snprintf(tb, sizeof(tb), "%.3f", timestamp_sec);
  s += tb;

  // 依次追加在线设备, 字段间用逗号分隔
  if (bms != nullptr && bms->online) {
    s += "," + bms_to_json(*bms);
  }
  if (backup != nullptr && backup->online) {
    s += "," + backup_bms_to_json(*backup);
  }
  if (mppt != nullptr && mppt->online) {
    s += "," + mppt_to_json(*mppt);
  }
  if (dcdc != nullptr && dcdc->online) {
    s += "," + dcdc_to_json(*dcdc);
  }
  if (flight != nullptr && flight->online) {
    s += "," + fc_to_json(*flight);
  }
  if (lora != nullptr && !lora->samples.empty()) {
    s += "," + lora_to_json(*lora);
  }

  s += "}";
  return s;
}

}  // namespace airship_link
