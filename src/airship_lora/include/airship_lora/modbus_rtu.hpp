// 灵云01号伴飞电脑 — LoRa 集中器 Modbus RTU 协议解析
// 纯协议逻辑, 无 ROS / 无串口依赖, 便于 gtest 单测。
//
// 协议(与 Qt 参考实现一致):
//   - 功能码 0x04 读输入寄存器
//   - 温度寄存器: temp_reg_addr + (node_id-1), 1 寄存器, int16(raw)/10.0 ℃
//   - 压力寄存器: pressure_reg_addr + (node_id-1)*2, 2 寄存器, (high<<16|low) Pa
//   - CRC-16/MODBUS
#ifndef AIRSHIP_LORA__MODBUS_RTU_HPP_
#define AIRSHIP_LORA__MODBUS_RTU_HPP_

#include <cstdint>
#include <vector>

namespace airship_lora
{

// 单个 LoRa 节点采样(纯数据, 与 ROS 消息解耦)
struct LoraSampleData
{
  int node_id = 0;
  bool is_pressure = false;
  float temp_celsius = 0.0f;
  double pressure_pa = 0.0;
  int raw = 0;
  int online = 0;
  int alarm = 0;
  // 有效性标志(2026-09-03): 区分"读取成功"与"部分成功", 防 0 值污染 summary/JSON:
  //   temp_valid:  温度寄存器本轮读取成功(temp_celsius 为真实测量);
  //   press_valid: 压力寄存器本轮读取成功(仅压力节点);
  //   stale:       本轮读取失败, 数值为离线去抖窗口内沿用的上一帧有效值。
  bool temp_valid = false;
  bool press_valid = false;
  bool stale = false;
};

// CRC-16/MODBUS
uint16_t modbus_crc16(const uint8_t * data, size_t len);

// 构建读输入寄存器请求帧 (功能码 0x04): [slave][0x04][addrHi][addrLo][qtyHi][qtyLo][crc]
std::vector<uint8_t> build_read_request(
  uint8_t slave_addr, uint16_t start_addr, uint16_t quantity);

// 校验完整响应帧 CRC(含尾部 CRC), 合法返回 true
bool check_crc(const std::vector<uint8_t> & frame);

// 解析温度响应帧 -> temp_celsius / raw。校验从机地址与 CRC, 失败返回 false。
bool parse_temp_response(
  const std::vector<uint8_t> & resp, uint8_t slave_addr, float & temp_celsius, int & raw);

// 解析压力响应帧 -> pressure_pa。校验从机地址与 CRC, 失败返回 false。
bool parse_pressure_response(
  const std::vector<uint8_t> & resp, uint8_t slave_addr, double & pressure_pa);

// 温度报警判定: 0 正常 / 1 超上限 / -1 超下限
int check_temp_alarm(float temp_celsius, float low, float high);

}  // namespace airship_lora

#endif  // AIRSHIP_LORA__MODBUS_RTU_HPP_
