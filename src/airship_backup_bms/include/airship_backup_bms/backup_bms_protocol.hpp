// 灵云01号伴飞电脑 — 12S 备用电源 BMS 串口协议解析 (北辰串口协议 V1.4)
//
// 本模块为纯 C++ 协议库(无 ROS 依赖),便于 gtest 单元测试。
//
// 帧格式:
//   发送 0x56 [地址][主机][读写][指令][长度H][长度L][数据][CRC-H][CRC-L]
//   响应 0x57 [地址][主机][读写][指令][长度H][长度L][数据][CRC-H][CRC-L]
//     - 地址默认 0xFF; 主机(上位机)=0x10; 读写 0=读
//     - 长度(2 字节, 高在前)只包含数据长度 L
//     - CRC16(0xA001, 初值 0xFFFF) 覆盖 "地址..数据" 所有字节, 高字节在前
//
// 轮询指令:
//   0x06 基本信息1  总压/总电流/SOC/SOH/电压温度统计/告警/保护/故障/系统状态
//   0x07 单节温度   温度数量 + 逐点温度(0.1℃ 有符号)
//   0x08 单节电压   单体数量 + 逐节电压(1mV)
#ifndef AIRSHIP_BACKUP_BMS__BACKUP_BMS_PROTOCOL_HPP_
#define AIRSHIP_BACKUP_BMS__BACKUP_BMS_PROTOCOL_HPP_

#include <cstdint>
#include <vector>

namespace airship_backup_bms
{

// ===== 帧常量 =====
constexpr uint8_t kReqStart = 0x56;    // 请求起始字节
constexpr uint8_t kRespStart = 0x57;   // 响应起始字节
constexpr uint8_t kDefaultAddr = 0xFF; // 默认从机地址
constexpr uint8_t kHostUpper = 0x10;   // 主机信息: 上位机
constexpr uint8_t kCmdRead = 0x00;     // 读写标志: 读

// ===== 指令号 =====
constexpr uint8_t kCmdBasicInfo = 0x06;   // 基本信息1
constexpr uint8_t kCmdCellTemp = 0x07;    // 单节温度
constexpr uint8_t kCmdCellVoltage = 0x08; // 单节电压

// ===== 解析结果聚合 (与 BackupBmsStatus 字段对应, 无 ROS 依赖) =====
struct BackupBmsData
{
  // 基本信息1 (0x06)
  float pack_voltage = 0.0f;      // 总电压 [V]
  float pack_current = 0.0f;      // 总电流 [A] (充电为正)
  float soc = 0.0f;               // SOC [%]
  float soh = 0.0f;               // SOH [%]
  float max_cell_voltage = 0.0f;  // 最高单体电压 [V]
  float min_cell_voltage = 0.0f;  // 最低单体电压 [V]
  float cell_voltage_diff = 0.0f; // 单体压差 [V]
  float max_cell_temp = 0.0f;     // 最高单体温度 [℃]
  float min_cell_temp = 0.0f;     // 最低单体温度 [℃]
  float avg_cell_temp = 0.0f;     // 平均单体温度 [℃]
  float temp_diff = 0.0f;         // 单体温差 [℃]
  uint32_t alarm_word = 0;        // 告警标志位
  uint32_t protect_word = 0;      // 保护标志位
  uint32_t fault_word = 0;        // 故障标志位
  uint32_t system_word = 0;       // 系统状态1
  // 单节电压 (0x08) / 单节温度 (0x07)
  std::vector<float> cell_voltages;  // [V]
  std::vector<float> cell_temps;     // [℃]
};

// CRC16 (0xA001, 初值 0xFFFF)
uint16_t crc16(const uint8_t * buf, uint32_t len);

// 构建读请求帧 (含 CRC)
std::vector<uint8_t> build_read_request(uint8_t addr, uint8_t host, uint8_t cmd);

// 校验并切分响应帧: 校验起始/地址/主机/读写/指令/长度/CRC。
// 成功时 data/data_len 指向数据段。
bool parse_response_frame(
  const uint8_t * frame, uint32_t len, uint8_t addr, uint8_t host,
  uint8_t expect_cmd, const uint8_t ** data, uint32_t * data_len);

// 解析基本信息1 (0x06)。len 不足关键字段时按可用边界安全解析, 返回是否读到总压等核心数据。
bool parse_basic_info(const uint8_t * data, uint32_t len, BackupBmsData & out);

// 解析单节电压 (0x08): data[0..1]=数量, 其后每 2 字节一节 1mV
bool parse_cell_voltages(const uint8_t * data, uint32_t len, BackupBmsData & out);

// 解析单节温度 (0x07): data[0..1]=数量, 其后每 2 字节一点 0.1℃ 有符号
bool parse_cell_temps(const uint8_t * data, uint32_t len, BackupBmsData & out);

}  // namespace airship_backup_bms

#endif  // AIRSHIP_BACKUP_BMS__BACKUP_BMS_PROTOCOL_HPP_
