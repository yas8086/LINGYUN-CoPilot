// 灵云01号伴飞电脑 — 锂电池 BMS CAN 协议解析
// 协议: bms_ems_v01 (CAN DBC, 三元锂 102 串)
//
// 本模块为纯 C++ 协议解析库(无 ROS 依赖),便于 gtest 单元测试。
// 注意: 帧 ID 为 SocketCAN 收到的 29 位扩展帧 ID(已做 0x1FFFFFFF 掩码)。
//
// 关键报文:
//   0x001400 BattInfo02        总压/总电流/SOC/RealSoc (小端 u16)
//   0x001300 BattInfo01        运行状态/连接状态/绝缘电阻/报警级别
//   0x003000+ CellVoltage_XX   逐节单体电压 (每帧5节, 12位信号, 0.001V@+1V)
//   0x001500 CellTempStatistic 温度统计 (max/min/avg, 1℃@-50)
//   0x002110 PACKTemp          极耳温度 8 点 (1℃@-50)
#ifndef AIRSHIP_BMS__BMS_PROTOCOL_HPP_
#define AIRSHIP_BMS__BMS_PROTOCOL_HPP_

#include <array>
#include <cstdint>

namespace airship_bms
{

// 报文帧 ID (29 位扩展帧, 掩码后)
constexpr uint32_t kBattInfo02 = 0x001400;   // 总压/总电流/SOC
constexpr uint32_t kBattInfo01 = 0x001300;   // 运行状态/报警级别
constexpr uint32_t kCellVoltageBase = 0x003000;  // 单体电压帧基址 (每帧 +0x10)
constexpr uint32_t kCellTempStatistic = 0x001500;  // 温度统计
constexpr uint32_t kPackTemp = 0x002110;     // 极耳温度 8 点
constexpr uint32_t kErrorCode = 0x001FE0;    // 故障/告警字 (64 位, 见 ErrorCode)
constexpr uint32_t kSoh = 0x005FF0;          // SOH/总容量/循环次数/额定电压
constexpr uint32_t kSop = 0x001100;          // SOP 功率限制 (充放电电流/电压限值)
constexpr uint32_t kCellVoltStatistic = 0x001200;  // 单体电压统计 (max/min/avg)
constexpr uint32_t kPoleTempStatistic = 0x001700;  // 极柱温度统计

// 每帧固定节数
constexpr uint32_t kCellPerVoltFrame = 5;
constexpr uint32_t kMaxCells = 256;
constexpr uint32_t kMaxPoleTemps = 8;

// 解析结果聚合 (与 BmsStatus 字段对应, 但无 ROS 依赖)
struct BmsData
{
  // 总状态 (BattInfo02)
  float pack_voltage = 0.0f;      // 总电压 [V]
  float pack_current = 0.0f;      // 总电流 [A] (含 -100A 偏移)
  float soc = 0.0f;               // SOC [%]
  float real_soc = 0.0f;          // 真实 SOC [%]
  // 运行/告警 (BattInfo01)
  uint8_t run_state = 0;              // 运行状态
  uint8_t connection_status = 0;      // 连接状态
  uint16_t positive_insulation_kohm = 0;  // 正极绝缘电阻 [kΩ]
  uint16_t negative_insulation_kohm = 0;  // 负极绝缘电阻 [kΩ]
  uint8_t alarm_level = 0;            // 告警级别
  // 单体电压 (CellVoltage_XX)
  std::array<float, kMaxCells> cell_voltages = {};  // [V]
  // 温度统计 (CellTempStatistic)
  float max_cell_temp = 0.0f;     // 最高单体温度 [℃]
  float min_cell_temp = 0.0f;     // 最低单体温度 [℃]
  float avg_cell_temp = 0.0f;     // 平均单体温度 [℃]
  float temp_diff = 0.0f;         // 温差 [℃]
  // 极耳温度 (PACKTemp)
  std::array<float, kMaxPoleTemps> pole_temps = {};  // [℃]
  // 故障/告警字 (ErrorCode, 64 个 1-bit 信号; 数字越大越严重)
  uint32_t fault_word1 = 0;   // 一级报警 Warn (bit16-31) | 系统故障位 (bit0-15)
  uint32_t fault_word2 = 0;   // 二级报警 Alarm (bit32-47)
  uint32_t fault_word3 = 0;   // 三级报警 CriticalAlarm (bit48-63)
  // 健康/容量 (SOH)
  float soh = 0.0f;                 // 健康状态 [%]
  float pack_total_cap = 0.0f;      // 电池总容量 [AH]
  uint16_t charge_times = 0;        // 循环次数
  float pack_rated_voltage = 0.0f;  // 额定电压 [V]
  // 功率限制 (SOP)
  float max_charge_current = 0.0f;  // 最大充电电流 [A]
  float max_discharge_current = 0.0f;  // 最大放电电流 [A]
  float charge_max_cell_volt = 0.0f;  // 充电单节电压上限 [V]
  float discharge_min_cell_volt = 0.0f;  // 放电单节电压下限 [V]
  // 单体电压统计 (CellVoltStatistic)
  float stat_max_cell_volt = 0.0f;  // 统计最高单体电压 [V]
  float stat_min_cell_volt = 0.0f;  // 统计最低单体电压 [V]
  float stat_avg_cell_volt = 0.0f;  // 统计平均单体电压 [V]
  // 极柱温度统计 (PoleTempStatistic)
  float pole_max_temp = 0.0f;   // 极柱最高温度 [℃]
  float pole_min_temp = 0.0f;   // 极柱最低温度 [℃]
  float pole_temp_diff = 0.0f;  // 极柱温差 [℃]
};

// 解析 BattInfo02: 总压/总电流/SOC/RealSoc (len 为 DLC, 不足 8 字节时忽略并返回)
void parse_batt_info(const uint8_t * data, uint32_t len, BmsData & out);
// 解析 BattInfo01: 运行状态/连接状态/绝缘电阻/报警级别
void parse_batt_status(const uint8_t * data, uint32_t len, BmsData & out);
// 解析 CellVoltage_XX: 逐节单体电压, frame_id 用于定位起始节号
void parse_cell_voltage(uint32_t frame_id, const uint8_t * data, uint32_t len, BmsData & out);
// 解析 CellTempStatistic: 温度统计
void parse_cell_temp_statistic(const uint8_t * data, uint32_t len, BmsData & out);
// 解析 PACKTemp: 极耳温度 8 点
void parse_pack_temp(const uint8_t * data, uint32_t len, BmsData & out);
// 解析 ErrorCode: 64 个 1-bit 故障/告警位 -> fault_word1/2/3
void parse_error_code(const uint8_t * data, uint32_t len, BmsData & out);
// 解析 SOH: SOH/总容量/循环次数/额定电压 (均为 0.1 缩放 u16 小端)
void parse_soh(const uint8_t * data, uint32_t len, BmsData & out);
// 解析 SOP: 最大充/放电电流与单节电压限值 (0.1 缩放 u16 小端)
void parse_sop(const uint8_t * data, uint32_t len, BmsData & out);
// 解析 CellVoltStatistic: 平均/最大/最小单体电压 (12-bit Intel 小端, 0.001V@+1V)
void parse_cell_volt_statistic(const uint8_t * data, uint32_t len, BmsData & out);
// 解析 PoleTempStatistic: 极柱最高/最低温度与温差 (1℃@-50)
void parse_pole_temp_statistic(const uint8_t * data, uint32_t len, BmsData & out);

}  // namespace airship_bms

#endif  // AIRSHIP_BMS__BMS_PROTOCOL_HPP_