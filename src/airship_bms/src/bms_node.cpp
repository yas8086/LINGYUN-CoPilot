// 灵云01号伴飞电脑 — 锂电池 BMS 驱动节点
// 协议: bms_ems_v01 (CAN DBC)
// 功能:
//   1. 接收并解析 BMS 上报的 CAN 帧
//   2. 解析单体电压帧 (0x80003xxx, 每帧5节) 与温度帧 (0x80004xxx, 每帧8点)
//   3. 解析故障帧 (ErrorCode1: 0x80001fd0)
//   4. 发布 BmsStatus
//
// 注意: BMS 协议字段较多, 部分总线/单体映射随工程配置变化。
//       此处已实现核心帧; 其余字段待厂家 DBC 精确定位后补充。
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "airship_can/can_interface.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_utils/can_utils.hpp"

using airship_can::CanFrame;
using airship_can::SocketCanInterface;

// BMS 协议帧 ID 基址 (来自官方 DBC bms_ems_v01)
constexpr uint32_t kBattInfo02 = 0x80001100;   // 总压/总电流/SOC
constexpr uint32_t kBattInfo01 = 0x80001000;   // 运行状态/告警级别
constexpr uint32_t kCellVoltageBase = 0x80003000;  // 单体电压帧 (每帧5节)
constexpr uint32_t kCellTempBase = 0x80004000;     // 单体温度帧 (每帧8点)
constexpr uint32_t kFaultId = 0x80001fd0;          // 故障/告警帧

// 每帧固定节数
constexpr uint32_t kCellPerVoltFrame = 5;
constexpr uint32_t kCellPerTempFrame = 8;

class BmsNode : public rclcpp::Node
{
public:
  explicit BmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("bms_node", options),
    can_("can0"),
    running_(false)
  {
    // ===== 参数 =====
    can_if_ = this->declare_parameter("can_interface", std::string("can0"));
    cell_count_ = static_cast<uint16_t>(this->declare_parameter("cell_count", 0));

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::BmsStatus>("/bms/status", rclcpp::QoS(10));

    can_ = SocketCanInterface(can_if_);
    if (!can_.open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CAN 接口 %s 失败, 稍后重试", can_if_.c_str());
    }

    running_ = true;
    receive_thread_ = std::thread(&BmsNode::receive_loop, this);
  }

  ~BmsNode() override
  {
    running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    can_.close();
  }

private:
  // 接收线程: 按帧 ID 路由解析
  void receive_loop()
  {
    while (running_.load()) {
      CanFrame frame{};
      if (!can_.receive(frame, 100)) {
        continue;
      }
      if (frame.id == kBattInfo02) {
        parse_batt_info02(frame);
      } else if (frame.id == kBattInfo01) {
        parse_batt_info01(frame);
      } else if ((frame.id >= kCellVoltageBase) && (frame.id < kCellVoltageBase + 0x100)) {
        parse_cell_voltage_frame(frame);
      } else if ((frame.id >= kCellTempBase) && (frame.id < kCellTempBase + 0x100)) {
        parse_cell_temp_frame(frame);
      } else if (frame.id == kFaultId) {
        parse_error_frame(frame);
      }
    }
  }

  // 解析 BattInfo02 (0x80001100): 总压/总电流/SOC
  void parse_batt_info02(const CanFrame & frame)
  {
    using airship_utils::get_u16_le;
    using airship_utils::scale_u16;
    // BattVolt: 0.1V/bit; BattCurr: 0.1A/bit, offset=-100V; SOC: 0.1%/bit
    pack_voltage_ = scale_u16(get_u16_le(frame.data, 0), 0.1f);
    pack_current_ = scale_u16(get_u16_le(frame.data, 2), 0.1f) - 100.0f;
    soc_ = scale_u16(get_u16_le(frame.data, 4), 0.1f);
    publish_status();
  }

  // 解析 BattInfo01 (0x80001000): 告警级别/运行状态
  void parse_batt_info01(const CanFrame & frame)
  {
    using airship_utils::get_u8;
    // BMS_AlarmLevel: byte6 低8位
    alarm_level_ = get_u8(frame.data, 6);
    publish_status();
  }

  // 解析单体电压帧: 每帧5节, 每节2字节(0.1mV? 协议未定, 先用比例因子 1.0 占位)
  // TODO: 确认单体电压分辨率后修正 scale
  void parse_cell_voltage_frame(const CanFrame & frame)
  {
    using airship_utils::get_u16_le;
    const uint32_t frame_idx = frame.id - kCellVoltageBase;
    const uint32_t cell_start = frame_idx * kCellPerVoltFrame;
    for (uint32_t i = 0; i < kCellPerVoltFrame; ++i) {
      const uint32_t idx = cell_start + i;
      if (idx >= 256) {
        break;
      }
      cell_voltages_[idx] = static_cast<float>(get_u16_le(frame.data, i * 2)) * 0.001f;  // mV->V
    }
    publish_status();
  }

  // 解析单体温度帧: 每帧8点, 每点2字节(0.1℃)
  void parse_cell_temp_frame(const CanFrame & frame)
  {
    using airship_utils::get_u16_le;
    const uint32_t frame_idx = frame.id - kCellTempBase;
    const uint32_t cell_start = frame_idx * kCellPerTempFrame;
    for (uint32_t i = 0; i < kCellPerTempFrame; ++i) {
      const uint32_t idx = cell_start + i;
      if (idx >= 256) {
        break;
      }
      cell_temps_[idx] = static_cast<float>(get_u16_le(frame.data, i * 2)) * 0.1f;
    }
    publish_status();
  }

  // 解析故障帧 (ErrorCode1)
  void parse_error_frame(const CanFrame & frame)
  {
    using airship_utils::get_u16_le;
    // 帧结构: 根据 DBC, 前两个字节含告警级别/故障字
    fault_word1_ = get_u16_le(frame.data, 2);
    alarm_level_ = frame.data[0];
    publish_status();
  }

  // 发布聚合状态
  void publish_status()
  {
    auto msg = airship_msgs::msg::BmsStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.pack_voltage = pack_voltage_;
    msg.pack_current = pack_current_;
    msg.soc = soc_;
    msg.cell_voltages = cell_voltages_;
    msg.cell_temps = cell_temps_;

    // 计算最高/最低单体电压与压差(基于已收到数据)
    msg.max_cell_voltage = std::numeric_limits<float>::quiet_NaN();
    msg.min_cell_voltage = std::numeric_limits<float>::quiet_NaN();
    msg.cell_voltage_diff = 0.0f;
    for (uint32_t i = 0; i < 256; ++i) {
      if (std::isfinite(cell_voltages_[i])) {
        if (std::isnan(msg.max_cell_voltage) || cell_voltages_[i] > msg.max_cell_voltage) {
          msg.max_cell_voltage = cell_voltages_[i];
        }
        if (std::isnan(msg.min_cell_voltage) || cell_voltages_[i] < msg.min_cell_voltage) {
          msg.min_cell_voltage = cell_voltages_[i];
        }
      }
    }
    if (!std::isnan(msg.max_cell_voltage) && !std::isnan(msg.min_cell_voltage)) {
      msg.cell_voltage_diff = msg.max_cell_voltage - msg.min_cell_voltage;
    }

    msg.fault_word1 = fault_word1_;
    msg.alarm_level = alarm_level_;
    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  std::string can_if_;
  SocketCanInterface can_;
  std::atomic<bool> running_;
  std::thread receive_thread_;

  uint16_t cell_count_;
  std::array<float, 256> cell_voltages_ = {};
  std::array<float, 256> cell_temps_ = {};
  float pack_voltage_ = 0.0f;
  float pack_current_ = 0.0f;
  float soc_ = 0.0f;
  uint16_t fault_word1_ = 0;
  uint8_t alarm_level_ = 0;

  rclcpp::Publisher<airship_msgs::msg::BmsStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BmsNode>());
  rclcpp::shutdown();
  return 0;
}
