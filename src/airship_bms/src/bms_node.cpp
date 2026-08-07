// 灵云01号伴飞电脑 — 锂电池 BMS 驱动节点
// 协议: bms_ems_v01 (CAN DBC)
//
// 功能:
//   1. 接收并解析 BMS 上报的 CAN 帧
//   2. 按帧 ID 路由到 bms_protocol 解析
//   3. 累积结果, 发布 BmsStatus
//
// 协议解析逻辑见 airship_bms::bms_protocol (纯库, 可 gtest)。
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "airship_bms/bms_protocol.hpp"
#include "airship_can/can_interface.hpp"
#include "airship_msgs/msg/bms_status.hpp"

using airship_bms::BmsData;
using airship_can::CanFrame;
using airship_can::SocketCanInterface;

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
      using namespace airship_bms;  // NOLINT
      if (frame.id == kBattInfo02) {
        parse_batt_info(frame.data, bms_data_);
      } else if (frame.id == kBattInfo01) {
        parse_batt_status(frame.data, bms_data_);
      } else if (frame.id == kCellTempStatistic) {
        parse_cell_temp_statistic(frame.data, bms_data_);
      } else if (frame.id == kPackTemp) {
        parse_pack_temp(frame.data, bms_data_);
      } else if ((frame.id >= kCellVoltageBase) &&
        (frame.id < kCellVoltageBase + 0x100))
      {
        parse_cell_voltage(frame.id, frame.data, bms_data_);
      } else {
        continue;
      }
      publish_status();
    }
  }

  // 发布聚合状态
  void publish_status()
  {
    auto msg = airship_msgs::msg::BmsStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.pack_voltage = bms_data_.pack_voltage;
    msg.pack_current = bms_data_.pack_current;
    msg.soc = bms_data_.soc;
    msg.cell_voltages = bms_data_.cell_voltages;
    msg.fault_word1 = 0;
    msg.alarm_level = bms_data_.alarm_level;

    // 计算最高/最低单体电压与压差(基于已收到数据)
    msg.max_cell_voltage = std::numeric_limits<float>::quiet_NaN();
    msg.min_cell_voltage = std::numeric_limits<float>::quiet_NaN();
    msg.cell_voltage_diff = 0.0f;
    for (uint32_t i = 0; i < cell_count_; ++i) {
      const float v = bms_data_.cell_voltages[i];
      if (v <= 0.0f) {
        continue;  // 未收到该节数据
      }
      if (std::isnan(msg.max_cell_voltage) || v > msg.max_cell_voltage) {
        msg.max_cell_voltage = v;
      }
      if (std::isnan(msg.min_cell_voltage) || v < msg.min_cell_voltage) {
        msg.min_cell_voltage = v;
      }
    }
    if (!std::isnan(msg.max_cell_voltage) && !std::isnan(msg.min_cell_voltage)) {
      msg.cell_voltage_diff = msg.max_cell_voltage - msg.min_cell_voltage;
    }

    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  std::string can_if_;
  SocketCanInterface can_;
  std::atomic<bool> running_;
  std::thread receive_thread_;

  uint16_t cell_count_;
  BmsData bms_data_;

  rclcpp::Publisher<airship_msgs::msg::BmsStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BmsNode>());
  rclcpp::shutdown();
  return 0;
}