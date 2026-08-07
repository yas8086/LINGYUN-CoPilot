// 灵云01号伴飞电脑 — DCDC 电源模块驱动节点
// 协议: 昊瑞昌 HRC-GFD360-48-4K (280-500V输入, 4KW, 输出48V)
//
// 功能:
//   1. 周期性下发电源控制帧 (0x18EF3010, 200ms) 维持开机/设定电压/限流
//   2. 周期性查询模拟量 (0x18D80047)
//   3. 接收状态帧与模拟量回应, 解析后发布 DcdcStatus
//
// 协议解析逻辑见 airship_dcdc::dcdc_protocol (纯库, 可 gtest)。
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "airship_can/can_interface.hpp"
#include "airship_dcdc/dcdc_protocol.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"

using airship_can::CanFrame;
using airship_can::SocketCanInterface;
using airship_dcdc::DcdcData;

class DcdcNode : public rclcpp::Node
{
public:
  explicit DcdcNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("dcdc_node", options),
    can_("can0"),
    running_(false)
  {
    // ===== 参数 =====
    can_if_ = this->declare_parameter("can_interface", std::string("can0"));
    control_rate_hz_ = this->declare_parameter("control_rate_hz", 5.0);   // 200ms
    set_voltage_ = this->declare_parameter("set_voltage", 48.0);          // 输出 [V]
    set_current_ = this->declare_parameter("set_current", 80.0);          // 限流 [A]
    dcdc_enabled_ = this->declare_parameter("dcdc_enabled", true);        // 是否开机

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::DcdcStatus>("/dcdc/status", rclcpp::QoS(10));

    can_ = SocketCanInterface(can_if_);
    if (!can_.open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CAN 接口 %s 失败, 稍后重试", can_if_.c_str());
    }

    // 启动收发线程
    running_ = true;
    control_thread_ = std::thread(&DcdcNode::control_loop, this);
    receive_thread_ = std::thread(&DcdcNode::receive_loop, this);
  }

  ~DcdcNode() override
  {
    running_ = false;
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    can_.close();
  }

private:
  // 控制帧线程: 周期下发电源控制帧(必须持续发送) 与模拟量查询
  void control_loop()
  {
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_hz_));
    while (running_.load()) {
      if (can_.is_open()) {
        can_.send(airship_dcdc::build_control_frame(
            dcdc_enabled_, static_cast<float>(set_voltage_), static_cast<float>(set_current_)));
        can_.send(airship_dcdc::build_analog_query_frame());
      }
      std::this_thread::sleep_for(period);
    }
  }

  // 接收线程: 解析状态帧与模拟量回应
  void receive_loop()
  {
    while (running_.load()) {
      CanFrame frame{};
      if (!can_.receive(frame, 100)) {
        continue;
      }
      if (frame.id == airship_dcdc::kStatusId) {
        airship_dcdc::parse_status(frame.data, dcdc_data_);
      } else if (frame.id == airship_dcdc::kAnalogRespId) {
        airship_dcdc::parse_analog(frame.data, dcdc_data_);
      } else {
        continue;
      }
      publish_status();
    }
  }

  // 发布聚合状态
  void publish_status()
  {
    auto msg = airship_msgs::msg::DcdcStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.input_voltage = dcdc_data_.input_voltage;
    msg.output_voltage = dcdc_data_.output_voltage;
    msg.output_current = dcdc_data_.output_current;
    msg.output_power = dcdc_data_.output_voltage * dcdc_data_.output_current;
    msg.ambient_temp = dcdc_data_.ambient_temp;
    msg.heatsink_temp = dcdc_data_.heatsink_temp;
    msg.output_enabled = dcdc_data_.output_enabled;
    msg.fault_word = dcdc_data_.fault_word;
    msg.set_voltage = static_cast<float>(set_voltage_);
    msg.set_current = static_cast<float>(set_current_);
    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  std::string can_if_;
  SocketCanInterface can_;
  std::atomic<bool> running_;
  std::thread control_thread_;
  std::thread receive_thread_;

  double control_rate_hz_;
  double set_voltage_;
  double set_current_;
  bool dcdc_enabled_;

  DcdcData dcdc_data_;

  rclcpp::Publisher<airship_msgs::msg::DcdcStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DcdcNode>());
  rclcpp::shutdown();
  return 0;
}