// 灵云01号伴飞电脑 — DCDC 电源模块驱动节点
// 协议: 昊瑞昌 HRC-GFD360-48-4K (280-500V输入, 4KW, 输出48V)
// 功能:
//   1. 周期性下发电源控制帧 (0x18EF3010, 200ms) 维持开机/设定电压/限流
//   2. 接收电源状态帧 (0x18FF3247, 500ms) 解析故障状态
//   3. 周期性查询模拟量 (0x18D80047) 获取输入/输出电压/电流/温度
//   4. 解析后发布 DcdcStatus
#include <atomic>
#include <cstring>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "airship_can/can_interface.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_utils/can_utils.hpp"

using airship_can::CanFrame;
using airship_can::SocketCanInterface;

// 昊瑞昌 DCDC 协议常量
constexpr uint32_t kPowerControlId = 0x18EF3010;  // VMS->DCDC 电源控制帧
constexpr uint32_t kStatusId = 0x18FF3247;        // DCDC->VMS 电源状态帧
constexpr uint32_t kAnalogQueryId = 0x18D80047;   // VMS->DCDC 模拟量查询
constexpr uint32_t kAnalogRespId = 0x18D84700;    // DCDC->VMS 模拟量回应

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
    query_rate_hz_ = this->declare_parameter("query_rate_hz", 1.0);       // 1000ms
    set_voltage_ = this->declare_parameter("set_voltage", 48.0);          // 输出 [V]
    set_current_ = this->declare_parameter("set_current", 80.0);          // 限流 [A]
    dcdc_enabled_ = this->declare_parameter("dcdc_enabled", true);        // 是否开机

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::DcdcStatus>("/dcdc/status", rclcpp::QoS(10));

    // CAN 接口在构造后打开; 失败仅告警, 由看门狗重试
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
  // 构造电源控制帧: 开机 + 设定电压 + 限流
  CanFrame build_control_frame()
  {
    CanFrame frame{};
    frame.id = kPowerControlId;
    frame.extended = true;
    frame.len = 8;
    // Byte2: Bit5~4 = 01(开机) or 00(关机)
    frame.data[2] = dcdc_enabled_ ? 0x10 : 0x00;
    // Byte3~4: 输出电压 (0.1V/BIT, 小端)
    const uint16_t v = static_cast<uint16_t>(set_voltage_ * 10.0f);
    frame.data[3] = static_cast<uint8_t>(v & 0xFF);
    frame.data[4] = static_cast<uint8_t>((v >> 8) & 0xFF);
    // Byte5~6: 输出限流 (0.1A/BIT, 小端)
    const uint16_t i = static_cast<uint16_t>(set_current_ * 10.0f);
    frame.data[5] = static_cast<uint8_t>(i & 0xFF);
    frame.data[6] = static_cast<uint8_t>((i >> 8) & 0xFF);
    return frame;
  }

  // 构造模拟量查询帧
  CanFrame build_analog_query_frame()
  {
    CanFrame frame{};
    frame.id = kAnalogQueryId;
    frame.extended = true;
    frame.len = 8;
    return frame;
  }

  // 控制帧线程: 周期下发电源控制帧(必须持续发送, 否则 DCDC 可能停机)
  void control_loop()
  {
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_hz_));
    while (running_.load()) {
      if (can_.is_open()) {
        can_.send(build_control_frame());
        can_.send(build_analog_query_frame());
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
      if (frame.id == kStatusId) {
        parse_status_frame(frame);
      } else if (frame.id == kAnalogRespId) {
        parse_analog_frame(frame);
      }
    }
  }

  // 解析电源状态帧 (0x18FF3247)
  void parse_status_frame(const CanFrame & frame)
  {
    auto msg = airship_msgs::msg::DcdcStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.can_id = frame.id;
    // Byte0: 故障状态字节
    msg.fault_word = frame.data[0];
    // Bit2: 输出状态 (0停机 1开机)
    msg.output_enabled = (frame.data[0] & 0x04) != 0;
    // (设定值来自本地参数, 此处填充)
    msg.set_voltage = set_voltage_;
    msg.set_current = set_current_;
    status_pub_->publish(msg);
  }

  // 解析模拟量回应帧 (0x18D84700)
  void parse_analog_frame(const CanFrame & frame)
  {
    using airship_utils::get_u16_le;
    using airship_utils::scale_u16;
    using airship_utils::temp_with_offset;

    auto msg = airship_msgs::msg::DcdcStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.can_id = frame.id;
    // Byte0~1: 输入电压 (1V/BIT)
    msg.input_voltage = scale_u16(get_u16_le(frame.data, 0), 1.0f);
    // Byte2~3: 输出电压 (0.1V/BIT)
    msg.output_voltage = scale_u16(get_u16_le(frame.data, 2), 0.1f);
    // Byte4~5: 输出电流 (0.1A/BIT)
    msg.output_current = scale_u16(get_u16_le(frame.data, 4), 0.1f);
    // Byte6: 环境温度 (1℃/BIT, -40偏移)
    msg.ambient_temp = temp_with_offset(static_cast<int8_t>(frame.data[6]));
    // Byte7: 散热器温度
    msg.heatsink_temp = temp_with_offset(static_cast<int8_t>(frame.data[7]));
    // 输出功率 (计算值)
    msg.output_power = msg.output_voltage * msg.output_current;
    msg.set_voltage = set_voltage_;
    msg.set_current = set_current_;
    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  std::string can_if_;
  SocketCanInterface can_;
  std::atomic<bool> running_;
  std::thread control_thread_;
  std::thread receive_thread_;

  double control_rate_hz_;
  double query_rate_hz_;
  double set_voltage_;
  double set_current_;
  bool dcdc_enabled_;

  rclcpp::Publisher<airship_msgs::msg::DcdcStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DcdcNode>());
  rclcpp::shutdown();
  return 0;
}
