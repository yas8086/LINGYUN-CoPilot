// 灵云01号伴飞电脑 — DCDC 电源模块驱动节点
// 协议: 昊瑞康 HRC-GFD360-48-4K (280-500V输入, 4KW, 输出48V)
//
// 职责边界:
//   - 本节点仅监控: 接收状态帧与模拟量回应, 解析后发布 DcdcStatus
//   - 控制保活(周期下发电源控制帧 + 模拟量查询帧)由独立 dcdc_hold 进程负责,
//     以保证 ROS2 崩溃时 DCDC 仍保持输出。二者不再双发, 避免总线冗余。
//
// 协议解析逻辑见 airship_dcdc::dcdc_protocol (纯库, 可 gtest)。
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "airship_can/can_interface.hpp"
#include "airship_dcdc/dcdc_protocol.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/safety_status.hpp"

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
    // 保留 control_rate_hz 参数声明以兼容旧配置(控制保活实际由 dcdc_hold 承担)
    this->declare_parameter("control_rate_hz", 5.0);
    set_voltage_ = this->declare_parameter("set_voltage", 48.0);          // 输出 [V]
    set_current_ = this->declare_parameter("set_current", 80.0);          // 限流 [A]
    dcdc_enabled_ = this->declare_parameter("dcdc_enabled", true);        // 是否开机
    // 实际控制值以 dcdc_hold 进程的环境变量为准(单一配置源):
    // 若环境变量存在则覆盖 ROS 参数, 使本节点上报的 set_voltage/set_current
    // 与 dcdc_hold 实际下发到 DCDC 的值一致, 避免状态消息误导监控。
    const char * env_v = std::getenv("DCDC_SET_VOLTAGE");
    if (env_v != nullptr && env_v[0] != '\0') {
      set_voltage_ = std::strtod(env_v, nullptr);
    }
    env_v = std::getenv("DCDC_SET_CURRENT");
    if (env_v != nullptr && env_v[0] != '\0') {
      set_current_ = std::strtod(env_v, nullptr);
    }
    // 无数据超时 (s): 超过该时长未收到任何有效帧, 判定设备离线, 兜底发布 online=false
    link_timeout_s_ = this->declare_parameter("link_timeout_s", 3.0);
    if (link_timeout_s_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "link_timeout_s 非法值 %.3f, 重置为 3.0", link_timeout_s_);
      link_timeout_s_ = 3.0;
    }

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::DcdcStatus>("/dcdc/status", rclcpp::QoS(10));

    // ===== 安全状态订阅 (记录日志, 不改变控制行为) =====
    safety_sub_ = this->create_subscription<airship_msgs::msg::SafetyStatus>(
      "/safety/status", rclcpp::QoS(10),
      std::bind(&DcdcNode::on_safety, this, std::placeholders::_1));

    can_ = SocketCanInterface(can_if_);
    if (!can_.open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CAN 接口 %s 失败, 稍后重试", can_if_.c_str());
    }

    // 启动接收线程(监控)
    running_ = true;
    last_data_time_ = this->now();
    last_offline_pub_time_ = this->now();
    receive_thread_ = std::thread(&DcdcNode::receive_loop, this);
  }

  ~DcdcNode() override
  {
    running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    can_.close();
  }

private:
  // 接收线程: 解析状态帧与模拟量回应 (来自 dcdc_hold 的查询触发)
  void receive_loop()
  {
    while (running_.load()) {
      if (!can_.ensure_open()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      CanFrame frame{};
      if (!can_.receive(frame, 100)) {
        // 无数据超时兜底: 距上次有效数据超过 link_timeout_s 后周期发布 online=false,
        // 让下游能感知设备失联(而非停留在最后一次旧数据)。
        const auto now = this->now();
        if ((now - last_data_time_).seconds() > link_timeout_s_ &&
          (now - last_offline_pub_time_).seconds() >= link_timeout_s_)
        {
          last_offline_pub_time_ = now;
          publish_status(false);
        }
        continue;
      }
      last_data_time_ = this->now();
      if (frame.id == airship_dcdc::kStatusId) {
        airship_dcdc::parse_status(frame.data, frame.len, dcdc_data_);
      } else if (frame.id == airship_dcdc::kAnalogRespId) {
        airship_dcdc::parse_analog(frame.data, frame.len, dcdc_data_);
      } else {
        continue;
      }
      publish_status(true);
    }
  }

  // 安全状态回调: 记录日志 (仅监控, 不改变控制)
  void on_safety(const airship_msgs::msg::SafetyStatus::SharedPtr msg)
  {
    if (msg->safe_to_control != last_safe_to_control_) {
      last_safe_to_control_ = msg->safe_to_control;
      if (msg->safe_to_control) {
        RCLCPP_INFO(this->get_logger(), "安全状态: 允许下发控制 (safe_to_control=true)");
      } else {
        RCLCPP_WARN(
          this->get_logger(), "安全状态: 禁止下发控制 (safe_to_control=false): %s",
          msg->reason.empty() ? "原因未知" : msg->reason.c_str());
      }
    }
  }

  // 发布聚合状态
  void publish_status(bool online)
  {
    auto msg = airship_msgs::msg::DcdcStatus();
    msg.header.stamp = this->now();
    msg.online = online;
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
  std::thread receive_thread_;

  double set_voltage_;
  double set_current_;
  bool dcdc_enabled_;
  double link_timeout_s_;

  DcdcData dcdc_data_;
  rclcpp::Time last_data_time_;
  rclcpp::Time last_offline_pub_time_;

  rclcpp::Publisher<airship_msgs::msg::DcdcStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<airship_msgs::msg::SafetyStatus>::SharedPtr safety_sub_;
  bool last_safe_to_control_ = true;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DcdcNode>());
  rclcpp::shutdown();
  return 0;
}