// 灵云01号伴飞电脑 — 设备监控聚合节点
// 功能:
//   1. 订阅 BMS/MPPT/DCDC 状态, 记录各设备最近更新时间
//   2. 看门狗定时器: 检测设备离线(超时未收到数据), 发布 DeviceAlert
//   3. 设备状态变化告警(如 DCDC 故障位触发) 生成 DeviceAlert
//   4. 输出统一日志
//
// 输出话题:
//   /monitor/device_alert   (airship_msgs/DeviceAlert)
#include <cmath>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "airship_msgs/msg/bms_status.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/device_alert.hpp"
#include "airship_msgs/msg/mppt_status.hpp"

using std::placeholders::_1;

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode()
  : rclcpp::Node("monitor_node")
  {
    // ===== 参数 =====
    watchdog_hz_ = this->declare_parameter("watchdog_rate_hz", 1.0);   // 看门狗频率
    timeout_s_ = this->declare_parameter("link_timeout_s", 3.0);       // 离线判定超时

    // ===== 发布器 =====
    alert_pub_ = this->create_publisher<airship_msgs::msg::DeviceAlert>(
      "/monitor/device_alert", rclcpp::QoS(10));

    // ===== 订阅 =====
    bms_sub_ = this->create_subscription<airship_msgs::msg::BmsStatus>(
      "/bms/status", rclcpp::QoS(10), std::bind(&MonitorNode::on_bms, this, _1));
    mppt_sub_ = this->create_subscription<airship_msgs::msg::MpptStatus>(
      "/mppt/status", rclcpp::QoS(10), std::bind(&MonitorNode::on_mppt, this, _1));
    dcdc_sub_ = this->create_subscription<airship_msgs::msg::DcdcStatus>(
      "/dcdc/status", rclcpp::QoS(10), std::bind(&MonitorNode::on_dcdc, this, _1));

    // ===== 看门狗定时器 =====
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / watchdog_hz_)),
      std::bind(&MonitorNode::watchdog_callback, this));
  }

private:
  // BMS 状态回调
  void on_bms(const airship_msgs::msg::BmsStatus::SharedPtr msg)
  {
    last_bms_time_ = this->now();
    bms_has_data_ = true;
    // BMS 告警级别检查
    if (msg->alarm_level != last_bms_alarm_) {
      publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_BMS,
        msg->alarm_level >= 2 ? airship_msgs::msg::DeviceAlert::SEVERITY_CRITICAL :
        airship_msgs::msg::DeviceAlert::SEVERITY_WARNING,
        msg->fault_word1,
        "BMS 告警级别变化: " + std::to_string(msg->alarm_level),
        msg->alarm_level != 0);
      last_bms_alarm_ = msg->alarm_level;
    }
  }

  // MPPT 状态回调
  void on_mppt(const airship_msgs::msg::MpptStatus::SharedPtr msg)
  {
    last_mppt_time_ = this->now();
    mppt_has_data_ = true;
    // MPPT 故障状态检查
    if (msg->fault_state != last_mppt_fault_) {
      publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_MPPT,
        msg->fault_state ? airship_msgs::msg::DeviceAlert::SEVERITY_WARNING :
        airship_msgs::msg::DeviceAlert::SEVERITY_INFO,
        msg->fault_state,
        "MPPT 故障状态变化: 0x" + std::to_string(msg->fault_state),
        msg->fault_state != 0);
      last_mppt_fault_ = msg->fault_state;
    }
  }

  // DCDC 状态回调
  void on_dcdc(const airship_msgs::msg::DcdcStatus::SharedPtr msg)
  {
    last_dcdc_time_ = this->now();
    dcdc_has_data_ = true;
    // DCDC 故障位检查 (Bit7 总故障)
    if (msg->fault_word & 0x80) {
      publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_DCDC,
        airship_msgs::msg::DeviceAlert::SEVERITY_CRITICAL,
        msg->fault_word,
        "DCDC 总故障异常: 0x" + std::to_string(msg->fault_word),
        true);
    }
  }

  // 看门狗: 检测设备离线
  void watchdog_callback()
  {
    const rclcpp::Time now = this->now();
    check_link(airship_msgs::msg::DeviceAlert::DEVICE_BMS, bms_has_data_, last_bms_time_);
    check_link(airship_msgs::msg::DeviceAlert::DEVICE_MPPT, mppt_has_data_, last_mppt_time_);
    check_link(airship_msgs::msg::DeviceAlert::DEVICE_DCDC, dcdc_has_data_, last_dcdc_time_);
  }

  // 链路超时检测
  void check_link(uint8_t device_type, bool has_data, const rclcpp::Time & last_time)
  {
    if (!has_data) {
      return;  // 从未收到数据, 不判离线(避免启动误报)
    }
    const double age = (this->now() - last_time).seconds();
    if (age > timeout_s_) {
      publish_alert(device_type,
        airship_msgs::msg::DeviceAlert::SEVERITY_WARNING,
        0,
        "设备离线(超时 " + std::to_string(age) + "s)",
        true);
    }
  }

  // 发布告警
  void publish_alert(
    uint8_t device_type, uint8_t severity, uint16_t code,
    const std::string & message, bool active)
  {
    auto alert = airship_msgs::msg::DeviceAlert();
    alert.header.stamp = this->now();
    alert.device_type = device_type;
    alert.severity = severity;
    alert.code = code;
    alert.message = message;
    alert.active = active;
    alert_pub_->publish(alert);

    const char * level = (severity == airship_msgs::msg::DeviceAlert::SEVERITY_CRITICAL) ?
      "CRITICAL" : (severity == airship_msgs::msg::DeviceAlert::SEVERITY_WARNING) ? "WARN" : "INFO";
    RCLCPP_WARN(this->get_logger(), "[%s][dev=%u] %s", level, device_type, message.c_str());
  }

  // ===== 成员 =====
  double watchdog_hz_;
  double timeout_s_;

  rclcpp::Publisher<airship_msgs::msg::DeviceAlert>::SharedPtr alert_pub_;
  rclcpp::Subscription<airship_msgs::msg::BmsStatus>::SharedPtr bms_sub_;
  rclcpp::Subscription<airship_msgs::msg::MpptStatus>::SharedPtr mppt_sub_;
  rclcpp::Subscription<airship_msgs::msg::DcdcStatus>::SharedPtr dcdc_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_bms_time_;
  rclcpp::Time last_mppt_time_;
  rclcpp::Time last_dcdc_time_;
  bool bms_has_data_ = false;
  bool mppt_has_data_ = false;
  bool dcdc_has_data_ = false;
  uint8_t last_bms_alarm_ = 0;
  uint16_t last_mppt_fault_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
