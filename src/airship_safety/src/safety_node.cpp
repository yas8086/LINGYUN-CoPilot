// 灵云01号伴飞电脑 — 安全仲裁节点 (safety_node)
//
// 功能: 订阅各设备状态, 聚合安全判据, 发布 safe_to_control 总开关到 /safety/status。
// 判据(fail-safe): 任一关键设备异常 => safe_to_control=false。
//
// 当前判据:
//   - DCDC 自身无故障 (fault_word 无故障位; 排除 BIT2 输出状态位)
//     注: 48V 为飞艇弱电主源, DCDC 控制由独立 dcdc_hold 保活维持, 本节点不直接断电。
//   - BMS 在线 且 总压不低于下限 且 无告警级
//   - 12S 备用电源 在线 且 总压不低于下限 且 无故障位
//
// 输出话题:
//   /safety/status   (airship_msgs/SafetyStatus)
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "airship_msgs/msg/backup_bms_status.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/safety_status.hpp"
#include "airship_safety/safety_logic.hpp"

using std::placeholders::_1;

class SafetyNode : public rclcpp::Node
{
public:
  explicit SafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("safety_node", options)
  {
    pub_rate_hz_ = this->declare_parameter("pub_rate_hz", 10.0);
    dcdc_timeout_s_ = this->declare_parameter("dcdc_timeout_s", 3.0);
    bms_timeout_s_ = this->declare_parameter("bms_timeout_s", 3.0);
    bms_min_voltage_ = this->declare_parameter("bms_min_voltage", 40.0);  // 总压下限 [V]
    backup_bms_timeout_s_ = this->declare_parameter("backup_bms_timeout_s", 3.0);
    backup_bms_min_voltage_ = this->declare_parameter("backup_bms_min_voltage", 24.0);  // [V]

    // 除零防护: 频率必须为正
    if (pub_rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "pub_rate_hz 非法值 %.3f, 重置为 10.0", pub_rate_hz_);
      pub_rate_hz_ = 10.0;
    }

    status_pub_ = this->create_publisher<airship_msgs::msg::SafetyStatus>(
      "/safety/status", rclcpp::QoS(10));

    dcdc_sub_ = this->create_subscription<airship_msgs::msg::DcdcStatus>(
      "/dcdc/status", rclcpp::QoS(10), std::bind(&SafetyNode::on_dcdc, this, _1));
    bms_sub_ = this->create_subscription<airship_msgs::msg::BmsStatus>(
      "/bms/status", rclcpp::QoS(10), std::bind(&SafetyNode::on_bms, this, _1));
    backup_sub_ = this->create_subscription<airship_msgs::msg::BackupBmsStatus>(
      "/backup_bms/status", rclcpp::QoS(10), std::bind(&SafetyNode::on_backup, this, _1));

    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / pub_rate_hz_)),
      std::bind(&SafetyNode::publish_status, this));

    // 统一时间源
    last_dcdc_time_ = this->now();
    last_bms_time_ = this->now();
    last_backup_time_ = this->now();
  }

private:
  void on_dcdc(const airship_msgs::msg::DcdcStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dcdc_online_ = msg->online;
    dcdc_fault_word_ = msg->fault_word;
    // 仅在线消息刷新链路时间戳(issue#3): 设备离线时 dcdc_node 仍周期发布 online=false
    // 兜底消息, 若据此刷新 last_dcdc_time_, 超时判据 (now-last)<timeout 将永久失效。
    // 离线由超时机制兜底, 保证 fail-safe(与 monitor_node 处理一致)。
    if (msg->online) {
      dcdc_has_data_ = true;
      last_dcdc_time_ = this->now();
    }
  }

  void on_bms(const airship_msgs::msg::BmsStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bms_online_ = msg->online;
    bms_pack_voltage_ = msg->pack_voltage;
    bms_alarm_level_ = msg->alarm_level;
    // 仅在线消息刷新链路时间戳(issue#3), 理由见 on_dcdc
    if (msg->online) {
      bms_has_data_ = true;
      last_bms_time_ = this->now();
    }
  }

  void on_backup(const airship_msgs::msg::BackupBmsStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    backup_online_ = msg->online;
    backup_pack_voltage_ = msg->pack_voltage;
    backup_fault_word_ = msg->fault_word;
    // 仅在线消息刷新链路时间戳(issue#3), 理由见 on_dcdc
    if (msg->online) {
      backup_has_data_ = true;
      last_backup_time_ = this->now();
    }
  }

  void publish_status()
  {
    auto msg = airship_msgs::msg::SafetyStatus();
    msg.header.stamp = this->now();

    bool dcdc_judged = false;
    bool battery_judged = false;
    bool backup_battery_judged = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // DCDC 判据: 未收到数据或超时 => 视为不安全 (fail-safe)
      const bool dcdc_link_online =
        dcdc_has_data_ && (this->now() - last_dcdc_time_).seconds() < dcdc_timeout_s_;
      const bool dcdc_online = dcdc_link_online && dcdc_online_;
      dcdc_judged = airship_safety::dcdc_judge(dcdc_online, dcdc_fault_word_);

      // 主 BMS 判据: 未收到数据或超时 => 视为不安全 (fail-safe)
      const bool bms_link_online =
        bms_has_data_ && (this->now() - last_bms_time_).seconds() < bms_timeout_s_;
      const bool bms_online = bms_link_online && bms_online_;
      battery_judged = airship_safety::battery_judge(
        bms_online, bms_pack_voltage_, bms_alarm_level_, static_cast<float>(bms_min_voltage_));

      // 12S 备用电源判据: 未收到数据或超时 => 视为不安全 (fail-safe)
      const bool backup_link_online =
        backup_has_data_ && (this->now() - last_backup_time_).seconds() < backup_bms_timeout_s_;
      const bool backup_online = backup_link_online && backup_online_;
      backup_battery_judged = airship_safety::backup_battery_judge(
        backup_online, backup_pack_voltage_, backup_fault_word_,
        static_cast<float>(backup_bms_min_voltage_));
    }

    const auto decision = airship_safety::aggregate(dcdc_judged, battery_judged,
      backup_battery_judged);

    msg.dcdc_ok = decision.dcdc_ok;
    msg.battery_ok = battery_judged;
    msg.backup_battery_ok = backup_battery_judged;
    msg.flight_armed = false;
    msg.safe_to_control = decision.safe_to_control;
    msg.reason = decision.reason;

    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  double pub_rate_hz_;
  double dcdc_timeout_s_;
  double bms_timeout_s_;
  double bms_min_voltage_;
  double backup_bms_timeout_s_;
  double backup_bms_min_voltage_;

  std::mutex mutex_;
  bool dcdc_online_ = false;
  uint8_t dcdc_fault_word_ = 0;
  bool dcdc_has_data_ = false;
  rclcpp::Time last_dcdc_time_;
  bool bms_online_ = false;
  float bms_pack_voltage_ = 0.0f;
  uint8_t bms_alarm_level_ = 0;
  bool bms_has_data_ = false;
  rclcpp::Time last_bms_time_;
  bool backup_online_ = false;
  float backup_pack_voltage_ = 0.0f;
  uint32_t backup_fault_word_ = 0;
  bool backup_has_data_ = false;
  rclcpp::Time last_backup_time_;

  rclcpp::Publisher<airship_msgs::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<airship_msgs::msg::DcdcStatus>::SharedPtr dcdc_sub_;
  rclcpp::Subscription<airship_msgs::msg::BmsStatus>::SharedPtr bms_sub_;
  rclcpp::Subscription<airship_msgs::msg::BackupBmsStatus>::SharedPtr backup_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
