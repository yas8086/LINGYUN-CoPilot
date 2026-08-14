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

#include "airship_msgs/msg/backup_bms_status.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/device_alert.hpp"
#include "airship_msgs/msg/lo_ra_summary.hpp"
#include "airship_msgs/msg/mppt_status.hpp"
#include "airship_monitor/alert_dedup.hpp"

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
    // 除零防护: 频率必须为正, 否则 1000.0/rate 产生 inf 导致 static_cast<int> 未定义行为
    if (watchdog_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "watchdog_rate_hz 非法值 %.3f, 重置为 1.0", watchdog_hz_);
      watchdog_hz_ = 1.0;
    }

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
    lora_sub_ = this->create_subscription<airship_msgs::msg::LoRaSummary>(
      "/lora/summary", rclcpp::QoS(10), std::bind(&MonitorNode::on_lora, this, _1));
    backup_sub_ = this->create_subscription<airship_msgs::msg::BackupBmsStatus>(
      "/backup_bms/status", rclcpp::QoS(10), std::bind(&MonitorNode::on_backup, this, _1));

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
        // DeviceAlert.code 为 16 位, 仅携带故障字低 16 位(系统故障位); 完整 32 位经 BmsStatus/JSON 下传
        static_cast<uint16_t>(msg->fault_word1),
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
    // DCDC 故障判定 (排除 Bit2 输出状态位, 与 safety 判据一致); 仅在跳变时告警/解除
    const bool fault = (msg->fault_word & kDcdcFaultMask) != 0;
    if (fault != last_dcdc_fault_active_) {
      publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_DCDC,
        fault ? airship_msgs::msg::DeviceAlert::SEVERITY_CRITICAL :
        airship_msgs::msg::DeviceAlert::SEVERITY_INFO,
        msg->fault_word,
        fault ? ("DCDC 故障异常: 0x" + std::to_string(msg->fault_word)) :
        "DCDC 故障已解除",
        fault);
      last_dcdc_fault_active_ = fault;
    }
  }

  // LoRa 汇总回调: 超温/离线/串口掉线告警 (仅状态变化时触发)
  void on_lora(const airship_msgs::msg::LoRaSummary::SharedPtr msg)
  {
    last_lora_time_ = this->now();
    lora_has_data_ = true;

    // 串口掉线告警 (状态变化触发)
    if (msg->serial_online != last_lora_serial_online_) {
      publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_LORA,
        msg->serial_online ? airship_msgs::msg::DeviceAlert::SEVERITY_INFO :
        airship_msgs::msg::DeviceAlert::SEVERITY_WARNING,
        0,
        msg->serial_online ? "LoRa 485 串口已恢复" : "LoRa 485 串口掉线",
        !msg->serial_online);
      last_lora_serial_online_ = msg->serial_online;
    }

    if (msg->alarm_count != last_lora_alarm_) {
      if (msg->alarm_count > 0) {
        std::string ids;
        for (size_t i = 0; i < msg->alarm_node_ids.size(); ++i) {
          if (i > 0) {ids += ",";}
          ids += std::to_string(msg->alarm_node_ids[i]);
        }
        publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_LORA,
          airship_msgs::msg::DeviceAlert::SEVERITY_WARNING,
          static_cast<uint16_t>(msg->alarm_count),
          "LoRa 温度告警节点: " + ids,
          true);
      } else {
        publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_LORA,
          airship_msgs::msg::DeviceAlert::SEVERITY_INFO,
          0,
          "LoRa 温度告警解除",
          false);
      }
      last_lora_alarm_ = msg->alarm_count;
    }

    // 部分节点离线告警 (状态变化触发, 避免每次 summary 刷屏)
    const bool has_node_offline = msg->node_count > 0 && msg->online_count < msg->node_count;
    if (has_node_offline != last_lora_node_offline_) {
      last_lora_node_offline_ = has_node_offline;
      if (has_node_offline) {
        publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_LORA,
          airship_msgs::msg::DeviceAlert::SEVERITY_WARNING,
          static_cast<uint16_t>(msg->node_count - msg->online_count),
          "LoRa 节点离线: " + std::to_string(msg->online_count) + "/" +
            std::to_string(msg->node_count),
          true);
      } else {
        publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_LORA,
          airship_msgs::msg::DeviceAlert::SEVERITY_INFO,
          0,
          "LoRa 节点全部恢复在线",
          false);
      }
    }
  }

  // 12S 备用电源 BMS 状态回调
  void on_backup(const airship_msgs::msg::BackupBmsStatus::SharedPtr msg)
  {
    last_backup_time_ = this->now();
    backup_has_data_ = true;
    // 备用电源告警/保护/故障任一触发即视为异常 (状态跳变时告警/解除, 避免刷屏)
    const bool abnormal = (msg->alarm_word != 0) || (msg->protect_word != 0) ||
      (msg->fault_word != 0);
    if (abnormal != last_backup_abnormal_) {
      publish_alert(airship_msgs::msg::DeviceAlert::DEVICE_BACKUP_BMS,
        abnormal ? airship_msgs::msg::DeviceAlert::SEVERITY_WARNING :
        airship_msgs::msg::DeviceAlert::SEVERITY_INFO,
        static_cast<uint16_t>(msg->fault_word & 0xFFFF),
        abnormal ? ("备用电源异常: alarm=0x" + std::to_string(msg->alarm_word) +
        " protect=0x" + std::to_string(msg->protect_word) +
        " fault=0x" + std::to_string(msg->fault_word)) :
        "备用电源异常已解除",
        abnormal);
      last_backup_abnormal_ = abnormal;
    }
  }

  // 看门狗: 检测设备离线 (仅在状态变化时发告警, 避免刷屏)
  void watchdog_callback()
  {
    check_link(airship_msgs::msg::DeviceAlert::DEVICE_BMS, bms_has_data_, last_bms_time_,
      &bms_online_);
    check_link(airship_msgs::msg::DeviceAlert::DEVICE_MPPT, mppt_has_data_, last_mppt_time_,
      &mppt_online_);
    check_link(airship_msgs::msg::DeviceAlert::DEVICE_DCDC, dcdc_has_data_, last_dcdc_time_,
      &dcdc_online_);
    check_link(airship_msgs::msg::DeviceAlert::DEVICE_LORA, lora_has_data_, last_lora_time_,
      &lora_online_);
    check_link(airship_msgs::msg::DeviceAlert::DEVICE_BACKUP_BMS, backup_has_data_,
      last_backup_time_, &backup_online_);
  }

  // 链路状态检测: 状态变化时发告警 (离线发告警, 恢复发解除)
  void check_link(
    uint8_t device_type, bool has_data, const rclcpp::Time & last_time, bool * online_flag)
  {
    if (!has_data) {
      return;  // 从未收到数据, 不判离线(避免启动误报)
    }
    const double age = (this->now() - last_time).seconds();
    const bool now_online = age <= timeout_s_;

    // 去重状态机: 仅在跳变时发告警
    const auto t = airship_monitor::update_online(now_online, online_flag);
    if (!t.changed) {
      return;
    }
    if (t.now_online) {
      publish_alert(device_type, airship_msgs::msg::DeviceAlert::SEVERITY_INFO, 0,
        "设备已上线", false);
    } else {
      publish_alert(device_type, airship_msgs::msg::DeviceAlert::SEVERITY_WARNING, 0,
        "设备离线(超时 " + std::to_string(age) + "s)", true);
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
  rclcpp::Subscription<airship_msgs::msg::LoRaSummary>::SharedPtr lora_sub_;
  rclcpp::Subscription<airship_msgs::msg::BackupBmsStatus>::SharedPtr backup_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_bms_time_;
  rclcpp::Time last_mppt_time_;
  rclcpp::Time last_dcdc_time_;
  rclcpp::Time last_lora_time_;
  rclcpp::Time last_backup_time_;
  bool bms_has_data_ = false;
  bool mppt_has_data_ = false;
  bool dcdc_has_data_ = false;
  bool lora_has_data_ = false;
  bool backup_has_data_ = false;
  // 设备在线状态(用于离线告警去重, 仅在变化时发告警)
  bool bms_online_ = false;
  bool mppt_online_ = false;
  bool dcdc_online_ = false;
  bool lora_online_ = false;
  bool backup_online_ = false;
  uint8_t last_bms_alarm_ = 0;
  uint16_t last_mppt_fault_ = 0;
  int32_t last_lora_alarm_ = 0;
  bool last_lora_serial_online_ = false;
  bool last_lora_node_offline_ = false;
  // 备用电源异常告警去重
  bool last_backup_abnormal_ = false;
  // DCDC 故障告警去重: 与 safety 一致, 排除 Bit2 输出状态位
  static constexpr uint8_t kDcdcFaultMask = 0xFB;  // ~0x04
  bool last_dcdc_fault_active_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
