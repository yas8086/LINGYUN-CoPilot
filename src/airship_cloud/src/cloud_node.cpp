// 灵云01号伴飞电脑 — 4G 数据汇总上云节点 (cloud_node)
//
// 订阅 BMS/MPPT/DCDC/飞控状态, 定时打包为 JSON, 经 MQTT 上云。
// 打包逻辑复用 airship_link::json_packer; 传输用 MQTT(libmosquitto)。
//
// 参数:
//   mqtt_host / mqtt_port / mqtt_topic / mqtt_username / mqtt_password
//   tx_rate_hz  (打包发送频率)
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "airship_cloud/mqtt_client.hpp"
#include "airship_link/json_packer.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/flight_status.hpp"
#include "airship_msgs/msg/mppt_status.hpp"

using std::placeholders::_1;

class CloudNode : public rclcpp::Node
{
public:
  explicit CloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("cloud_node", options)
  {
    host_ = this->declare_parameter("mqtt_host", std::string("localhost"));
    port_ = this->declare_parameter("mqtt_port", 1883);
    topic_ = this->declare_parameter("mqtt_topic", std::string("lingyun01/telemetry"));
    username_ = this->declare_parameter("mqtt_username", std::string(""));
    password_ = this->declare_parameter("mqtt_password", std::string(""));
    tx_rate_hz_ = this->declare_parameter("tx_rate_hz", 2.0);

    // 订阅各设备状态
    bms_sub_ = this->create_subscription<airship_msgs::msg::BmsStatus>(
      "/bms/status", rclcpp::QoS(10), std::bind(&CloudNode::on_bms, this, _1));
    mppt_sub_ = this->create_subscription<airship_msgs::msg::MpptStatus>(
      "/mppt/status", rclcpp::QoS(10), std::bind(&CloudNode::on_mppt, this, _1));
    dcdc_sub_ = this->create_subscription<airship_msgs::msg::DcdcStatus>(
      "/dcdc/status", rclcpp::QoS(10), std::bind(&CloudNode::on_dcdc, this, _1));
    fc_sub_ = this->create_subscription<airship_msgs::msg::FlightStatus>(
      "/fc/status", rclcpp::QoS(10), std::bind(&CloudNode::on_fc, this, _1));

    // 初始化 MQTT
    mqtt_ = std::make_unique<airship_cloud::MqttClient>(
      host_, port_, "lingyun01_onboard", username_, password_);
    if (mqtt_->connect()) {
      RCLCPP_INFO(this->get_logger(), "MQTT 连接已启动: %s:%d, topic=%s",
        host_.c_str(), port_, topic_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "MQTT 连接启动失败");
    }

    // 发送定时器
    tx_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / tx_rate_hz_)),
      std::bind(&CloudNode::tx_callback, this));
  }

private:
  void on_bms(const airship_msgs::msg::BmsStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bms_ = *msg;
  }

  void on_mppt(const airship_msgs::msg::MpptStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mppt_ = *msg;
  }

  void on_dcdc(const airship_msgs::msg::DcdcStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dcdc_ = *msg;
  }

  void on_fc(const airship_msgs::msg::FlightStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fc_ = *msg;
  }

  void tx_callback()
  {
    std::string json;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      json = airship_link::pack_telemetry_json(
        this->now().seconds(), &bms_, &mppt_, &dcdc_, &fc_);
    }

    if (!mqtt_->is_connected()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "MQTT 未连接, 丢弃数据");
      return;
    }
    if (!mqtt_->publish(topic_, json)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "MQTT 发布失败");
    }
  }

  // ===== 成员 =====
  std::string host_;
  int port_;
  std::string topic_;
  std::string username_;
  std::string password_;
  double tx_rate_hz_;

  std::mutex mutex_;
  airship_msgs::msg::BmsStatus bms_;
  airship_msgs::msg::MpptStatus mppt_;
  airship_msgs::msg::DcdcStatus dcdc_;
  airship_msgs::msg::FlightStatus fc_;

  std::unique_ptr<airship_cloud::MqttClient> mqtt_;

  rclcpp::Subscription<airship_msgs::msg::BmsStatus>::SharedPtr bms_sub_;
  rclcpp::Subscription<airship_msgs::msg::MpptStatus>::SharedPtr mppt_sub_;
  rclcpp::Subscription<airship_msgs::msg::DcdcStatus>::SharedPtr dcdc_sub_;
  rclcpp::Subscription<airship_msgs::msg::FlightStatus>::SharedPtr fc_sub_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudNode>());
  rclcpp::shutdown();
  return 0;
}
