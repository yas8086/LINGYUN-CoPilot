// 灵云01号伴飞电脑 — 4G 数据汇总上云节点 (cloud_node)
//
// 订阅 BMS/MPPT/DCDC/飞控状态, 定时打包为 JSON, 经 MQTT 上云。
// 打包逻辑复用 airship_link::json_packer; 传输用 MQTT(libmosquitto)。
//
// 参数:
//   mqtt_host / mqtt_port / mqtt_topic / mqtt_username / mqtt_password
//   mqtt_tls_enable / mqtt_tls_ca_cert / mqtt_tls_insecure
//   tx_rate_hz  (打包发送频率)
// 说明: 当 mqtt_password 参数为空时, 回退读取环境变量 MQTT_PASSWORD,
//       避免密码经命令行参数暴露在 /proc/<pid>/cmdline。
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "airship_cloud/mqtt_client.hpp"
#include "airship_link/json_packer.hpp"
#include "airship_msgs/msg/backup_bms_status.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/flight_status.hpp"
#include "airship_msgs/msg/lo_ra_samples.hpp"
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
    // 密码参数为空时回退读取环境变量, 避免密码出现在进程 cmdline
    if (password_.empty()) {
      const char * env_pwd = std::getenv("MQTT_PASSWORD");
      if (env_pwd != nullptr) {
        password_ = env_pwd;
      }
    }
    tls_enable_ = this->declare_parameter("mqtt_tls_enable", false);
    tls_ca_cert_ = this->declare_parameter("mqtt_tls_ca_cert", std::string(""));
    tls_insecure_ = this->declare_parameter("mqtt_tls_insecure", false);
    tx_rate_hz_ = this->declare_parameter("tx_rate_hz", 2.0);
    // 除零防护: 频率必须为正, 否则 1000.0/rate 产生 inf 导致 static_cast<int> 未定义行为
    if (tx_rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "tx_rate_hz 非法值 %.3f, 重置为 2.0", tx_rate_hz_);
      tx_rate_hz_ = 2.0;
    }
    // 端口与 TLS 组合自检: EMQX Cloud Serverless 必须 8883+TLS, 误配只会表现为连不上
    // (且明文 8883 约等于明文传输), 启动期即告警便于尽早发现。
    if (tls_enable_ && port_ != 8883) {
      RCLCPP_WARN(this->get_logger(),
        "TLS 已启用但端口=%d(Serverless 应使用 8883), 可能无法连接", port_);
    }
    if (!tls_enable_ && port_ == 8883) {
      RCLCPP_WARN(this->get_logger(),
        "端口 8883 通常搭配 TLS, 但当前 mqtt_tls_enable=false(明文传输, 数据泄露风险)");
    }

    // 订阅各设备状态
    // 纯"取最新帧"上云中继: SensorDataQoS()(BEST_EFFORT + KeepLast(5)),
    // 避免 reliable 在慢消费者堆积、浪费内存与背压(生产端保持可靠, best_effort 可兼容)。
    bms_sub_ = this->create_subscription<airship_msgs::msg::BmsStatus>(
      "/bms/status", rclcpp::SensorDataQoS(), std::bind(&CloudNode::on_bms, this, _1));
    mppt_sub_ = this->create_subscription<airship_msgs::msg::MpptStatus>(
      "/mppt/status", rclcpp::SensorDataQoS(), std::bind(&CloudNode::on_mppt, this, _1));
    mppt2_sub_ = this->create_subscription<airship_msgs::msg::MpptStatus>(
      "/mppt2/status", rclcpp::SensorDataQoS(), std::bind(&CloudNode::on_mppt2, this, _1));
    dcdc_sub_ = this->create_subscription<airship_msgs::msg::DcdcStatus>(
      "/dcdc/status", rclcpp::SensorDataQoS(), std::bind(&CloudNode::on_dcdc, this, _1));
    fc_sub_ = this->create_subscription<airship_msgs::msg::FlightStatus>(
      "/fc/status", rclcpp::SensorDataQoS(), std::bind(&CloudNode::on_fc, this, _1));
    lora_sub_ = this->create_subscription<airship_msgs::msg::LoRaSamples>(
      "/lora/samples", rclcpp::SensorDataQoS(), std::bind(&CloudNode::on_lora, this, _1));
    backup_sub_ = this->create_subscription<airship_msgs::msg::BackupBmsStatus>(
      "/backup_bms/status", rclcpp::SensorDataQoS(), std::bind(&CloudNode::on_backup, this, _1));

    // 初始化 MQTT
    mqtt_ = std::make_unique<airship_cloud::MqttClient>(
      host_, port_, "lingyun01_onboard", username_, password_,
      tls_enable_, tls_ca_cert_, tls_insecure_);
    if (mqtt_->connect()) {
      RCLCPP_INFO(this->get_logger(), "MQTT 连接已启动: %s:%d, tls=%s, topic=%s",
        host_.c_str(), port_, tls_enable_ ? "on" : "off", topic_.c_str());
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

  void on_mppt2(const airship_msgs::msg::MpptStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mppt2_ = *msg;
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

  void on_lora(const airship_msgs::msg::LoRaSamples::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lora_ = *msg;
  }

  void on_backup(const airship_msgs::msg::BackupBmsStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    backup_ = *msg;
  }

  void tx_callback()
  {
    // 先检查连接再打包(与 link_node 的"先检查通道再打包"策略一致):
    // 4G 弱网断连是常态, 断连期间若仍先做 JSON 序列化+多段内存分配纯属浪费。
    if (!mqtt_->is_connected()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "MQTT 未连接, 丢弃数据");
      return;
    }

    std::string json;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      json = airship_link::pack_telemetry_json(
        this->now().seconds(), &bms_, &mppt_, &mppt2_, &dcdc_, &fc_, &lora_, &backup_);
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
  bool tls_enable_;
  std::string tls_ca_cert_;
  bool tls_insecure_;
  double tx_rate_hz_;

  std::mutex mutex_;
  airship_msgs::msg::BmsStatus bms_;
  airship_msgs::msg::MpptStatus mppt_;
  airship_msgs::msg::MpptStatus mppt2_;
  airship_msgs::msg::DcdcStatus dcdc_;
  airship_msgs::msg::FlightStatus fc_;
  airship_msgs::msg::LoRaSamples lora_;
  airship_msgs::msg::BackupBmsStatus backup_;

  std::unique_ptr<airship_cloud::MqttClient> mqtt_;

  rclcpp::Subscription<airship_msgs::msg::BmsStatus>::SharedPtr bms_sub_;
  rclcpp::Subscription<airship_msgs::msg::MpptStatus>::SharedPtr mppt_sub_;
  rclcpp::Subscription<airship_msgs::msg::MpptStatus>::SharedPtr mppt2_sub_;
  rclcpp::Subscription<airship_msgs::msg::DcdcStatus>::SharedPtr dcdc_sub_;
  rclcpp::Subscription<airship_msgs::msg::FlightStatus>::SharedPtr fc_sub_;
  rclcpp::Subscription<airship_msgs::msg::LoRaSamples>::SharedPtr lora_sub_;
  rclcpp::Subscription<airship_msgs::msg::BackupBmsStatus>::SharedPtr backup_sub_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudNode>());
  rclcpp::shutdown();
  return 0;
}
