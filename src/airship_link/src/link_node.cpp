// 灵云01号伴飞电脑 — 数传链路节点 (串口 + UDP 双下传)
// 功能:
//   1. 订阅 BMS/MPPT/DCDC 状态, 缓存最新值
//   2. 定时聚合打包为单行 JSON, 加帧头帧尾
//   3. 同时经串口(地面 Qt 上位机) 与 UDP(数传网口, 如 L33) 下传
//
// 帧格式: 0xAA 0x55 [JSON]\n
// UDP 通道: udp_host/udp_port 配置目标地址; udp_host 为空时仅串口下传
#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "airship_link/json_packer.hpp"
#include "airship_link/serial_interface.hpp"
#include "airship_link/udp_sender.hpp"
#include "airship_msgs/msg/backup_bms_status.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/flight_status.hpp"
#include "airship_msgs/msg/lo_ra_samples.hpp"
#include "airship_msgs/msg/mppt_status.hpp"

using std::placeholders::_1;

class LinkNode : public rclcpp::Node
{
public:
  LinkNode()
  : rclcpp::Node("link_node"), serial_("/dev/ttyUSB0"), udp_("", 0)
  {
    // ===== 参数 =====
    serial_dev_ = this->declare_parameter("serial_device", std::string("/dev/ttyUSB0"));
    baud_rate_ = this->declare_parameter("baud_rate", 115200);
    tx_rate_hz_ = this->declare_parameter("tx_rate_hz", 5.0);  // 200ms
    // 除零防护: 频率必须为正, 否则 1000.0/rate 产生 inf 导致 static_cast<int> 未定义行为
    if (tx_rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "tx_rate_hz 非法值 %.3f, 重置为 5.0", tx_rate_hz_);
      tx_rate_hz_ = 5.0;
    }
    // 串口自动重连周期 (ms): 检测到串口掉线后按此周期尝试重连
    reconnect_ms_ = this->declare_parameter("reconnect_ms", 2000);

    // UDP 数传网口下传参数 (如 L33: host=192.168.10.254); host 为空则仅串口下传
    udp_host_ = this->declare_parameter("udp_host", std::string(""));
    udp_port_ = static_cast<uint16_t>(this->declare_parameter("udp_port", 14551));
    udp_enabled_ = !udp_host_.empty();

    // ===== 订阅 =====
    bms_sub_ = this->create_subscription<airship_msgs::msg::BmsStatus>(
      "/bms/status", rclcpp::QoS(10), std::bind(&LinkNode::on_bms, this, _1));
    mppt_sub_ = this->create_subscription<airship_msgs::msg::MpptStatus>(
      "/mppt/status", rclcpp::QoS(10), std::bind(&LinkNode::on_mppt, this, _1));
    mppt2_sub_ = this->create_subscription<airship_msgs::msg::MpptStatus>(
      "/mppt2/status", rclcpp::QoS(10), std::bind(&LinkNode::on_mppt2, this, _1));
    dcdc_sub_ = this->create_subscription<airship_msgs::msg::DcdcStatus>(
      "/dcdc/status", rclcpp::QoS(10), std::bind(&LinkNode::on_dcdc, this, _1));
    fc_sub_ = this->create_subscription<airship_msgs::msg::FlightStatus>(
      "/fc/status", rclcpp::QoS(10), std::bind(&LinkNode::on_fc, this, _1));
    lora_sub_ = this->create_subscription<airship_msgs::msg::LoRaSamples>(
      "/lora/samples", rclcpp::QoS(10), std::bind(&LinkNode::on_lora, this, _1));
    backup_sub_ = this->create_subscription<airship_msgs::msg::BackupBmsStatus>(
      "/backup_bms/status", rclcpp::QoS(10), std::bind(&LinkNode::on_backup, this, _1));

    // ===== 打开发送串口 =====
    serial_ = airship_link::SerialInterface(serial_dev_);
    if (!serial_.open(static_cast<airship_link::BaudRate>(baud_rate_))) {
      RCLCPP_WARN(this->get_logger(), "打开串口 %s 失败, 数传链路不可用", serial_dev_.c_str());
    }

    // ===== 初始化 UDP 数传网口下传 =====
    if (udp_enabled_) {
      udp_ = airship_link::UdpSender(udp_host_, udp_port_);
      if (udp_.open()) {
        RCLCPP_INFO(this->get_logger(), "UDP 下传已启用: %s:%u", udp_host_.c_str(), udp_port_);
      } else {
        udp_enabled_ = false;
        RCLCPP_WARN(this->get_logger(), "UDP 下传初始化失败(地址 %s:%u), 仅串口下传",
          udp_host_.c_str(), udp_port_);
      }
    }

    // ===== 发送定时器 =====
    tx_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / tx_rate_hz_)),
      std::bind(&LinkNode::tx_callback, this));
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

  // 定时打包并发送 (串口与 UDP 独立通道, 一路故障不影响另一路)
  void tx_callback()
  {
    // 串口掉线时按 reconnect_ms_ 周期尝试重连 (数传电台/USB 插拔后自动恢复)
    if (!serial_.is_open()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_reconnect_at_ < std::chrono::milliseconds(reconnect_ms_)) {
        if (!udp_enabled_ || !udp_.is_open()) {
          return;  // 串口不可用且无 UDP 通道则跳过本周期(无需打包)
        }
      } else {
        last_reconnect_at_ = now;
        const bool ok = serial_.open(static_cast<airship_link::BaudRate>(baud_rate_));
        if (ok) {
          RCLCPP_INFO(
            this->get_logger(), "数传串口 %s 重连成功", serial_dev_.c_str());
        } else {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "数传串口 %s 不可用, 重连中", serial_dev_.c_str());
          if (!udp_enabled_ || !udp_.is_open()) {
            return;  // 无可用通道
          }
        }
      }
    }

    // 无任何可用下传通道时提前返回, 避免无谓打包
    if (!serial_.is_open() && (!udp_enabled_ || !udp_.is_open())) {
      return;
    }

    std::string json;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      json = airship_link::pack_telemetry_json(
        this->now().seconds(), &bms_, &mppt_, &mppt2_, &dcdc_, &fc_, &lora_, &backup_);
    }

    // 帧头 + JSON + 换行
    std::string frame;
    frame.push_back(static_cast<char>(0xAA));
    frame.push_back(static_cast<char>(0x55));
    frame += json;
    frame.push_back('\n');

    // ===== 串口通道 (地面 Qt 上位机) =====
    if (serial_.is_open() && !serial_.write(frame)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "数传串口 %s 写入失败, 本帧已丢弃", serial_dev_.c_str());
      // 写入失败通常意味着串口物理掉线(USB 拔插/数传电台断连):
      // 主动关闭使 is_open()==false, 从而在下一周期落入上方重连分支自动恢复。
      // 否则 fd 描述符仍 >=0, is_open() 恒为 true, 链路将永远无法自愈。
      serial_.close();
    }

    // ===== UDP 网口通道 (如 L33 数传) =====
    if (udp_enabled_ && udp_.is_open() && !udp_.send(frame)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "UDP 下传失败(目标 %s:%u)", udp_host_.c_str(), udp_port_);
    }
  }

  // ===== 成员 =====
  std::string serial_dev_;
  int baud_rate_;
  double tx_rate_hz_;
  int reconnect_ms_;
  std::chrono::steady_clock::time_point last_reconnect_at_{};

  // UDP 数传网口下传
  std::string udp_host_;
  uint16_t udp_port_;
  bool udp_enabled_ = false;

  airship_link::SerialInterface serial_;
  airship_link::UdpSender udp_;
  std::mutex mutex_;
  airship_msgs::msg::BmsStatus bms_;
  airship_msgs::msg::MpptStatus mppt_;
  airship_msgs::msg::MpptStatus mppt2_;
  airship_msgs::msg::DcdcStatus dcdc_;
  airship_msgs::msg::FlightStatus fc_;
  airship_msgs::msg::LoRaSamples lora_;
  airship_msgs::msg::BackupBmsStatus backup_;

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
  rclcpp::spin(std::make_shared<LinkNode>());
  rclcpp::shutdown();
  return 0;
}
