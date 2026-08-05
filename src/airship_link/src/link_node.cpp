// 灵云01号伴飞电脑 — 串口数传链路节点
// 功能:
//   1. 订阅 BMS/MPPT/DCDC 状态, 缓存最新值
//   2. 定时聚合打包为单行 JSON, 加帧头帧尾
//   3. 经串口下传地面 Qt 上位机
//
// 帧格式: 0xAA 0x55 [JSON]\n
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "airship_link/json_packer.hpp"
#include "airship_link/serial_interface.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_msgs/msg/dcdc_status.hpp"
#include "airship_msgs/msg/mppt_status.hpp"

using std::placeholders::_1;

class LinkNode : public rclcpp::Node
{
public:
  LinkNode()
  : rclcpp::Node("link_node"), serial_("/dev/ttyUSB0")
  {
    // ===== 参数 =====
    serial_dev_ = this->declare_parameter("serial_device", std::string("/dev/ttyUSB0"));
    baud_rate_ = this->declare_parameter("baud_rate", 115200);
    tx_rate_hz_ = this->declare_parameter("tx_rate_hz", 5.0);  // 200ms

    // ===== 订阅 =====
    bms_sub_ = this->create_subscription<airship_msgs::msg::BmsStatus>(
      "/bms/status", rclcpp::QoS(10), std::bind(&LinkNode::on_bms, this, _1));
    mppt_sub_ = this->create_subscription<airship_msgs::msg::MpptStatus>(
      "/mppt/status", rclcpp::QoS(10), std::bind(&LinkNode::on_mppt, this, _1));
    dcdc_sub_ = this->create_subscription<airship_msgs::msg::DcdcStatus>(
      "/dcdc/status", rclcpp::QoS(10), std::bind(&LinkNode::on_dcdc, this, _1));

    // ===== 打开发送串口 =====
    serial_ = airship_link::SerialInterface(serial_dev_);
    if (!serial_.open(static_cast<airship_link::BaudRate>(baud_rate_))) {
      RCLCPP_WARN(this->get_logger(), "打开串口 %s 失败, 数传链路不可用", serial_dev_.c_str());
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

  void on_dcdc(const airship_msgs::msg::DcdcStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dcdc_ = *msg;
  }

  // 定时打包并发送
  void tx_callback()
  {
    std::string json;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      json = airship_link::pack_telemetry_json(this->now().seconds(), &bms_, &mppt_, &dcdc_);
    }

    if (!serial_.is_open()) {
      return;
    }

    // 帧头 + JSON + 换行
    std::string frame;
    frame.push_back(static_cast<char>(0xAA));
    frame.push_back(static_cast<char>(0x55));
    frame += json;
    frame.push_back('\n');
    serial_.write(frame);
  }

  // ===== 成员 =====
  std::string serial_dev_;
  int baud_rate_;
  double tx_rate_hz_;

  airship_link::SerialInterface serial_;
  std::mutex mutex_;
  airship_msgs::msg::BmsStatus bms_;
  airship_msgs::msg::MpptStatus mppt_;
  airship_msgs::msg::DcdcStatus dcdc_;

  rclcpp::Subscription<airship_msgs::msg::BmsStatus>::SharedPtr bms_sub_;
  rclcpp::Subscription<airship_msgs::msg::MpptStatus>::SharedPtr mppt_sub_;
  rclcpp::Subscription<airship_msgs::msg::DcdcStatus>::SharedPtr dcdc_sub_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinkNode>());
  rclcpp::shutdown();
  return 0;
}
