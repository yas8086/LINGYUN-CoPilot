// 灵云01号伴飞电脑 — MPPT 光伏控制器驱动节点
// 协议: YQPV_SPC/SMC 系列 MPPT-CAN通信协议 V1.2
//
// 功能:
//   1. 周期发送只读远程帧查询各地址段数据
//   2. 接收从机回应帧, 按地址段路由到 mppt_protocol 解析
//   3. 累积解析结果, 发布 MpptStatus
//
// 协议解析逻辑见 airship_mppt::mppt_protocol (纯库, 可 gtest)。
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "airship_can/can_interface.hpp"
#include "airship_mppt/mppt_protocol.hpp"
#include "airship_msgs/msg/mppt_status.hpp"

using airship_can::CanFrame;
using airship_can::SocketCanInterface;
using airship_mppt::MpptData;
using airship_mppt::ReadCode;

class MpptNode : public rclcpp::Node
{
public:
  explicit MpptNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("mppt_node", options),
    can_("can0"),
    running_(false)
  {
    // ===== 参数 =====
    can_if_ = this->declare_parameter("can_interface", std::string("can0"));
    query_rate_hz_ = this->declare_parameter("query_rate_hz", 1.0);
    device_addr_ = static_cast<uint8_t>(this->declare_parameter("device_addr", 1));

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::MpptStatus>("/mppt/status", rclcpp::QoS(10));

    can_ = SocketCanInterface(can_if_);
    if (!can_.open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CAN 接口 %s 失败, 稍后重试", can_if_.c_str());
    }

    running_ = true;
    query_thread_ = std::thread(&MpptNode::query_loop, this);
    receive_thread_ = std::thread(&MpptNode::receive_loop, this);
  }

  ~MpptNode() override
  {
    running_ = false;
    if (query_thread_.joinable()) {
      query_thread_.join();
    }
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    can_.close();
  }

private:
  // 查询线程: 依次轮询各只读地址段
  void query_loop()
  {
    const auto period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / query_rate_hz_));
    const ReadCode codes[] = {
      ReadCode::kCodeRated, ReadCode::kCodeRealtime, ReadCode::kCodeState,
      ReadCode::kCodeEnergyDay, ReadCode::kCodeEnergyTotal,
      ReadCode::kCodeTemp, ReadCode::kCodeControl,
    };
    while (running_.load()) {
      if (can_.is_open()) {
        for (ReadCode code : codes) {
          can_.send(airship_mppt::build_query_frame(code));
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      }
      std::this_thread::sleep_for(period);
    }
  }

  // 接收线程: 从机回应帧 ID 为 0x14[code]A1[dev]
  void receive_loop()
  {
    while (running_.load()) {
      CanFrame frame{};
      if (!can_.receive(frame, 100)) {
        continue;
      }
      // 校验只读类型与目标地址
      const uint8_t type = static_cast<uint8_t>((frame.id >> 24) & 0xFF);
      const uint8_t target = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
      if (type != airship_mppt::kReadType || target != airship_mppt::kTargetAddr) {
        continue;
      }
      const uint8_t code = static_cast<uint8_t>((frame.id >> 16) & 0xFF);
      switch (static_cast<ReadCode>(code)) {
        case ReadCode::kCodeRated:
          airship_mppt::parse_rated(frame.data, mppt_data_);
          break;
        case ReadCode::kCodeRealtime:
          airship_mppt::parse_realtime(frame.data, mppt_data_);
          break;
        case ReadCode::kCodeState:
          airship_mppt::parse_state(frame.data, mppt_data_);
          break;
        case ReadCode::kCodeEnergyDay:
          airship_mppt::parse_energy_day(frame.data, mppt_data_);
          break;
        case ReadCode::kCodeEnergyTotal:
          airship_mppt::parse_energy_total(frame.data, mppt_data_);
          break;
        case ReadCode::kCodeTemp:
          airship_mppt::parse_temp(frame.data, mppt_data_);
          break;
        case ReadCode::kCodeControl:
          airship_mppt::parse_control(frame.data, mppt_data_);
          break;
        default:
          continue;
      }
      publish_status();
    }
  }

  // 发布聚合状态
  void publish_status()
  {
    auto msg = airship_msgs::msg::MpptStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.device_addr = device_addr_;

    msg.pv_voltage = mppt_data_.pv_voltage;
    msg.battery_voltage = mppt_data_.battery_voltage;
    msg.charge_current = mppt_data_.charge_current;
    msg.pv_power = mppt_data_.pv_voltage * mppt_data_.charge_current;

    msg.rated_voltage = mppt_data_.rated_voltage;
    msg.rated_current = mppt_data_.rated_current;

    msg.charge_state = mppt_data_.charge_state;
    msg.fault_state = mppt_data_.fault_state;

    msg.energy_today = mppt_data_.energy_today;
    msg.energy_month = mppt_data_.energy_month;
    msg.energy_total = mppt_data_.energy_total;

    msg.air_temp = mppt_data_.air_temp;
    msg.module_temp = mppt_data_.module_temp;

    msg.control_mode = mppt_data_.control_mode;
    msg.charging_enabled = mppt_data_.charging_enabled;

    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  std::string can_if_;
  SocketCanInterface can_;
  std::atomic<bool> running_;
  std::thread query_thread_;
  std::thread receive_thread_;

  double query_rate_hz_;
  uint8_t device_addr_;

  MpptData mppt_data_;

  rclcpp::Publisher<airship_msgs::msg::MpptStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpptNode>());
  rclcpp::shutdown();
  return 0;
}