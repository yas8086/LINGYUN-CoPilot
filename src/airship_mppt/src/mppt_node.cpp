// 灵云01号伴飞电脑 — MPPT 光伏控制器驱动节点
// 协议: YQPV_SPC/SMC 系列 MPPT-CAN通信协议 V1.2
// 功能:
//   1. 周期发送只读远程帧查询各地址段数据
//   2. 接收从机回应帧, 按地址段路由解析
//   3. 发布 MpptStatus
//
// 只读 ID 格式: 0x14 [code:8] [0xA1] [src_addr]  (主机查询)
//               0x14 [code:8] [0xA1] [dev_addr]  (从机回应)
// 关键地址段: 0x1402 额定参数 / 0x1403 实时电压电流 / 0x1404 充电+故障状态
//            0x1405 日/月发电量 / 0x1406 总发电量 / 0x1408 温度
#include <atomic>
#include <cstring>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "airship_can/can_interface.hpp"
#include "airship_msgs/msg/mppt_status.hpp"
#include "airship_utils/can_utils.hpp"

using airship_can::CanFrame;
using airship_can::SocketCanInterface;

// MPPT 协议常量
constexpr uint8_t kTargetAddr = 0xA1;   // 目标地址(协议固定)
constexpr uint8_t kSrcAddr = 0x00;      // 本机地址

// 只读地址段 (code)
constexpr uint8_t kCodeVoltageCurrent = 0x03;  // 光伏/电池电压/充电电流
constexpr uint8_t kCodeState = 0x04;           // 充电状态/故障状态
constexpr uint8_t kCodeEnergyDay = 0x05;       // 日/月发电量
constexpr uint8_t kCodeEnergyTotal = 0x06;     // 总发电量
constexpr uint8_t kCodeTemp = 0x08;            // 温度

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
    query_rate_hz_ = this->declare_parameter("query_rate_hz", 1.0);  // 1000ms
    device_addr_ = static_cast<uint8_t>(this->declare_parameter("device_addr", 1));

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::MpptStatus>("/mppt/status", rclcpp::QoS(10));

    can_ = SocketCanInterface(can_if_);
    if (!can_.open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CAN 接口 %s 失败, 稍后重试", can_if_.c_str());
    }

    running_ = true;
    control_thread_ = std::thread(&MpptNode::query_loop, this);
    receive_thread_ = std::thread(&MpptNode::receive_loop, this);
  }

  ~MpptNode() override
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
  // 构造只读查询远程帧: 0x14 [code] [0xA1] [src]
  CanFrame build_query_frame(uint8_t code)
  {
    CanFrame frame{};
    frame.id = (0x14U << 24) | (static_cast<uint32_t>(code) << 16) |
      (static_cast<uint32_t>(kTargetAddr) << 8) | kSrcAddr;
    frame.extended = true;
    frame.len = 8;
    return frame;
  }

  // 查询线程: 依次轮询各地址段
  void query_loop()
  {
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / query_rate_hz_));
    const uint8_t codes[] = {kCodeVoltageCurrent, kCodeState, kCodeEnergyDay,
      kCodeEnergyTotal, kCodeTemp};
    while (running_.load()) {
      if (can_.is_open()) {
        for (uint8_t code : codes) {
          can_.send(build_query_frame(code));
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      }
      std::this_thread::sleep_for(period);
    }
  }

  // 接收线程: 从机回应帧 ID 为 0x14 [code] [0xA1] [dev_addr]
  void receive_loop()
  {
    while (running_.load()) {
      CanFrame frame{};
      if (!can_.receive(frame, 100)) {
        continue;
      }
      // 校验回应前缀: 0x14 [code] [0xA1]
      const uint8_t code = static_cast<uint8_t>((frame.id >> 16) & 0xFF);
      const uint8_t target = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
      if (target != kTargetAddr) {
        continue;
      }
      switch (code) {
        case kCodeVoltageCurrent:
          parse_voltage_frame(frame);
          break;
        case kCodeState:
          parse_state_frame(frame);
          break;
        case kCodeEnergyDay:
          parse_energy_frame(frame);
          break;
        case kCodeEnergyTotal:
          parse_energy_total_frame(frame);
          break;
        case kCodeTemp:
          parse_temp_frame(frame);
          break;
        default:
          break;
      }
    }
  }

  // 0x1403: 光伏电压/电池电压/充电电流
  void parse_voltage_frame(const CanFrame & frame)
  {
    using airship_utils::get_u16_le;
    using airship_utils::scale_u16;
    auto msg = airship_msgs::msg::MpptStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.device_addr = device_addr_;
    msg.can_id = frame.id;
    msg.pv_voltage = scale_u16(get_u16_le(frame.data, 0), 0.1f);
    msg.battery_voltage = scale_u16(get_u16_le(frame.data, 2), 0.1f);
    msg.charge_current = scale_u16(get_u16_le(frame.data, 4), 0.1f);
    msg.pv_power = msg.pv_voltage * msg.charge_current;
    status_pub_->publish(msg);
  }

  // 0x1404: 充电状态/故障状态
  void parse_state_frame(const CanFrame & frame)
  {
    using airship_utils::get_u16_le;
    auto msg = airship_msgs::msg::MpptStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.device_addr = device_addr_;
    msg.can_id = frame.id;
    msg.charge_state = get_u16_le(frame.data, 0);
    msg.fault_state = get_u16_le(frame.data, 4);
    status_pub_->publish(msg);
  }

  // 0x1405: 日/月发电量 (0.1kWh)
  void parse_energy_frame(const CanFrame & frame)
  {
    using airship_utils::get_u32_le;
    using airship_utils::scale_u16;
    using airship_utils::get_u16_le;
    auto msg = airship_msgs::msg::MpptStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.device_addr = device_addr_;
    msg.can_id = frame.id;
    msg.energy_today = static_cast<float>(get_u32_le(frame.data, 0)) * 0.1f;
    msg.energy_month = static_cast<float>(get_u32_le(frame.data, 4)) * 0.1f;
    status_pub_->publish(msg);
  }

  // 0x1406: 总发电量
  void parse_energy_total_frame(const CanFrame & frame)
  {
    using airship_utils::get_u32_le;
    auto msg = airship_msgs::msg::MpptStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.device_addr = device_addr_;
    msg.can_id = frame.id;
    msg.energy_total = static_cast<float>(get_u32_le(frame.data, 0)) * 0.1f;
    status_pub_->publish(msg);
  }

  // 0x1408: 机内温度/模块温度 (S16, 0.1℃, 负温度反码)
  void parse_temp_frame(const CanFrame & frame)
  {
    using airship_utils::get_i16_le;
    using airship_utils::scale_i16;
    auto msg = airship_msgs::msg::MpptStatus();
    msg.header.stamp = this->now();
    msg.online = true;
    msg.device_addr = device_addr_;
    msg.can_id = frame.id;
    msg.air_temp = scale_i16(get_i16_le(frame.data, 2), 0.1f);
    msg.module_temp = scale_i16(get_i16_le(frame.data, 4), 0.1f);
    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  std::string can_if_;
  SocketCanInterface can_;
  std::atomic<bool> running_;
  std::thread control_thread_;
  std::thread receive_thread_;

  double query_rate_hz_;
  uint8_t device_addr_;

  rclcpp::Publisher<airship_msgs::msg::MpptStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpptNode>());
  rclcpp::shutdown();
  return 0;
}
