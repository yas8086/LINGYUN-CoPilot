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
    // 除零防护: 频率必须为正, 否则 1000.0/rate 产生 inf 导致 static_cast<int> 未定义行为
    if (query_rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "query_rate_hz 非法值 %.3f, 重置为 1.0", query_rate_hz_);
      query_rate_hz_ = 1.0;
    }
    device_addr_ = static_cast<uint8_t>(this->declare_parameter("device_addr", 1));
    // 无数据超时 (s): 超过该时长未收到任何有效帧, 判定设备离线, 兜底发布 online=false
    link_timeout_s_ = this->declare_parameter("link_timeout_s", 3.0);
    if (link_timeout_s_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "link_timeout_s 非法值 %.3f, 重置为 3.0", link_timeout_s_);
      link_timeout_s_ = 3.0;
    }

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::MpptStatus>("/mppt/status", rclcpp::QoS(10));

    can_ = SocketCanInterface(can_if_);
    if (!can_.open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CAN 接口 %s 失败, 稍后重试", can_if_.c_str());
    }

    running_ = true;
    last_data_time_ = this->now();
    last_offline_pub_time_ = this->now();
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
      // USB-CAN 插拔/接口重启后自动重连
      if (can_.ensure_open()) {
        for (ReadCode code : codes) {
          can_.send(airship_mppt::build_query_frame(code));
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "CAN 接口 %s 不可用, 重连中", can_if_.c_str());
      }
      std::this_thread::sleep_for(period);
    }
  }

  // 接收线程: 从机回应帧 ID 为 0x14[code]A1[dev]
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
      // 校验只读类型与目标地址
      const uint8_t type = static_cast<uint8_t>((frame.id >> 24) & 0xFF);
      const uint8_t target = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
      if (type != airship_mppt::kReadType || target != airship_mppt::kTargetAddr) {
        continue;
      }
      // 校验源地址与期望设备一致, 避免多设备总线串扰导致误解析
      const uint8_t src = static_cast<uint8_t>(frame.id & 0xFF);
      if (src != device_addr_) {
        continue;
      }
      const uint8_t code = static_cast<uint8_t>((frame.id >> 16) & 0xFF);
      switch (static_cast<ReadCode>(code)) {
        case ReadCode::kCodeRated:
          airship_mppt::parse_rated(frame.data, frame.len, mppt_data_);
          break;
        case ReadCode::kCodeRealtime:
          airship_mppt::parse_realtime(frame.data, frame.len, mppt_data_);
          break;
        case ReadCode::kCodeState:
          airship_mppt::parse_state(frame.data, frame.len, mppt_data_);
          break;
        case ReadCode::kCodeEnergyDay:
          airship_mppt::parse_energy_day(frame.data, frame.len, mppt_data_);
          break;
        case ReadCode::kCodeEnergyTotal:
          airship_mppt::parse_energy_total(frame.data, frame.len, mppt_data_);
          break;
        case ReadCode::kCodeTemp:
          airship_mppt::parse_temp(frame.data, frame.len, mppt_data_);
          break;
        case ReadCode::kCodeControl:
          airship_mppt::parse_control(frame.data, frame.len, mppt_data_);
          break;
        default:
          continue;
      }
      publish_status(true);
    }
  }

  // 发布聚合状态
  void publish_status(bool online)
  {
    auto msg = airship_msgs::msg::MpptStatus();
    msg.header.stamp = this->now();
    msg.online = online;
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
  double link_timeout_s_;
  uint8_t device_addr_;

  MpptData mppt_data_;
  rclcpp::Time last_data_time_;
  rclcpp::Time last_offline_pub_time_;

  rclcpp::Publisher<airship_msgs::msg::MpptStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpptNode>());
  rclcpp::shutdown();
  return 0;
}
