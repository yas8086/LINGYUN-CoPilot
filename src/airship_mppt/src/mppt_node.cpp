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
#include "airship_utils/offline_utils.hpp"

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
    // 发布话题名: 支持多台 MPPT 各自独立话题 (如 /mppt1/status, /mppt2/status)
    topic_name_ = this->declare_parameter("topic_name", std::string("/mppt/status"));
    // 无数据超时 (s): 超过该时长未收到任何有效帧, 判定设备离线, 兜底发布 online=false
    link_timeout_s_ = this->declare_parameter("link_timeout_s", 3.0);
    if (link_timeout_s_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "link_timeout_s 非法值 %.3f, 重置为 3.0", link_timeout_s_);
      link_timeout_s_ = 3.0;
    }

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::MpptStatus>(topic_name_, rclcpp::QoS(10));

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
      const auto cycle_start = std::chrono::steady_clock::now();
      // USB-CAN 插拔/接口重启后自动重连
      if (can_.ensure_open()) {
        for (ReadCode code : codes) {
          // 以设备地址作为源地址发送, 避免用 0x00(广播) 请求导致 MPPT 不应答
          if (!can_.send(airship_mppt::build_query_frame(code, device_addr_))) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "MPPT 查询帧发送失败(接口 %s, 设备地址 %u)", can_if_.c_str(), device_addr_);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "CAN 接口 %s 不可用, 重连中", can_if_.c_str());
      }
      // 周期调度: 扣除本轮查询已消耗(7 帧×20ms≈140ms + 发送耗时)后只 sleep 剩余量,
      // 使实际查询周期贴近 query_rate_hz(旧实现固定 sleep period 导致实际偏慢 ~15%)。
      const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
      const auto remain = period - elapsed;
      if (remain > std::chrono::milliseconds::zero()) {
        std::this_thread::sleep_for(remain);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 已超周期, 最小喘息防忙转
      }
    }
  }

  // 接收线程: 从机回应帧 ID 为 0x14[code]A1[dev]
  void receive_loop()
  {
    while (running_.load()) {
      // 无数据超时兜底: 距上次有效数据超过 link_timeout_s 后周期发布 online=false,
      // 让下游能感知设备失联(而非停留在最后一次旧数据)。
      // 注意: 该检查必须在循环顶部无条件执行——旧实现放在 receive()==false 分支内,
      // 双 MPPT 部署下总线上持续存在另一台设备的查询/回应帧(无关帧), receive()
      // 总能返回数据导致检查被持续跳过("饿死"), 失联检测会随查询频率升高而失效;
      // CAN 接口不可用期间同样会跳过检查。
      {
        const auto now = this->now();
        if (airship_utils::should_publish_offline(
            now.seconds(), last_data_time_.seconds(),
            last_offline_pub_time_.seconds(), link_timeout_s_))
        {
          last_offline_pub_time_ = now;
          publish_status(false);
        }
      }
      if (!can_.ensure_open()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      CanFrame frame{};
      if (!can_.receive(frame, 100)) {
        continue;
      }
      // 帧 ID 匹配(纯函数): 校验只读类型/目标标志/源设备地址, 返回只读段 code;
      // nullopt 表示非本设备或非法帧, 直接忽略(防多设备总线串扰误解析)。
      const auto code = airship_mppt::match_response_id(frame.id, device_addr_);
      if (!code) {
        continue;
      }
      switch (*code) {
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
          continue;  // match_response_id 已保证不会是未知 code, 此分支仅为编译完备
      }
      // 只有解析到本设备有效帧才刷新在线时间戳; 避免总线上其他设备的不相关帧
      // "喂新鲜" last_data_time_, 从而让 offline 兜底失效、设备失联仍误报在线。
      last_data_time_ = this->now();
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
    // 协议 0x03 实时段无光伏电流字段, 真实光伏功率不可得;
    // 用 电池电压×充电电流 近似(即充电功率, 略低于光伏功率, 差值为 MPPT 效率损耗)。
    // 旧实现误用 光伏电压(实测可到 320V)×充电电流, 高估约 25%+。
    msg.pv_power = mppt_data_.battery_voltage * mppt_data_.charge_current;

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
  std::string topic_name_;

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
