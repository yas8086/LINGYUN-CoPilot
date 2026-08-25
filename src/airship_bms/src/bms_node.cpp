// 灵云01号伴飞电脑 — 锂电池 BMS 驱动节点
// 协议: bms_ems_v01 (CAN DBC)
//
// 功能:
//   1. 接收并解析 BMS 上报的 CAN 帧
//   2. 按帧 ID 路由到 bms_protocol 解析
//   3. 累积结果, 发布 BmsStatus
//
// 协议解析逻辑见 airship_bms::bms_protocol (纯库, 可 gtest)。
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "airship_bms/bms_protocol.hpp"
#include "airship_can/can_interface.hpp"
#include "airship_msgs/msg/bms_status.hpp"
#include "airship_utils/offline_utils.hpp"

using airship_bms::BmsData;
using airship_can::CanFrame;
using airship_can::SocketCanInterface;

class BmsNode : public rclcpp::Node
{
public:
  explicit BmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("bms_node", options),
    can_("can0"),
    running_(false)
  {
    // ===== 参数 =====
    can_if_ = this->declare_parameter("can_interface", std::string("can0"));
    // 电芯数: 默认 102(本项目主BMS)。旧默认 0 时 cell_voltages 恒空、max/min 恒 NaN,
    // 无参数运行时出现空数据; 增加 ≥1 下限校验防负值经 uint16 回绕(如 -1 → 65535)。
    {
      int cell_count_param = this->declare_parameter("cell_count", 102);
      if (cell_count_param <= 0) {
        RCLCPP_WARN(this->get_logger(), "cell_count=%d 非法(须为正), 重置为 102", cell_count_param);
        cell_count_param = 102;
      }
      cell_count_ = static_cast<uint16_t>(cell_count_param);
    }
    // 发布节流间隔 (s): 102 串 BMS 高频发帧(30+ 帧/100ms), 逐帧发布会达 300+ 次/秒,
    // 每次都要 assign 复制 102 个 float。按此间隔聚合发布一次(默认 0.2s => 5Hz)。
    pub_interval_s_ = this->declare_parameter("pub_interval_s", 0.2);
    if (pub_interval_s_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "pub_interval_s 非法值 %.3f, 重置为 0.2", pub_interval_s_);
      pub_interval_s_ = 0.2;
    }
    // 无数据超时 (s): 超过该时长未收到任何有效帧, 判定设备离线, 兜底发布 online=false
    link_timeout_s_ = this->declare_parameter("link_timeout_s", 3.0);
    if (link_timeout_s_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "link_timeout_s 非法值 %.3f, 重置为 3.0", link_timeout_s_);
      link_timeout_s_ = 3.0;
    }
    // 上限防护: 配置电芯数超过协议上限时截断, 避免 cell_voltages 数组越界
    if (cell_count_ > airship_bms::kMaxCells) {
      RCLCPP_WARN(
        this->get_logger(), "cell_count=%u 超过协议上限 %u, 已截断",
        cell_count_, airship_bms::kMaxCells);
      cell_count_ = airship_bms::kMaxCells;
    }

    // ===== 发布器 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::BmsStatus>("/bms/status", rclcpp::QoS(10));

    can_ = SocketCanInterface(can_if_);
    if (!can_.open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CAN 接口 %s 失败, 稍后重试", can_if_.c_str());
    }

    running_ = true;
    last_data_time_ = this->now();
    last_offline_pub_time_ = this->now();
    last_pub_time_ = this->now();
    receive_thread_ = std::thread(&BmsNode::receive_loop, this);
  }

  ~BmsNode() override
  {
    running_ = false;
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    can_.close();
  }

private:
  // 接收线程: 按帧 ID 路由解析
  void receive_loop()
  {
    while (running_.load()) {
      // 无数据超时兜底: 距上次有效数据超过 link_timeout_s 后周期发布 online=false,
      // 让下游能感知设备失联(而非停留在最后一次旧数据)。
      // 注意: 该检查必须在循环顶部无条件执行——旧实现放在 receive()==false 分支内,
      // 总线上无关帧频繁到达时 receive() 总能返回数据导致检查被持续跳过("饿死"),
      // 设备失联的 offline 发布会被无限延迟; CAN 接口不可用期间同样会跳过检查。
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
      // USB-CAN 插拔/接口重启后自动重连
      if (!can_.ensure_open()) {
        was_disconnected_ = true;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "CAN 接口 %s 不可用, 3s 后重试", can_if_.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        continue;
      }
      // 从断连恢复: 清零累积数据, 避免重连后首帧未到期间沿用上一段陈旧值
      if (was_disconnected_) {
        was_disconnected_ = false;
        bms_data_ = airship_bms::BmsData{};
      }
      CanFrame frame{};
      if (!can_.receive(frame, 100)) {
        continue;
      }
      bool matched = true;
      using namespace airship_bms;  // NOLINT
      if (frame.id == kBattInfo02) {
        parse_batt_info(frame.data, frame.len, bms_data_);
      } else if (frame.id == kBattInfo01) {
        parse_batt_status(frame.data, frame.len, bms_data_);
      } else if (frame.id == kCellTempStatistic) {
        parse_cell_temp_statistic(frame.data, frame.len, bms_data_);
      } else if (frame.id == kPackTemp) {
        parse_pack_temp(frame.data, frame.len, bms_data_);
      } else if (frame.id == kErrorCode) {
        parse_error_code(frame.data, frame.len, bms_data_);
      } else if (frame.id == kSoh) {
        parse_soh(frame.data, frame.len, bms_data_);
      } else if (frame.id == kSop) {
        parse_sop(frame.data, frame.len, bms_data_);
      } else if (frame.id == kCellVoltStatistic) {
        parse_cell_volt_statistic(frame.data, frame.len, bms_data_);
      } else if (frame.id == kPoleTempStatistic) {
        parse_pole_temp_statistic(frame.data, frame.len, bms_data_);
      } else {
        // 单体电压帧: 每帧 5 节, 帧 ID 每帧 +0x10 (0x3000, 0x3010, ...)
        // 按实际电芯数计算所需帧数, 覆盖完整范围:
        //   102 节 -> 21 帧, 帧 ID 至 0x3000 + 0x10*20 = 0x3140
        // 旧实现固定 +0x100 仅覆盖 16 帧(节 1~80), 第 81~102 节被丢弃(安全盲区)
        const uint32_t cell_frames =
          (static_cast<uint32_t>(cell_count_) + airship_bms::kCellPerVoltFrame - 1) /
          airship_bms::kCellPerVoltFrame;
        const bool in_cell_range = frame.id >= kCellVoltageBase &&
          frame.id < kCellVoltageBase + 0x10u * cell_frames;
        if (in_cell_range) {
          parse_cell_voltage(frame.id, frame.data, frame.len, bms_data_);
        } else {
          matched = false;
        }
      }
      if (matched) {
        // 仅本设备有效帧刷新在线时间戳: 总线上其他设备的无关帧不应"喂新鲜"
        // last_data_time_, 否则设备失联后 offline 兜底永远不触发。
        last_data_time_ = this->now();
        // 发布节流: 按 pub_interval_s_ 聚合发布, 避免高频帧逐个发布 300+ 次/秒。
        // (帧间隔天然大于节流间隔时每帧仍会发布, 行为等价)
        const auto now = this->now();
        if ((now - last_pub_time_).seconds() >= pub_interval_s_) {
          last_pub_time_ = now;
          publish_status(true);
        }
      }
    }
  }

  // 发布聚合状态
  void publish_status(bool online)
  {
    auto msg = airship_msgs::msg::BmsStatus();
    msg.header.stamp = this->now();
    msg.online = online;
    msg.pack_voltage = bms_data_.pack_voltage;
    msg.pack_current = bms_data_.pack_current;
    msg.soc = bms_data_.soc;
    msg.real_soc = bms_data_.real_soc;
    // 可变长: 仅携带实际电芯数, 避免每帧固定序列化 256 个浮点浪费带宽
    msg.cell_voltages.assign(
      bms_data_.cell_voltages.begin(),
      bms_data_.cell_voltages.begin() + cell_count_);
    msg.fault_word1 = bms_data_.fault_word1;
    msg.fault_word2 = bms_data_.fault_word2;
    msg.fault_word3 = bms_data_.fault_word3;
    msg.soh = bms_data_.soh;
    msg.alarm_level = bms_data_.alarm_level;

    // 绝缘电阻与极耳温度
    msg.positive_insulation_kohm = bms_data_.positive_insulation_kohm;
    msg.negative_insulation_kohm = bms_data_.negative_insulation_kohm;
    for (uint32_t i = 0; i < airship_bms::kMaxPoleTemps; ++i) {
      msg.pole_temps[i] = bms_data_.pole_temps[i];
    }

    // 温度统计 (来自 CellTempStatistic)
    msg.max_cell_temp = bms_data_.max_cell_temp;
    msg.min_cell_temp = bms_data_.min_cell_temp;
    msg.avg_cell_temp = bms_data_.avg_cell_temp;
    msg.temp_diff = bms_data_.temp_diff;

    // 计算最高/最低单体电压与压差(基于已收到数据)
    msg.max_cell_voltage = std::numeric_limits<float>::quiet_NaN();
    msg.min_cell_voltage = std::numeric_limits<float>::quiet_NaN();
    msg.cell_voltage_diff = 0.0f;
    for (uint32_t i = 0; i < cell_count_; ++i) {
      const float v = bms_data_.cell_voltages[i];
      if (v <= 0.0f) {
        continue;  // 未收到该节数据
      }
      if (std::isnan(msg.max_cell_voltage) || v > msg.max_cell_voltage) {
        msg.max_cell_voltage = v;
      }
      if (std::isnan(msg.min_cell_voltage) || v < msg.min_cell_voltage) {
        msg.min_cell_voltage = v;
      }
    }
    if (!std::isnan(msg.max_cell_voltage) && !std::isnan(msg.min_cell_voltage)) {
      msg.cell_voltage_diff = msg.max_cell_voltage - msg.min_cell_voltage;
    }

    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  std::string can_if_;
  SocketCanInterface can_;
  std::atomic<bool> running_;
  std::thread receive_thread_;

  uint16_t cell_count_;
  double link_timeout_s_;
  double pub_interval_s_;
  BmsData bms_data_;
  bool was_disconnected_ = false;
  rclcpp::Time last_data_time_;
  rclcpp::Time last_offline_pub_time_;
  rclcpp::Time last_pub_time_;

  rclcpp::Publisher<airship_msgs::msg::BmsStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BmsNode>());
  rclcpp::shutdown();
  return 0;
}
