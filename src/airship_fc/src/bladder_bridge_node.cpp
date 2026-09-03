// 灵云01号飞艇伴飞电脑 — 气囊压差桥接节点 (LoRa -> uXRCE-DDS -> PX4 飞控)
//
// 订阅 /lora/samples (airship_msgs/LoRaSamples, lora_node 2s 周期轮询产出),
// 提取 4 路压力节点(固定槽位: 左副囊/左主囊/右主囊/右副囊), 发布
// px4_msgs/AirshipBladderPressure 到 /fmu/in/airship_bladder_pressure。
//
// 链路: 本节点 -> MicroXRCEAgent(树莓派 UDP 8888) -> PX4 uxrce_dds_client
//       -> uORB airship_bladder_pressure -> 飞控控制逻辑
// 核心槽位语义(有效/沿用/NaN 三态)由 bladder_bridge_logic.hpp 纯逻辑库实现。
//
// 时间戳: PX4 v1.17 uXRCE-DDS client 内置 XRCE PING/PONG timesync, 收到消息
// 自动把树莓派系统时钟换算为 PX4 时钟, 故 timestamp 填 now() 微秒即可。
//
// QoS: 发布端用默认 reliable+volatile。PX4 侧 datareader 为 BEST_EFFORT,
// 按 RTPS 规则 pub(reliable) >= reader(best_effort) 兼容; reliable 还能让
// ros2 topic echo(默认 reliable)直接订阅验证, 无需额外 QoS 参数。
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "airship_msgs/msg/lo_ra_samples.hpp"
#include "px4_msgs/msg/airship_bladder_pressure.hpp"
#include "rclcpp/rclcpp.hpp"

#include "airship_fc/bladder_bridge_logic.hpp"

namespace
{

using airship_msgs::msg::LoRaSamples;
using px4_msgs::msg::AirshipBladderPressure;

}  // namespace

class BladderBridgeNode : public rclcpp::Node
{
public:
  explicit BladderBridgeNode(const rclcpp::NodeOptions & options)
  : Node("bladder_bridge_node", options)
  {
    // 槽位映射(顺序即槽位, 不可重排): [左副囊, 左主囊, 右主囊, 右副囊]
    const auto ids = this->declare_parameter<std::vector<int64_t>>(
      "bladder_node_ids", std::vector<int64_t>{6, 13, 14, 15});
    if (ids.size() != bladder_bridge::kNumBladders) {
      RCLCPP_FATAL(
        this->get_logger(),
        "bladder_node_ids 必须恰好 %zu 个(当前 %zu), 顺序=槽位 [左副囊,左主囊,右主囊,右副囊]",
        bladder_bridge::kNumBladders, ids.size());
      throw std::invalid_argument("bladder_node_ids size");
    }
    std::array<int32_t, bladder_bridge::kNumBladders> node_ids{};
    for (std::size_t i = 0; i < ids.size(); ++i) {
      node_ids[i] = static_cast<int32_t>(ids[i]);
    }
    state_ = std::make_unique<bladder_bridge::BladderBridgeState>(node_ids);

    const std::string sub_topic = this->declare_parameter("sub_topic", "/lora/samples");
    const std::string pub_topic =
      this->declare_parameter("pub_topic", "/fmu/in/airship_bladder_pressure");

    pub_ = this->create_publisher<AirshipBladderPressure>(pub_topic, 10);
    sub_ = this->create_subscription<LoRaSamples>(
      sub_topic, 10,
      [this](const LoRaSamples::SharedPtr msg) {on_samples(msg);});

    RCLCPP_INFO(
      this->get_logger(),
      "气囊压差桥已启动: %s(node_id %d/%d/%d/%d) -> %s, 槽位 [左副囊,左主囊,右主囊,右副囊]",
      sub_topic.c_str(), node_ids[0], node_ids[1], node_ids[2], node_ids[3],
      pub_topic.c_str());
  }

private:
  void on_samples(const LoRaSamples::SharedPtr msg)
  {
    // ROS 消息 -> 归一化输入(纯逻辑库不依赖 ROS 类型)
    std::vector<bladder_bridge::SampleInput> round;
    round.reserve(msg->samples.size());
    for (const auto & s : msg->samples) {
      bladder_bridge::SampleInput in;
      in.node_id = s.node_id;
      in.temp_c = s.temp_celsius;
      in.pressure_pa = s.pressure_pa;
      in.online = s.online;
      in.stale = s.stale;
      in.temp_valid = s.temp_valid;
      in.press_valid = s.press_valid;
      round.push_back(in);
    }

    const auto slots = state_->update(round);

    AirshipBladderPressure out;
    out.timestamp = static_cast<uint64_t>(
      this->now().nanoseconds()) / 1000ULL;  // 树莓派系统时钟微秒, client timesync 自动换算
    out.timestamp_sample = static_cast<uint64_t>(msg->timestamp.sec) * 1000000ULL +
      msg->timestamp.nanosec / 1000ULL;
    for (std::size_t i = 0; i < bladder_bridge::kNumBladders; ++i) {
      out.pressure_delta_pa[i] = slots[i].pressure_pa;
      out.temperature_c[i] = slots[i].temp_c;
      out.valid[i] = slots[i].valid ? 1 : 0;
      out.stale[i] = slots[i].stale ? 1 : 0;
    }
    pub_->publish(out);
  }

  std::unique_ptr<bladder_bridge::BladderBridgeState> state_;
  rclcpp::Publisher<AirshipBladderPressure>::SharedPtr pub_;
  rclcpp::Subscription<LoRaSamples>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BladderBridgeNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
