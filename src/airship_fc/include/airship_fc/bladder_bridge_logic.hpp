// 灵云01号飞艇伴飞电脑 — 气囊压差桥接核心逻辑(纯逻辑, 无 rclcpp 依赖)
//
// 从 bladder_bridge_node.cpp 的核心语义提取可单测的状态机:
//   - LoRa node_id -> 固定槽位(左副囊/左主囊/右主囊/右副囊)映射
//   - 槽位有效/沿用/NaN 三态语义(与 AirshipBladderPressure.msg 注释一致):
//       本轮有效(stale=0 && press_valid=1 && online=1) -> valid=1, stale=0, 实测值
//       本轮无效/节点缺失且有历史                        -> valid=0, stale=1, 沿用缓存
//       本轮无效且无历史                                 -> valid=0, stale=0, NaN
// 节点层保留: ROS 消息转换(LoRaSamples->SampleInput)、px4_msgs 发布。
//
// 关键语义(2026-09-03 设计): LoRa 去抖沿用轮(stale=1)里 press_valid 是被
// 拷贝的旧值, 不可信——bridge 必须按 stale 先判, 不被拷贝来的 press_valid=1 迷惑。
#ifndef AIRSHIP_FC__BLADDER_BRIDGE_LOGIC_HPP_
#define AIRSHIP_FC__BLADDER_BRIDGE_LOGIC_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace bladder_bridge
{

constexpr std::size_t kNumBladders = 4;

// 槽位输出(POD)
struct BladderSlot
{
  float pressure_pa = std::numeric_limits<float>::quiet_NaN();  // 压差 [Pa]
  float temp_c = std::numeric_limits<float>::quiet_NaN();       // 温度 [degC]
  bool valid = false;   // true=本轮真实测量
  bool stale = false;   // true=沿用历史值(非本轮实测)
};

// 归一化输入(节点层从 airship_msgs/LoRaSample 平移填充, 库不依赖 ROS 消息)
struct SampleInput
{
  int32_t node_id = -1;
  float temp_c = 0.0f;
  double pressure_pa = 0.0;
  int32_t online = 0;
  int32_t stale = 0;
  int32_t temp_valid = 0;
  int32_t press_valid = 0;
};

// 槽位状态机: node_id->固定槽位匹配 + 缺失/无效沿用最近有效值
class BladderBridgeState
{
public:
  explicit BladderBridgeState(std::array<int32_t, kNumBladders> node_ids)
  : node_ids_(node_ids) {}

  // 一轮 LoRaSamples -> 4 槽位; 内部保存最近有效值用于沿用。
  // 乱序输入不影响槽位映射; 无关节点忽略; 同 node_id 多条取第一条(防御)。
  std::array<BladderSlot, kNumBladders> update(const std::vector<SampleInput> & round)
  {
    std::array<BladderSlot, kNumBladders> out{};

    // 本轮各槽位匹配到的样本索引(-1=缺失)
    std::array<int, kNumBladders> matched{-1, -1, -1, -1};
    for (std::size_t i = 0; i < round.size(); ++i) {
      for (std::size_t s = 0; s < kNumBladders; ++s) {
        if (round[i].node_id == node_ids_[s]) {
          if (matched[s] < 0) {   // 同 node_id 取第一条
            matched[s] = static_cast<int>(i);
          }
          break;
        }
      }
    }

    for (std::size_t s = 0; s < kNumBladders; ++s) {
      // 压力: stale=1 的去抖沿用轮里 press_valid 是拷贝的旧值不可信, 必须先判 stale
      const bool press_ok = matched[s] >= 0 &&
        round[matched[s]].stale == 0 &&
        round[matched[s]].press_valid == 1 &&
        round[matched[s]].online == 1;
      if (press_ok) {
        const SampleInput & in = round[matched[s]];
        out[s].pressure_pa = static_cast<float>(in.pressure_pa);
        out[s].valid = true;
        out[s].stale = false;
        last_press_[s] = out[s].pressure_pa;  // 更新沿用缓存
        has_press_[s] = true;
      } else if (has_press_[s]) {
        out[s].pressure_pa = last_press_[s];
        out[s].valid = false;
        out[s].stale = true;
      }  // 无缓存: 保持 NaN, valid=0, stale=0

      // 温度独立同规则处理(温度失败不影响压力槽位状态)
      const bool temp_ok = matched[s] >= 0 &&
        round[matched[s]].stale == 0 &&
        round[matched[s]].temp_valid == 1 &&
        round[matched[s]].online == 1;
      if (temp_ok) {
        out[s].temp_c = round[matched[s]].temp_c;
        last_temp_[s] = out[s].temp_c;
        has_temp_[s] = true;
      } else if (has_temp_[s]) {
        out[s].temp_c = last_temp_[s];
      }  // 无缓存: 保持 NaN
    }
    return out;
  }

  const std::array<int32_t, kNumBladders> & node_ids() const {return node_ids_;}

private:
  std::array<int32_t, kNumBladders> node_ids_;
  std::array<float, kNumBladders> last_press_{};  // 最近有效压差缓存
  std::array<float, kNumBladders> last_temp_{};   // 最近有效温度缓存
  std::array<bool, kNumBladders> has_press_{};    // 是否见过有效压差
  std::array<bool, kNumBladders> has_temp_{};     // 是否见过有效温度
};

}  // namespace bladder_bridge

#endif  // AIRSHIP_FC__BLADDER_BRIDGE_LOGIC_HPP_
