// 灵云01号伴飞电脑 — DCDC 硬件保活进程 (dcdc_hold)
//
// 目的: 保证 DCDC 输出在极端情况下(ROS2/launch/DDS 崩溃)仍能持续。
//       DCDC 控制帧必须 200ms 周期持续下发, 否则 DCDC 停机断电。
//       本进程为独立 systemd 服务, 不依赖 ROS2/DDS, 由 systemd 保证重启。
//
// 设计原则:
//   - 无 ROS 依赖, 仅 socketcan + 协议库
//   - 周期发送电源控制帧 + 模拟量查询帧
//   - CAN 接口掉线自动重连 (ensure_open)
//   - 参数经环境变量注入 (便于 systemd 管理, 与 airship_params.yaml 保持一致)
//
// 环境变量:
//   DCDC_CAN_IF       CAN 接口 (默认 can0)
//   DCDC_ENABLED      1=开机 0=关机 (默认 1)
//   DCDC_SET_VOLTAGE  输出电压 V (默认 48.0)
//   DCDC_SET_CURRENT  限流 A (默认 80.0)
//   DCDC_PERIOD_MS    发送周期 ms (默认 200)
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <unistd.h>

#include "airship_can/can_interface.hpp"
#include "airship_dcdc/dcdc_protocol.hpp"

using airship_can::CanFrame;
using airship_can::SocketCanInterface;

namespace
{

// 读取环境变量, 缺省时返回默认值
std::string env_str(const char * name, const std::string & def)
{
  const char * v = std::getenv(name);
  return (v != nullptr && v[0] != '\0') ? std::string(v) : def;
}

// 读取 double 环境变量; 缺失/非法(非数字)时回退默认值, 避免 atof 解析得 0 的危险
double env_double(const char * name, double def)
{
  const char * v = std::getenv(name);
  if (v == nullptr || v[0] == '\0') {
    return def;
  }
  char * end = nullptr;
  const double val = std::strtod(v, &end);
  if (end == v || *end != '\0') {
    fprintf(stderr, "[dcdc_hold] 环境变量 %s='%s' 非法, 使用默认值 %.3g\n", name, v, def);
    return def;
  }
  return val;
}

// 读取 int 环境变量; 缺失/非法时回退默认值
int env_int(const char * name, int def)
{
  const char * v = std::getenv(name);
  if (v == nullptr || v[0] == '\0') {
    return def;
  }
  char * end = nullptr;
  const int64_t val = std::strtol(v, &end, 10);
  if (end == v || *end != '\0') {
    fprintf(stderr, "[dcdc_hold] 环境变量 %s='%s' 非法, 使用默认值 %d\n", name, v, def);
    return def;
  }
  return static_cast<int>(val);
}

}  // namespace

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  const std::string can_if = env_str("DCDC_CAN_IF", "can0");
  const bool enabled = env_int("DCDC_ENABLED", 1) != 0;
  const float set_voltage = static_cast<float>(env_double("DCDC_SET_VOLTAGE", 48.0));
  const float set_current = static_cast<float>(env_double("DCDC_SET_CURRENT", 80.0));
  const int period_ms = env_int("DCDC_PERIOD_MS", 200);
  if (period_ms < 50) {
    // 防止周期过小导致 CAN 发送过载
    fprintf(stderr, "[dcdc_hold] DCDC_PERIOD_MS 过小(需>=50), 使用 200\n");
  }
  const int safe_period_ms = (period_ms < 50) ? 200 : period_ms;

  fprintf(
    stderr,
    "[dcdc_hold] start can=%s enabled=%d V=%.1f A=%.1f period=%dms\n",
    can_if.c_str(), enabled ? 1 : 0, set_voltage, set_current, safe_period_ms);

  SocketCanInterface can(can_if);
  const auto period = std::chrono::milliseconds(safe_period_ms);
  // 重连失败日志节流: 避免每周期刷屏, 每 5s 打印一次
  const auto log_every = std::chrono::milliseconds(5000);
  auto last_log = std::chrono::steady_clock::now() - log_every;

  while (true) {
    if (!can.ensure_open()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_log >= log_every) {
        last_log = now;
        fprintf(
          stderr, "[dcdc_hold] CAN %s 不可用, 持续重连中...\n", can_if.c_str());
      }
      std::this_thread::sleep_for(period);
      continue;
    }

    // 电源控制帧 + 模拟量查询帧 (检查发送结果, 失败时告警)
    const bool ctrl_ok = can.send(airship_dcdc::build_control_frame(
        enabled, set_voltage, set_current));
    const bool analog_ok = can.send(airship_dcdc::build_analog_query_frame());
    if (!ctrl_ok || !analog_ok) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_log >= log_every) {
        last_log = now;
        fprintf(
          stderr, "[dcdc_hold] CAN 发送失败(control=%d analog=%d), 链路可能异常\n",
          ctrl_ok ? 1 : 0, analog_ok ? 1 : 0);
      }
    }

    std::this_thread::sleep_for(period);
  }

  return 0;
}
