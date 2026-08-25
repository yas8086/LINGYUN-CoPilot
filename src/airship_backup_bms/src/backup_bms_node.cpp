// 灵云01号伴飞电脑 — 12S 备用电源 BMS 驱动节点 (backup_bms_node)
//
// 通过串口(默认 9600 8N1)按北辰协议 V1.4 轮询 12S 备用电源 BMS:
//   0x06 基本信息1   -> 总压/总电流/SOC/SOH/电压温度统计/告警/保护/故障/系统状态
//   0x08 单节电压    -> 逐节单体电压
//   0x07 单节温度    -> 逐点温度
// 解析结果聚合发布到 /backup_bms/status (airship_msgs/BackupBmsStatus)。
//
// 协议解析见 airship_backup_bms::backup_bms_protocol (纯库, 可 gtest);
// 串口复用 airship_link::SerialInterface, 支持掉线自动重连(USB/串口插拔后自愈)。
//
// 参数:
//   serial_device   串口设备 (建议 udev 符号链接 /dev/airship_backup_bms)
//   baud_rate       波特率 (默认 9600)
//   addr            从机地址 (默认 0xFF)
//   host            主机信息 (上位机 0x10)
//   sample_period_ms  轮询周期 (ms, 默认 1000)
//   resp_timeout_ms   单次响应超时 (ms, 默认 200)
//   reconnect_ms      串口重连周期 (ms, 默认 2000)
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "airship_backup_bms/backup_bms_protocol.hpp"
#include "airship_link/serial_interface.hpp"
#include "airship_msgs/msg/backup_bms_status.hpp"

using airship_backup_bms::BackupBmsData;
using airship_backup_bms::build_read_request;
using airship_backup_bms::kCmdBasicInfo;
using airship_backup_bms::kCmdCellTemp;
using airship_backup_bms::kCmdCellVoltage;
using airship_backup_bms::parse_basic_info;
using airship_backup_bms::parse_cell_temps;
using airship_backup_bms::parse_cell_voltages;
using airship_backup_bms::parse_response_frame;

class BackupBmsNode : public rclcpp::Node
{
public:
  explicit BackupBmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("backup_bms_node", options),
    serial_("/dev/airship_backup_bms"),
    running_(false)
  {
    // ===== 参数 =====
    serial_device_ = this->declare_parameter("serial_device",
      std::string("/dev/airship_backup_bms"));
    baud_rate_ = this->declare_parameter("baud_rate", 9600);
    addr_ = static_cast<uint8_t>(this->declare_parameter("addr", 0xFF));
    host_ = static_cast<uint8_t>(this->declare_parameter("host", 0x10));
    sample_period_ms_ = this->declare_parameter("sample_period_ms", 1000);
    resp_timeout_ms_ = this->declare_parameter("resp_timeout_ms", 200);
    reconnect_ms_ = this->declare_parameter("reconnect_ms", 2000);
    // 轮询周期下限保护: 过小会导致 485/串口无法及时响应
    if (sample_period_ms_ < 100) {
      RCLCPP_WARN(this->get_logger(), "sample_period_ms 过小(%d), 重置为 100", sample_period_ms_);
      sample_period_ms_ = 100;
    }

    // ===== 发布器 =====
    status_pub_ = this->create_publisher<airship_msgs::msg::BackupBmsStatus>(
      "/backup_bms/status", rclcpp::QoS(10));

    // ===== 打开串口 =====
    serial_ = airship_link::SerialInterface(serial_device_);
    serial_online_.store(serial_.open(static_cast<airship_link::BaudRate>(baud_rate_)));
    if (!serial_online_.load()) {
      RCLCPP_WARN(this->get_logger(), "打开串口 %s 失败, 进入自动重连", serial_device_.c_str());
    }

    // ===== 轮询线程 =====
    running_ = true;
    poll_thread_ = std::thread(&BackupBmsNode::poll_loop, this);
  }

  ~BackupBmsNode() override
  {
    running_ = false;
    if (poll_thread_.joinable()) {
      poll_thread_.join();
    }
    serial_.close();
  }

private:
  // 解析函数分派类型: 区分轮询不同指令时对应的解析函数
  enum class PollKind
  {
    kBasic,     // 基本信息1 (0x06)
    kVoltages,  // 单节电压 (0x08)
    kTemps,     // 单节温度 (0x07)
  };

  // 确保串口已打开; 未打开则尝试重连。返回串口当前是否可用。
  bool ensure_serial()
  {
    if (serial_online_.load()) {
      return true;
    }
    serial_.close();
    const bool ok = serial_.open(static_cast<airship_link::BaudRate>(baud_rate_));
    const int attempt = ++reconnect_attempt_;
    if (ok) {
      serial_online_.store(true);
      reconnect_attempt_ = 0;
      RCLCPP_INFO(this->get_logger(), "[reconnect] 串口 %s 重连成功 (第 %d 次)", serial_device_.c_str(),
        attempt);
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "[reconnect] 串口 %s 重连失败 (第 %d 次)", serial_device_.c_str(), attempt);
    return false;
  }

  // 读取完整响应帧: 先读头部定位长度, 再读数据+CRC。
  // 返回响应帧(不含起始噪声); 失败返回空 vector。
  std::vector<uint8_t> read_response()
  {
    std::vector<uint8_t> frame;
    char buf[256];
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(resp_timeout_ms_);

    // 阶段1: 寻找起始字节 0x57 (跳过总线噪声/残留)
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      const int remaining = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
      if (remaining <= 0) {
        return {};  // 超时未找到起始字节
      }
      const int n = serial_.read(buf, 1, remaining);
      if (n < 0) {
        return {};  // 读错误
      }
      if (n == 0) {
        continue;
      }
      if (static_cast<uint8_t>(buf[0]) == 0x57) {
        frame.push_back(static_cast<uint8_t>(buf[0]));
        break;
      }
    }

    // 阶段2: 读取剩余头部(地址/主机/读写/指令/长度 6 字节)
    while (frame.size() < 7) {
      const auto now = std::chrono::steady_clock::now();
      const int remaining = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
      if (remaining <= 0) {
        return {};
      }
      const int n = serial_.read(buf, 7 - frame.size(), remaining);
      if (n <= 0) {
        return {};
      }
      frame.insert(frame.end(), buf, buf + n);
    }

    // 阶段3: 读取数据 + CRC
    const uint32_t dlen = (static_cast<uint32_t>(frame[5]) << 8) | frame[6];
    const size_t total = 7 + dlen + 2;
    if (total > sizeof(buf)) {
      return {};  // 长度异常, 拒绝
    }
    while (frame.size() < total) {
      const auto now = std::chrono::steady_clock::now();
      const int remaining = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
      if (remaining <= 0) {
        return {};
      }
      const int n = serial_.read(buf, total - frame.size(), remaining);
      if (n <= 0) {
        return {};
      }
      frame.insert(frame.end(), buf, buf + n);
    }
    return frame;
  }

  // 发送读请求并校验响应, 成功返回 true
  // parse_kind: 区分同一 cmd 映射的解析函数 (见 PollKind)
  bool query(uint8_t cmd, BackupBmsData & out, PollKind kind)
  {
    const auto req = build_read_request(addr_, host_, cmd);
    serial_.flush_rx();
    if (!serial_.write(reinterpret_cast<const char *>(req.data()), req.size())) {
      return false;
    }
    const auto frame = read_response();
    if (frame.empty()) {
      return false;
    }
    const uint8_t * data = nullptr;
    uint32_t dlen = 0;
    if (!parse_response_frame(
        frame.data(), static_cast<uint32_t>(frame.size()), addr_, host_, cmd, &data, &dlen))
    {
      return false;
    }
    switch (kind) {
      case PollKind::kBasic: return parse_basic_info(data, dlen, out);
      case PollKind::kVoltages: return parse_cell_voltages(data, dlen, out);
      case PollKind::kTemps: return parse_cell_temps(data, dlen, out);
    }
    return false;
  }

  // 单轮轮询: 依次读 0x06 / 0x08 / 0x07
  // 返回值 = online 判定: 仅由 0x06(基本信息) 成功决定——该帧携带 safety 判据
  //   依赖的 pack_voltage/fault_word, 若其失败而电压/温度子查询成功仍报在线,
  //   会用陈旧 fault_word 维持"安全"判定(旧实现 `|| ok` 即此缺陷)。
  // 输出参数 link_alive = 任一指令有响应, 作为串口物理链路存活的判定依据
  //   (避免"基本信息失败但电压正常响应"的半故障场景被误判为串口掉线)。
  bool poll_once(BackupBmsData & out, bool & link_alive)
  {
    const bool basic_ok = query(kCmdBasicInfo, out, PollKind::kBasic);
    const bool volt_ok = query(kCmdCellVoltage, out, PollKind::kVoltages);
    const bool temp_ok = query(kCmdCellTemp, out, PollKind::kTemps);
    link_alive = basic_ok || volt_ok || temp_ok;
    return basic_ok;
  }

  void publish_status(const BackupBmsData & d, bool online)
  {
    auto msg = airship_msgs::msg::BackupBmsStatus();
    msg.header.stamp = this->now();
    msg.online = online;
    msg.pack_voltage = d.pack_voltage;
    msg.pack_current = d.pack_current;
    msg.soc = d.soc;
    msg.soh = d.soh;
    msg.max_cell_voltage = d.max_cell_voltage;
    msg.min_cell_voltage = d.min_cell_voltage;
    msg.cell_voltage_diff = d.cell_voltage_diff;
    msg.max_cell_temp = d.max_cell_temp;
    msg.min_cell_temp = d.min_cell_temp;
    msg.avg_cell_temp = d.avg_cell_temp;
    msg.temp_diff = d.temp_diff;
    msg.cell_voltages = d.cell_voltages;
    msg.cell_temps = d.cell_temps;
    msg.alarm_word = d.alarm_word;
    msg.protect_word = d.protect_word;
    msg.fault_word = d.fault_word;
    msg.system_word = d.system_word;
    status_pub_->publish(msg);
  }

  // 轮询主循环 (固定节拍)
  void poll_loop()
  {
    auto next_wake = std::chrono::steady_clock::now();
    while (running_.load()) {
      // 串口掉线时优先重连
      if (!serial_online_.load() && !ensure_serial()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
        next_wake = std::chrono::steady_clock::now();
        continue;
      }

      // 维护"上一帧有效数据", 失败轮次沿用旧值仅置 online=false,
      // 避免发布 online=false 时数值被清零而被下游误判为真实测量 0。
      BackupBmsData data = last_good_data_;
      bool link_alive = false;
      const bool ok = poll_once(data, link_alive);
      if (ok) {
        last_good_data_ = data;
      }
      if (!link_alive) {
        ++consecutive_fail_;
        // 连续多轮全失败 -> 判定串口掉线, 进入自动重连
        if (consecutive_fail_ >= 3 && serial_online_.load()) {
          serial_.close();
          serial_online_.store(false);
          RCLCPP_WARN(
            this->get_logger(),
            "[reconnect] 检测到串口掉线(连续 %d 轮无有效响应), 进入自动重连", consecutive_fail_);
        }
      } else {
        consecutive_fail_ = 0;
      }
      publish_status(data, ok);

      // 固定节拍调度
      next_wake += std::chrono::milliseconds(sample_period_ms_);
      const auto now = std::chrono::steady_clock::now();
      if (next_wake > now) {
        std::this_thread::sleep_until(next_wake);
      } else {
        next_wake = now;
      }
    }
  }

  // ===== 成员 =====
  std::string serial_device_;
  int baud_rate_;
  uint8_t addr_;
  uint8_t host_;
  int sample_period_ms_;
  int resp_timeout_ms_;
  int reconnect_ms_;

  airship_link::SerialInterface serial_;
  std::atomic<bool> running_;
  std::atomic<bool> serial_online_{false};
  int consecutive_fail_ = 0;
  int reconnect_attempt_ = 0;
  std::thread poll_thread_;

  // 上一帧有效数据: 失败轮次沿用, 避免掉线时发布清零数值
  BackupBmsData last_good_data_;

  rclcpp::Publisher<airship_msgs::msg::BackupBmsStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BackupBmsNode>());
  rclcpp::shutdown();
  return 0;
}
