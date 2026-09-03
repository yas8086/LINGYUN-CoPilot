// 灵云01号伴飞电脑 — LoRa 温度/压力集中器采集节点 (lora_node)
//
// 通过 USB-RS485 串口以 Modbus RTU 轮询集中器, 采集多个 LoRa 节点的温度/压力。
// 协议解析见 airship_lora::modbus_rtu (纯库, 可 gtest); 串口复用 airship_link::SerialInterface。
//
// 接口:
//   发布   /lora/samples      (LoRaSamples)  每轮采集结果
//   发布   /lora/summary      (LoRaSummary)  温度统计汇总
//   服务   /lora/query_node   (QueryNode)    按节点 ID 查询最新采样
//   服务   /lora/query_summary(QuerySummary) 主动触发一次汇总
//
// 参数:
//   serial_device / baud_rate / slave_addr
//   node_ids            (int[])     LoRa 节点 ID 列表
//   node_types          (string[])  每个节点类型: "temperature"/"pressure" (与 node_ids 一一对应)
//   sample_period_ms / temp_reg_addr / pressure_reg_addr / alarm_low / alarm_high
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include "airship_link/serial_interface.hpp"
#include "airship_lora/modbus_rtu.hpp"
#include "airship_msgs/msg/lo_ra_sample.hpp"
#include "airship_msgs/msg/lo_ra_samples.hpp"
#include "airship_msgs/msg/lo_ra_summary.hpp"
#include "airship_msgs/srv/query_node.hpp"
#include "airship_msgs/srv/query_summary.hpp"

using airship_lora::LoraSampleData;
using airship_lora::build_read_request;
using airship_lora::check_temp_alarm;
using airship_lora::parse_pressure_response;
using airship_lora::parse_temp_response;

using std::placeholders::_1;
using std::placeholders::_2;

class LoraNode : public rclcpp::Node
{
public:
  explicit LoraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("lora_node", options),
    serial_("/dev/ttyUSB0"),
    running_(false)
  {
    // ===== 参数 =====
    serial_device_ = this->declare_parameter("serial_device", std::string("/dev/ttyUSB0"));
    baud_rate_ = this->declare_parameter("baud_rate", 9600);
    slave_addr_ = static_cast<uint8_t>(this->declare_parameter("slave_addr", 1));
    node_ids_ = this->declare_parameter("node_ids", std::vector<int64_t>{1, 2});
    node_types_ = this->declare_parameter(
      "node_types",
      std::vector<std::string>{"temperature", "temperature"});
    sample_period_ms_.store(this->declare_parameter("sample_period_ms", 2000));
    temp_reg_addr_ = static_cast<uint16_t>(this->declare_parameter("temp_reg_addr", 0x76C1));
    pressure_reg_addr_ =
      static_cast<uint16_t>(this->declare_parameter("pressure_reg_addr", 0x8EF9));
    alarm_low_ = this->declare_parameter("alarm_low", -10.0);
    alarm_high_ = this->declare_parameter("alarm_high", 60.0);
    resp_timeout_ms_ = this->declare_parameter("resp_timeout_ms", 200);
    // 单寄存器读失败就地重试次数: RS485 半双工应答偶发截断(实测日志 已收0B/6B),
    // 重试即可恢复, 避免把瞬态错误放大成"节点离线"
    read_retries_ = this->declare_parameter("read_retries", 1);
    // 离线去抖轮数: 连续失败达到该轮数才发布 online=0(下传列表才会移除该节点);
    // 其间沿用上一帧有效值, 使地面站看到的节点名单稳定不闪断 (2026-08-27)
    offline_debounce_rounds_ = this->declare_parameter("offline_debounce_rounds", 3);
    // 串口自动重连周期 (ms): 检测到串口掉线后按此周期尝试重连
    reconnect_ms_ = this->declare_parameter("reconnect_ms", 2000);

    // 动态参数: 运行中可修改 sample_period_ms (地面站可实时改频)
    param_cb_ = this->add_pre_set_parameters_callback(
      std::bind(&LoraNode::on_set_parameters, this, std::placeholders::_1));

    // 校验节点 ID 与类型数组长度一致
    if (node_ids_.size() != node_types_.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "node_ids(%zu) 与 node_types(%zu) 长度不一致, 多余项将被忽略",
        node_ids_.size(), node_types_.size());
    }
    // 采集周期预算校验: 周期小于全节点最坏耗时(含重试/超时)时轮次将超期,
    // 提前告警(不阻断启动, 调度有 next_wake=now 兜底) (2026-09-03)
    {
      const size_t n = std::min(node_ids_.size(), node_types_.size());
      const double worst_ms = static_cast<double>(n) *
        (read_retries_ + 1.0) * resp_timeout_ms_ * 2.0;
      if (static_cast<double>(sample_period_ms_.load()) < worst_ms) {
        RCLCPP_WARN(
          this->get_logger(),
          "sample_period_ms=%d 小于全节点最坏耗时预算 %.0fms(节点%zu, 含重试/超时),"
          " 采集轮次可能超期", sample_period_ms_.load(), worst_ms, n);
      }
    }

    // ===== 发布器 =====
    samples_pub_ = this->create_publisher<airship_msgs::msg::LoRaSamples>(
      "/lora/samples", rclcpp::QoS(10));
    summary_pub_ = this->create_publisher<airship_msgs::msg::LoRaSummary>(
      "/lora/summary", rclcpp::QoS(10));

    // ===== 服务 =====
    query_node_srv_ = this->create_service<airship_msgs::srv::QueryNode>(
      "/lora/query_node", std::bind(&LoraNode::on_query_node, this, _1, _2));
    query_summary_srv_ = this->create_service<airship_msgs::srv::QuerySummary>(
      "/lora/query_summary", std::bind(&LoraNode::on_query_summary, this, _1, _2));

    // ===== 打开 485 串口 =====
    serial_ = airship_link::SerialInterface(serial_device_);
    serial_online_.store(serial_.open(static_cast<airship_link::BaudRate>(baud_rate_)));
    if (!serial_online_.load()) {
      RCLCPP_WARN(this->get_logger(), "打开串口 %s 失败, 进入自动重连", serial_device_.c_str());
    }

    // ===== 采集线程 =====
    running_ = true;
    collect_thread_ = std::thread(&LoraNode::collection_loop, this);
  }

  ~LoraNode() override
  {
    running_ = false;
    if (collect_thread_.joinable()) {
      collect_thread_.join();
    }
    serial_.close();
  }

private:
  // 判断节点是否为压力传感器 (按 node_types 配置)
  bool is_pressure_type(size_t idx) const
  {
    if (idx >= node_types_.size()) {
      return false;
    }
    return node_types_[idx] == "pressure";
  }

  // 确保串口已打开; 未打开则尝试重连。返回串口当前是否可用。
  bool ensure_serial()
  {
    if (serial_online_.load()) {
      return true;
    }
    // 记录重连尝试耗时与状态变化
    const auto t0 = std::chrono::steady_clock::now();
    serial_.close();
    const bool ok = serial_.open(static_cast<airship_link::BaudRate>(baud_rate_));
    const double cost_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    const int attempt = ++reconnect_attempt_;

    if (ok) {
      serial_online_.store(true);
      consecutive_fail_ = 0;
      reconnect_attempt_ = 0;
      // 记录重连成功时刻, 用于区分"重连后又全超时"是否疑似节点集体离线(见掉线判定)
      last_reconnect_ok_time_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(
        this->get_logger(),
        "[reconnect] 第 %d 次重连成功, 状态: OFFLINE->ONLINE, 耗时 %.1f ms, 设备 %s",
        attempt, cost_ms, serial_device_.c_str());
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "[reconnect] 第 %d 次重连失败, 状态: 仍为 OFFLINE, 耗时 %.1f ms, 设备 %s",
      attempt, cost_ms, serial_device_.c_str());
    return false;
  }

  // 读取一个寄存器组(功能码 0x04)。quantity=1 温度(7字节响应), quantity=2 压力(9字节响应)。
  bool read_registers(uint16_t start_addr, uint16_t quantity, std::vector<uint8_t> & resp)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const auto req = build_read_request(slave_addr_, start_addr, quantity);
    serial_.flush_rx();
    if (!serial_.write(reinterpret_cast<const char *>(req.data()), req.size())) {
      io_write_error_ = true;  // 底层写失败: 串口物理链路已断的强信号
      RCLCPP_WARN(this->get_logger(), "[diag] TX 写失败, addr=0x%04X qty=%u", start_addr, quantity);
      return false;
    }

    const size_t resp_len = (quantity == 1) ? 7u : 9u;
    resp.clear();
    resp.reserve(resp_len);
    char buf[32];
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(resp_timeout_ms_);
    while (resp.size() < resp_len) {
      const auto now = std::chrono::steady_clock::now();
      const int remaining = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
      if (remaining <= 0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[diag] RX 超时, addr=0x%04X qty=%u 期望 %zuB 实收 %zuB",
          start_addr, quantity, resp_len, resp.size());
        return false;
      }
      const int n = serial_.read(buf, resp_len - resp.size(), remaining);
      if (n <= 0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[diag] RX 读错误, addr=0x%04X qty=%u 已收 %zuB", start_addr, quantity, resp.size());
        return false;
      }
      resp.insert(resp.end(), buf, buf + n);
    }
    const double cost_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_DEBUG(
      this->get_logger(),
      "[diag] RX 成功 addr=0x%04X qty=%u got=%zuB 耗时 %.1fms",
      start_addr, quantity, resp.size(), cost_ms);
    return true;
  }

  // 轮询一轮所有节点
  std::vector<LoraSampleData> poll_once()
  {
    std::vector<LoraSampleData> out;
    io_write_error_ = false;  // 每轮复位硬 I/O 错误标记
    const size_t n = std::min(node_ids_.size(), node_types_.size());

    for (size_t i = 0; i < n; ++i) {
      const int node_id = static_cast<int>(node_ids_[i]);
      const bool is_pressure = is_pressure_type(i);

      LoraSampleData s;
      s.node_id = node_id;
      s.is_pressure = is_pressure;
      s.online = 0;

      // 节点地址边界校验: node_id 过小/过大时寄存器偏移可能溢出 uint16
      // 计算用 uint32 避免截断, 校验最终寄存器地址不溢出
      const uint32_t taddr32 = static_cast<uint32_t>(temp_reg_addr_) +
        static_cast<uint32_t>(std::max(0, node_id - 1));
      const uint32_t paddr32 = static_cast<uint32_t>(pressure_reg_addr_) +
        static_cast<uint32_t>(std::max(0, node_id - 1)) * 2;
      const bool addr_valid = (node_id >= 1) && (taddr32 <= 0xFFFF) &&
        (!is_pressure || (paddr32 + 1 <= 0xFFFF));
      if (!addr_valid) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "节点 %d 地址越界(temp=0x%04X pressure=0x%04X), 跳过本轮",
          node_id, static_cast<unsigned>(taddr32), static_cast<unsigned>(paddr32));
        out.push_back(s);
        continue;
      }

      // 所有节点都读温度, 保留压力节点的温度值
      // 失败就地重试 read_retries_ 次: 单次应答截断/超时不应立即判离线
      const uint16_t taddr = static_cast<uint16_t>(taddr32);
      std::vector<uint8_t> resp;
      float temp = 0.0f;
      int raw = 0;
      bool temp_ok = false;
      for (int attempt = 0; attempt <= read_retries_ && !temp_ok; ++attempt) {
        temp_ok = read_registers(taddr, 1, resp) &&
          parse_temp_response(resp, slave_addr_, temp, raw);
        if (!temp_ok && attempt < read_retries_) {
          continue;  // 下一轮尝试
        }
      }
      if (temp_ok) {
        s.online = 1;
        s.temp_valid = true;
        s.raw = raw;
        s.temp_celsius = temp;
        s.alarm = check_temp_alarm(temp, alarm_low_, alarm_high_);
      }

      // 压力节点额外读压力
      if (is_pressure) {
        const uint16_t paddr = static_cast<uint16_t>(paddr32);
        double pa = 0.0;
        bool press_ok = false;
        for (int attempt = 0; attempt <= read_retries_ && !press_ok; ++attempt) {
          press_ok = read_registers(paddr, 2, resp) &&
            parse_pressure_response(resp, slave_addr_, pa);
          if (!press_ok && attempt < read_retries_) {
            continue;
          }
        }
        if (press_ok) {
          s.online = 1;
          s.press_valid = true;
          s.pressure_pa = pa;
        }
      }

      out.push_back(s);
    }
    return out;
  }

  // 离线去抖: 连续失败 < offline_debounce_rounds_ 轮时沿用上一帧有效值并视为在线,
  // 只有持续失败才放行 online=0。避免单个寄存器读瞬态失败使地面站节点名单闪烁
  // (2026-08-27 实测: 2 分钟内节点集合切换 53 次, 根因即单轮偶发失败)。
  // 沿用值置 stale=1(非本轮实测, 地面站可灰显); 硬 I/O 错误轮(io_write_error_)跳过
  // 沿用——串口写失败是断链强信号, 需让本轮 online_count=0 以触发快速重连通道
  // (2026-09-03 修复: 去抖曾掩盖该信号, 令 consecutive_fail_ 复位、重连被推迟)。
  void apply_debounce(std::vector<LoraSampleData> & samples)
  {
    const bool hard_fail = io_write_error_;
    for (auto & s : samples) {
      auto & d = deb_[s.node_id];
      if (s.online != 0) {
        d.last = s;
        d.consec_fail = 0;
        d.has = true;
        continue;
      }
      ++d.consec_fail;
      if (!d.has) {
        continue;  // 从未收到过有效数据(启动初期), 保持零值
      }
      // 失败轮次: 数值沿用最后已知有效值(而非归 0, 防 0 值污染下传/统计),
      // stale=1 标记非本轮实测; 去抖窗口内仍报在线, 持续失败或硬 I/O 错误才离线。
      s = d.last;
      s.stale = true;
      s.online = (!hard_fail && d.consec_fail < offline_debounce_rounds_) ? 1 : 0;
    }
  }

  // 采集线程主循环
  //  - 固定节拍调度: 以绝对唤醒时刻为准, 避免周期漂移
  //  - 串口掉线检测: 连续多轮全节点失败则判定串口掉线, 自动重连
  void collection_loop()
  {
    const auto period_clock = std::chrono::steady_clock::now();
    auto next_wake = period_clock;
    while (running_.load()) {
      const auto round_t0_ = std::chrono::steady_clock::now();
      // 串口掉线时优先重连
      if (!serial_online_.load() && !ensure_serial()) {
        publish_summary();  // 上报串口离线状态
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
        next_wake = std::chrono::steady_clock::now();
        continue;
      }

      auto samples = poll_once();
      apply_debounce(samples);
      const double round_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - round_t0_).count();
      // 统计本轮在线节点数
      int online_count = 0;
      for (const auto & s : samples) {
        if (s.online != 0) {++online_count;}
      }
      RCLCPP_DEBUG(
        this->get_logger(),
        "[diag] 一轮采集完成: 在线 %d/%zu, 耗时 %.1fms (周期 %dms)",
        online_count, samples.size(), round_ms, sample_period_ms_.load());
      if (online_count == 0) {
        ++consecutive_fail_;
        // 串口掉线判定: 硬 I/O 错误(写失败)是断链强信号, 快速重连;
        // 仅"所有节点读超时"时可能只是节点离线, 延迟更久再重连, 避免误伤自愈。
        const int reconnect_thr = io_write_error_ ? 3 : 20;
        if (consecutive_fail_ >= reconnect_thr && serial_online_.load()) {
          serial_.close();
          serial_online_.store(false);
          const auto now = std::chrono::steady_clock::now();
          // 区分两类"全超时": 无底层写错误信号, 且距上次重连成功很短, 说明串口已通
          // 但所有节点都不应答——更可能是节点侧集体离线(掉电/远离基站)而非串口故障。
          // (Modbus 主从架构无法彻底区分, 此日志帮助现场定位方向)
          if (!io_write_error_ &&
            now - last_reconnect_ok_time_ < std::chrono::seconds(10))
          {
            RCLCPP_WARN(
              this->get_logger(),
              "[reconnect] 重连成功后短时间内再次全节点无应答(无底层写错误), "
              "疑似所有 LoRa 节点集体离线(节点掉电/远离)而非串口故障, 需现场排查");
          }
          RCLCPP_WARN(
            this->get_logger(),
            "[reconnect] 检测到串口掉线(连续 %d 轮无有效响应, io_error=%d), 进入自动重连",
            consecutive_fail_, static_cast<int>(io_write_error_));
        }
      } else {
        consecutive_fail_ = 0;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & s : samples) {
          cache_[s.node_id] = s;
        }
      }
      publish_samples(samples);
      publish_summary();

      // 固定节拍: 计算下一次唤醒绝对时刻
      next_wake += std::chrono::milliseconds(sample_period_ms_.load());
      const auto now = std::chrono::steady_clock::now();
      if (next_wake > now) {
        std::this_thread::sleep_until(next_wake);
      } else {
        next_wake = now;  // 某轮耗时超周期, 立即进入下一轮, 不额外等待
      }
    }
  }

  // 动态参数回调: 更新采集周期
  // 其余参数(节点列表/寄存器地址/重试等)为构造期只读——静默接受会令
  // "设置成功"但永不生效误导运维, 故显式拒绝 (2026-09-03)
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      if (p.get_name() == "sample_period_ms") {
        const int v = p.as_int();
        if (v < 100) {  // 防止频率过高导致 485 无法及时响应
          result.successful = false;
          result.reason = "sample_period_ms 过小(需 >= 100)";
          return result;
        }
        // 预算校验: 全节点最坏耗时 ≈ N节点 × (每次读×(retries+1)次) × resp_timeout × 2(温/压),
        // 周期小于该值时轮次会超期(仅告警不拒绝, 调度兜底 next_wake=now)
        const size_t n = std::min(node_ids_.size(), node_types_.size());
        const double worst_ms = static_cast<double>(n) *
          (read_retries_ + 1.0) * resp_timeout_ms_ * 2.0;
        if (static_cast<double>(v) < worst_ms) {
          RCLCPP_WARN(
            this->get_logger(),
            "sample_period_ms=%d 小于全节点最坏耗时预算 %.0fms(节点%zu, "
            "含重试/超时), 采集轮次可能超期", v, worst_ms, n);
        }
        sample_period_ms_.store(v);
        RCLCPP_INFO(
          this->get_logger(), "采集周期已更新为 %d ms", sample_period_ms_.load());
      } else {
        result.successful = false;
        result.reason = "参数 '" + p.get_name() + "' 为构造期只读, 需重启节点生效";
        return result;
      }
    }
    return result;
  }

  // 填充单个 ROS 采样消息
  airship_msgs::msg::LoRaSample to_ros_sample(const LoraSampleData & s) const
  {
    airship_msgs::msg::LoRaSample m;
    m.node_id = s.node_id;
    m.is_pressure = s.is_pressure;
    m.timestamp = this->now();
    m.temp_celsius = s.temp_celsius;
    m.pressure_pa = s.pressure_pa;
    m.raw = s.raw;
    m.online = s.online;
    m.alarm = s.alarm;
    m.stale = s.stale ? 1 : 0;
    m.temp_valid = s.temp_valid ? 1 : 0;
    m.press_valid = s.press_valid ? 1 : 0;
    return m;
  }

  void publish_samples(const std::vector<LoraSampleData> & samples)
  {
    airship_msgs::msg::LoRaSamples msg;
    msg.timestamp = this->now();
    for (const auto & s : samples) {
      msg.samples.push_back(to_ros_sample(s));
    }
    samples_pub_->publish(msg);
  }

  // 汇总(基于缓存): 平均/最高/最低温度, 在线/报警统计
  airship_msgs::msg::LoRaSummary build_summary()
  {
    airship_msgs::msg::LoRaSummary msg;
    msg.timestamp = this->now();
    std::lock_guard<std::mutex> lock(mutex_);

    msg.serial_online = serial_online_.load();
    msg.node_count = static_cast<int>(cache_.size());
    int online_count = 0;
    int alarm_count = 0;
    int valid_temp = 0;
    float sum = 0.0f;
    float max_temp = -1.0e30f;
    float min_temp = 1.0e30f;
    for (const auto & kv : cache_) {
      const LoraSampleData & s = kv.second;
      // 串口掉线时节点一律判为离线, 使 online_count 归零(而非停留在串口断开前的旧值)
      const bool online = msg.serial_online && (s.online != 0);
      if (online) {
        ++online_count;
      }
      // 温度统计只计入"值本身有效"的样本: 温度寄存器读成功(含去抖沿用窗口, 沿用值
      // 的 temp_valid 来自其测量时刻)。防部分成功轮(温度失败+压力成功)的 0 值把
      // min_temp 拉到 0 / avg 失真 (2026-09-03)
      if (online && s.temp_valid) {
        sum += s.temp_celsius;
        ++valid_temp;
        if (s.temp_celsius > max_temp) {max_temp = s.temp_celsius;}
        if (s.temp_celsius < min_temp) {min_temp = s.temp_celsius;}
      }
      // 报警统计与 online 判定一致化: 串口掉线期间冻结的旧报警不应继续计数
      if (online && s.alarm != 0) {
        ++alarm_count;
        msg.alarm_node_ids.push_back(s.node_id);
      }
    }
    msg.online_count = online_count;
    msg.alarm_count = alarm_count;
    msg.avg_temp = (valid_temp > 0) ? sum / static_cast<float>(valid_temp) : 0.0f;
    msg.max_temp = (valid_temp > 0) ? max_temp : 0.0f;
    msg.min_temp = (valid_temp > 0) ? min_temp : 0.0f;
    return msg;
  }

  void publish_summary()
  {
    summary_pub_->publish(build_summary());
  }

  // 服务: 按节点 ID 查询
  void on_query_node(
    const std::shared_ptr<airship_msgs::srv::QueryNode::Request> req,
    std::shared_ptr<airship_msgs::srv::QueryNode::Response> resp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = cache_.find(req->node_id);
    if (it == cache_.end()) {
      resp->success = false;
      resp->message = "节点 " + std::to_string(req->node_id) + " 不在缓存中";
      return;
    }
    resp->success = true;
    resp->sample = to_ros_sample(it->second);
  }

  // 服务: 触发一次汇总
  void on_query_summary(
    const std::shared_ptr<airship_msgs::srv::QuerySummary::Request>,
    std::shared_ptr<airship_msgs::srv::QuerySummary::Response> resp)
  {
    resp->success = true;
    resp->summary = build_summary();
  }

  // ===== 成员 =====
  std::string serial_device_;
  int baud_rate_;
  uint8_t slave_addr_;
  std::vector<int64_t> node_ids_;
  std::vector<std::string> node_types_;
  std::atomic<int> sample_period_ms_;
  uint16_t temp_reg_addr_;
  uint16_t pressure_reg_addr_;
  double alarm_low_;
  double alarm_high_;
  int resp_timeout_ms_;
  int read_retries_;
  int offline_debounce_rounds_;
  int reconnect_ms_;

  airship_link::SerialInterface serial_;
  std::atomic<bool> running_;
  std::atomic<bool> serial_online_{false};  // 485 串口当前是否在线
  std::chrono::steady_clock::time_point last_reconnect_ok_time_{};  // 最近一次重连成功时刻(区分节点集体离线)
  int consecutive_fail_ = 0;                 // 连续全失败轮数(用于串口掉线判定)

  // 每节点离线去抖状态: 最近一次有效采样 + 连续失败计数
  struct NodeDeb
  {
    LoraSampleData last{};
    int consec_fail = 0;
    bool has = false;
  };
  std::map<int, NodeDeb> deb_;
  bool io_write_error_ = false;              // 本轮是否出现底层串口写失败(硬 I/O 信号)
  int reconnect_attempt_ = 0;                // 当前掉线周期内已重连次数(成功后清零)
  std::thread collect_thread_;

  std::mutex mutex_;
  std::map<int, LoraSampleData> cache_;

  rclcpp::Publisher<airship_msgs::msg::LoRaSamples>::SharedPtr samples_pub_;
  rclcpp::Publisher<airship_msgs::msg::LoRaSummary>::SharedPtr summary_pub_;
  rclcpp::Service<airship_msgs::srv::QueryNode>::SharedPtr query_node_srv_;
  rclcpp::Service<airship_msgs::srv::QuerySummary>::SharedPtr query_summary_srv_;
  PreSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoraNode>());
  rclcpp::shutdown();
  return 0;
}
