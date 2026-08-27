// 灵云01号伴飞电脑 — 飞控数据监控节点 (fc_monitor)
//
// 功能:
//   1. 订阅 MAVROS 各数据 topic, 聚合为统一的 FlightStatus 发布到 /fc/status
//   2. 订阅电机输出/空速/角速度等扩展数据
//   3. 多级异常检测 (WARNING/CRITICAL/EMERGENCY), 去抖+去重后发布 /fc/alert
//   4. 按周期将聚合状态写入 CSV 数据日志 (便于事后分析)
//
// MAVROS topic 映射 (MAVROS 2.14):
//   /mavros/state                    -> online/armed/flight_mode/landed_state
//   /mavros/extended_state           -> landed_state
//   /mavros/imu/data                 -> 姿态角 + 角速度 (四元数 -> 欧拉角)
//   /mavros/global_position/global   -> 经纬度/海拔
//   /mavros/global_position/local    -> 相对高度/速度 (ENU)
//   /mavros/actuator_control         -> 电机输出 (8 路)
//   /mavros/vfr_hud                  -> 空速(真空速)/地速/爬升率/油门/航向角
//   /mavros/battery                  -> 电压/电流/剩余电量
//
// 注: 安装版 mavros_msgs 无 Airspeed.msg, 真空速暂以 VfrHud.airspeed 填充;
//     升级 mavros_msgs 后可改订阅 /mavros/airspeed 分离指示空速(IAS)与真空速(TAS)。
//
// online 状态: 综合 MAVROS connected 标志与最近收到消息的心跳看门狗。
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/actuator_control.hpp>
#include <mavros_msgs/msg/esc_status.hpp>
#include <mavros_msgs/msg/esc_telemetry.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>

#include "airship_fc/fc_alert_logic.hpp"
#include "airship_msgs/msg/fc_alert.hpp"
#include "airship_msgs/msg/flight_status.hpp"
#include "airship_utils/math_utils.hpp"

using std::placeholders::_1;

// 异常等级常量
namespace
{
constexpr uint8_t kLevelInfo = airship_msgs::msg::FcAlert::LEVEL_INFO;
constexpr uint8_t kLevelWarning = airship_msgs::msg::FcAlert::LEVEL_WARNING;
constexpr uint8_t kLevelCritical = airship_msgs::msg::FcAlert::LEVEL_CRITICAL;
constexpr uint8_t kLevelEmergency = airship_msgs::msg::FcAlert::LEVEL_EMERGENCY;
// "高度过低"告警的起飞判定阈值: 历史最高 AGL 超过该值才视为已起飞, 启用低高度检测
constexpr float kTakeoffThresholdM = 10.0f;
}

// 单个参数的异常检测配置(纯阈值参数; 去抖/等级状态见 alert_states_)
struct AlertRule
{
  std::string name;            // 参数名
  float warning;               // 警告阈值 (超过即告警, 单位与 current 相同)
  float critical;              // 严重阈值
  float emergency;             // 危急阈值
  bool upper_side;             // true=超过阈值告警(>); false=低于阈值告警(<)
  // 转成纯逻辑库的 Threshold(fc_alert_logic.hpp)
  fc_logic::Threshold threshold() const
  {
    return {warning, critical, emergency, upper_side};
  }
};

class FcMonitorNode : public rclcpp::Node
{
public:
  explicit FcMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("fc_monitor_node", options)  // 与 launch/yaml 节点名一致(旧为 fc_monitor, 手动调试时 yaml 参数不生效)
  {
    pub_rate_hz_ = this->declare_parameter("pub_rate_hz", 10.0);
    fc_timeout_s_ = this->declare_parameter("fc_timeout_s", 2.0);
    // 除零防护: 频率必须为正, 否则 1000.0/rate 产生 inf 导致 static_cast<int> 未定义行为
    if (pub_rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "pub_rate_hz 非法值 %.3f, 重置为 10.0", pub_rate_hz_);
      pub_rate_hz_ = 10.0;
    }
    // 告警去抖: 连续 N 次采样超限才触发告警 (防止毛刺误报)
    debounce_n_ = this->declare_parameter("alert_debounce_n", 3);
    // 数据日志目录 (空字符串表示禁用 CSV 记录)
    log_dir_ = this->declare_parameter("log_dir", "");

    status_pub_ = this->create_publisher<airship_msgs::msg::FlightStatus>(
      "/fc/status", rclcpp::QoS(10));
    alert_pub_ = this->create_publisher<airship_msgs::msg::FcAlert>(
      "/fc/alert", rclcpp::QoS(10));

    // MAVROS 订阅 (best_effort 以保证高频 IMU 不丢包)
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_state, this, _1));
    ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_ext_state, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_imu, this, _1));
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_gps, this, _1));
    local_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/global_position/local", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_local, this, _1));
    actuator_sub_ = this->create_subscription<mavros_msgs::msg::ActuatorControl>(
      "/mavros/actuator_control", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_actuator, this, _1));
    vfr_sub_ = this->create_subscription<mavros_msgs::msg::VfrHud>(
      "/mavros/vfr_hud", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_vfr, this, _1));
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_battery, this, _1));
    // EKF 估计器状态 (GPS 位置锁定/退化/毛刺/加速度计错误)
    estimator_sub_ = this->create_subscription<mavros_msgs::msg::EstimatorStatus>(
      "/mavros/estimator_status/status", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_estimator, this, _1));
    // GPS 原始数据 (卫星数/定位精度 eph/epv)
    gps_raw_sub_ = this->create_subscription<mavros_msgs::msg::GPSRAW>(
      "/mavros/global_position/raw/gps", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_gps_raw, this, _1));
    // ESC 遥测 (转速/电流/电压/温度, DroneCAN/CAN 电调; PWM 供电时 esp_status 空)
    esc_status_sub_ = this->create_subscription<mavros_msgs::msg::ESCStatus>(
      "/mavros/esc_status", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_esc_status, this, _1));
    esc_telemetry_sub_ = this->create_subscription<mavros_msgs::msg::ESCTelemetry>(
      "/mavros/esc_telemetry", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_esc_telemetry, this, _1));

    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / pub_rate_hz_)),
      std::bind(&FcMonitorNode::publish_status, this));

    // 统一时间源, 避免默认构造的 rclcpp::Time(系统时间) 与 now()(ROS 时间) 相减报错
    last_msg_stamp_ = this->now();

    setup_alert_rules();
    init_logger();
  }

private:
  // 初始化异常检测规则 (阈值取自文档 07 安全阈值表)
  void setup_alert_rules()
  {
    rules_.push_back({"roll_deg", 20.0f, 30.0f, 45.0f, true});
    rules_.push_back({"pitch_deg", 10.0f, 15.0f, 20.0f, true});
    rules_.push_back({"yaw_rate", 10.0f, 20.0f, 30.0f, true});
    rules_.push_back({"climb_rate", 3.0f, 5.0f, 8.0f, true});
    rules_.push_back({"altitude_agl_high", 140.0f, 148.0f, 150.0f, true});
    rules_.push_back({"altitude_agl_low", 3.0f, 2.0f, 1.0f, false});
    rules_.push_back({"battery_voltage", 48.0f, 46.0f, 44.0f, false});
    rules_.push_back({"battery_remaining", 30.0f, 20.0f, 10.0f, false});
    // GPS 定位质量: fix_type < 3 (无 3D fix) 视为定位失效 (0=无GPS,1=NO_FIX,2=2D)
    rules_.push_back({"gps_fix_loss", 3.0f, 3.0f, 3.0f, false});
    // EKF 退化到位置锁定模式 (GPS 不可用) -> 定位精度骤降, 危急
    rules_.push_back({"ekf_const_pos_mode", 1.0f, 1.0f, 1.0f, false});
    // 卫星数过少 (低于 6 颗警告)-> 影响定位可靠性
    rules_.push_back({"gps_satellites_low", 6.0f, 5.0f, 4.0f, false});
    // 电机堵转: 任一电机输出>0.5 但转速≈0 (需 ESC 遥测数据, 否则跳过)
    rules_.push_back({"motor_stuck", 0.5f, 0.5f, 0.5f, false});
    // 去抖/告警状态与规则一一对应(状态与阈值参数分离存储)
    alert_states_.assign(rules_.size(), fc_logic::AlertState{});
  }

  // 初始化 CSV 日志
  // 按天切分的 CSV 文件名 (tm 可注入, 便于单元测试跨天切分逻辑)
  static std::string make_csv_path(const std::string & dir, const std::tm & lt)
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d",
      lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday);
    return dir + "/fc_status_" + buf + ".csv";
  }

  // 同上: 生成 YYYYMMDD 日期串
  static std::string date_key(const std::tm & lt)
  {
    char buf[9];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d",
      lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday);
    return buf;
  }

  static std::tm local_now()
  {
    const std::time_t t = std::time(nullptr);
    std::tm lt{};
    localtime_r(&t, &lt);
    return lt;
  }

  void init_logger()
  {
    if (log_dir_.empty()) {
      RCLCPP_INFO(this->get_logger(), "CSV 数据日志已禁用 (log_dir 为空)");
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);
    if (ec) {
      RCLCPP_WARN(this->get_logger(), "创建日志目录失败: %s, 禁用 CSV", ec.message().c_str());
      return;
    }
    open_daily_csv(local_now());
  }

  // 打开当日 CSV 文件(追加模式), 空文件写表头
  void open_daily_csv(const std::tm & lt)
  {
    const std::string path = make_csv_path(log_dir_, lt);
    csv_ofs_.open(path, std::ios::app);
    if (!csv_ofs_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CSV 失败: %s", path.c_str());
      return;
    }
    csv_open_date_ = date_key(lt);
    // 空文件时写表头
    if (csv_ofs_.tellp() == 0) {
      csv_ofs_
        << "stamp_sec,roll_deg,pitch_deg,yaw_deg,alt_agl,alt_amsl,vx,vy,vz,"
        << "airspeed,groundspeed,climb_rate,throttle,armed,flight_mode,"
        << "battery_v,battery_remaining,"
        << "m0,m1,m2,m3,m4,m5,m6,m7\n";
      csv_ofs_.flush();
    }
    RCLCPP_INFO(this->get_logger(), "CSV 数据日志已开启: %s", path.c_str());
  }

  void on_state(const mavros_msgs::msg::State::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mavros_connected_ = msg->connected;
    armed_ = msg->armed;
    flight_mode_ = msg->mode;
    has_valid_data_ = true;   // 收到任意 state 即视为飞控数据可用
    last_msg_stamp_ = this->now();
  }

  void on_ext_state(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    landed_state_ = msg->landed_state;
    last_msg_stamp_ = this->now();
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    airship_utils::quat_to_euler(
      msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z,
      &roll_rad_, &pitch_rad_, &yaw_rad_);
    roll_rate_ = msg->angular_velocity.x;
    pitch_rate_ = msg->angular_velocity.y;
    yaw_rate_ = msg->angular_velocity.z;
    last_msg_stamp_ = this->now();
  }

  void on_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lat_ = msg->latitude;
    lon_ = msg->longitude;
    alt_amsl_ = static_cast<float>(msg->altitude);
    last_msg_stamp_ = this->now();
  }

  void on_local(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    alt_rel_ = static_cast<float>(msg->pose.pose.position.z);
    // 记录历史最高 AGL (alt_agl = -alt_rel_): 用于"高度过低"告警的起飞判断,
    // 避免飞艇在地面(AGL≈0)时 altitude_agl_low 持续误报。
    const float alt_agl = -alt_rel_;
    if (alt_agl > max_alt_agl_) {
      max_alt_agl_ = alt_agl;
    }
    vx_ = static_cast<float>(msg->twist.twist.linear.x);
    vy_ = static_cast<float>(msg->twist.twist.linear.y);
    vz_ = static_cast<float>(msg->twist.twist.linear.z);
    last_msg_stamp_ = this->now();
  }

  void on_actuator(const mavros_msgs::msg::ActuatorControl::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < motor_outputs_.size() && i < msg->controls.size(); ++i) {
      motor_outputs_[i] = msg->controls[i];
    }
    last_msg_stamp_ = this->now();
  }

  void on_vfr(const mavros_msgs::msg::VfrHud::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // VfrHud.airspeed 即真实/真空速(MAVROS 2.14); 安装版无 Airspeed.msg, 指示空速/真空速同源填充
    airspeed_ = msg->airspeed;
    true_airspeed_ = msg->airspeed;
    groundspeed_ = msg->groundspeed;
    climb_rate_ = msg->climb;
    throttle_ = msg->throttle;
    heading_deg_ = msg->heading;
    last_msg_stamp_ = this->now();
  }

  void on_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // X25 EVO 未接电池/未配置电源监控时会上报电压 0 / 剩余 -1, 属无效数据。
    // 仅在数值有效时才更新并启用电池判据, 避免 EMERGENCY 误报。
    const bool valid = msg->voltage > 0.0f && msg->percentage >= 0.0f;
    if (valid) {
      battery_voltage_ = msg->voltage;
      battery_current_ = msg->current;
      // MAVROS /mavros/battery 为 sensor_msgs/BatteryState, 其 percentage 契约为
      // 0~1(MAVROS 已将 MAVLink 的 0~100 除以 100)。此处统一归一化为 0~100 百分数,
      // 供后续规则(阈值 30/20/10)直接比较, 也与本包 FlightStatus.battery_remaining
      // 契约(0~100)一致。条件归一化为防御性写法(兼容直接透传 0~100 的实现)。
      battery_remaining_ = fc_logic::normalize_battery_remaining(msg->percentage);
      battery_has_data_ = true;
    }
    last_msg_stamp_ = this->now();
  }

  void on_estimator(const mavros_msgs::msg::EstimatorStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ekf_pos_vert_agl_ = msg->pos_vert_agl_status_flag;
    ekf_const_pos_mode_ = msg->const_pos_mode_status_flag;
    ekf_gps_glitch_ = msg->gps_glitch_status_flag;
    ekf_accel_error_ = msg->accel_error_status_flag;
    last_msg_stamp_ = this->now();
  }

  void on_gps_raw(const mavros_msgs::msg::GPSRAW::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gps_fix_type_ = msg->fix_type;
    gps_satellites_ = msg->satellites_visible;
    gps_eph_ = msg->eph;
    gps_epv_ = msg->epv;
    last_msg_stamp_ = this->now();
  }

  void on_esc_status(const mavros_msgs::msg::ESCStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // ESC_STATUS 携带每路电调的 rpm/current/voltage
    const size_t n = std::min(msg->esc_status.size(), esc_rpm_.size());
    esc_count_ = static_cast<uint8_t>(n);
    for (size_t i = 0; i < n; ++i) {
      esc_rpm_[i] = static_cast<float>(msg->esc_status[i].rpm);
      esc_current_[i] = msg->esc_status[i].current;
      esc_voltage_[i] = msg->esc_status[i].voltage;
    }
    last_msg_stamp_ = this->now();
  }

  void on_esc_telemetry(const mavros_msgs::msg::ESCTelemetry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // ESC_TELEMETRY 携带每路电调温度 (转速/电流/电压也含, 与 esc_status 冗余)
    const size_t n = std::min(msg->esc_telemetry.size(), esc_temperature_.size());
    // 若仅收到 ESC_TELEMETRY 而无 ESC_STATUS(esc_count_ 仍为 0), 以温度通道数初始化
    // 计数, 保证消息内 esc_count 与遥测数组语义自洽(否则温度有值但 n=0 自相矛盾)
    if (esc_count_ == 0 && n > 0) {
      esc_count_ = static_cast<uint8_t>(n);
    }
    for (size_t i = 0; i < n; ++i) {
      esc_temperature_[i] = msg->esc_telemetry[i].temperature;
    }
    last_msg_stamp_ = this->now();
  }

  // 取当前值用于规则判断
  float rule_value(const std::string & name)
  {
    if (name == "roll_deg") {return roll_rad_ * 180.0f / static_cast<float>(M_PI);}
    if (name == "pitch_deg") {return pitch_rad_ * 180.0f / static_cast<float>(M_PI);}
    if (name == "yaw_rate") {return yaw_rate_ * 180.0f / static_cast<float>(M_PI);}
    if (name == "climb_rate") {return climb_rate_;}
    // 高度取反: alt_rel 来自 MAVROS /global_position/local (ENU, Z 向上时为正值)
    // 【现场待确认】PX4 实际 alt_rel 符号依飞控输出而定; 若已为向上正, 取负会反转方向。
    // 部署时须按实际飞控数据核对该符号, 见 docs/05_飞控监控与联调.md。
    if (name == "altitude_agl_high") {return -alt_rel_;}
    if (name == "altitude_agl_low") {return -alt_rel_;}
    if (name == "battery_voltage") {return battery_voltage_;}
    if (name == "battery_remaining") {return battery_remaining_;}
    // GPS fix < 3 -> 触发 (upper=较低值告警, 这里返回当前 fix 值)
    if (name == "gps_fix_loss") {return static_cast<float>(gps_fix_type_);}
    // EKF 位置锁定模式标志 (0/1)
    if (name == "ekf_const_pos_mode") {return ekf_const_pos_mode_ ? 1.0f : 0.0f;}
    // 卫星数过少
    if (name == "gps_satellites_low") {return static_cast<float>(gps_satellites_);}
    // 电机堵转: 任一电机输出>0.5 且对应转速<阈值时置 1
    if (name == "motor_stuck") {return check_motor_stuck() ? 1.0f : 0.0f;}
    return 0.0f;
  }

  // 电机堵转检测: 任一(有ESC遥测的)电机 输出>0.5 但转速≈0 (需 ESC 数据)
  bool check_motor_stuck() const
  {
    if (esc_count_ == 0 || !armed_) {
      return false;   // 无 ESC 遥测或未解锁, 不做堵转判断
    }
    const size_t n = std::min(
      static_cast<size_t>(esc_count_),
      std::min(motor_outputs_.size(), esc_rpm_.size()));
    for (size_t i = 0; i < n; ++i) {
      if (motor_outputs_[i] > 0.5f && esc_rpm_[i] < 50.0f) {
        return true;   // 输出高但几乎无转速 -> 疑似堵转
      }
    }
    return false;
  }

  // 异常检测: 遍历规则, 去抖后等级变化时发告警
  void run_anomaly_detection()
  {
    for (size_t i = 0; i < rules_.size(); ++i) {
      const AlertRule & r = rules_[i];
      // 电池数据尚未上报/无效时跳过电池判据, 避免初始值 0 触发 EMERGENCY 误报
      if (!battery_has_data_ &&
        (r.name == "battery_voltage" || r.name == "battery_remaining"))
      {
        continue;
      }
      // 高度过低告警: 仅在已起飞(历史最高 AGL 超阈值)后启用, 避免地面(AGL≈0)持续误报
      if (r.name == "altitude_agl_low" && max_alt_agl_ < kTakeoffThresholdM) {
        continue;
      }
      // GPS 定位相关: 飞控未连接 GPS 时 (fix_type=0) 不判定位失效, 避免无 GPS 期间持续误报
      if ((r.name == "gps_fix_loss" || r.name == "gps_satellites_low") &&
        gps_fix_type_ == 0)
      {
        continue;
      }
      // 电机堵转: 无 ESC 遥测数据(未接 CAN/DroneCAN)时不判堵转
      if (r.name == "motor_stuck" && esc_count_ == 0) {
        continue;
      }
      const float value = rule_value(r.name);
      // 去抖状态机(纯逻辑, 见 fc_alert_logic.hpp): 累积越界、等级化去重、恢复解除
      const auto d = fc_logic::update_alert(r.threshold(), value, alert_states_[i], debounce_n_);
      if (d.action == fc_logic::Action::kActivate) {
        publish_alert(r, d.level, d.value, true);
      } else if (d.action == fc_logic::Action::kClear) {
        publish_alert(r, kLevelInfo, d.value, false);
      }
    }
  }

  void publish_alert(
    const AlertRule & r, uint8_t level, float value, bool active)
  {
    auto alert = airship_msgs::msg::FcAlert();
    alert.header.stamp = this->now();
    alert.level = level;
    alert.param_name = r.name;
    alert.current_value = value;
    // 按实际触发等级填充阈值, 而非恒用 warning 阈值
    const float thresh = (level == kLevelEmergency) ? r.emergency :
      (level == kLevelCritical) ? r.critical : r.warning;
    alert.threshold = thresh;
    alert.active = active;
    if (active) {
      alert.description = "飞控参数超限: " + r.name + " = " + std::to_string(value);
      alert.suggestion = "请检查飞控状态";
    } else {
      alert.description = "飞控参数恢复正常: " + r.name;
    }
    alert_pub_->publish(alert);

    const char * level_str = (level == kLevelEmergency) ? "EMERGENCY" :
      (level == kLevelCritical) ? "CRITICAL" :
      (level == kLevelWarning) ? "WARNING" : "INFO";
    RCLCPP_WARN(this->get_logger(), "[FC][%s] %s = %.2f (threshold %.2f)",
      level_str, r.name.c_str(), value, r.warning);
  }

  void publish_status()
  {
    auto msg = airship_msgs::msg::FlightStatus();
    msg.header.stamp = this->now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // 心跳看门狗: MAVROS 断连或超时未收到数据则判离线
      const bool heartbeat_ok =
        (this->now() - last_msg_stamp_).seconds() < fc_timeout_s_;
      msg.online = mavros_connected_ && heartbeat_ok;
      msg.armed = armed_;
      msg.flight_mode = flight_mode_;
      msg.landed_state = landed_state_;
      msg.roll_deg = roll_rad_ * 180.0f / static_cast<float>(M_PI);
      msg.pitch_deg = pitch_rad_ * 180.0f / static_cast<float>(M_PI);
      msg.yaw_deg = yaw_rad_ * 180.0f / static_cast<float>(M_PI);
      msg.roll_rate = roll_rate_;
      msg.pitch_rate = pitch_rate_;
      msg.yaw_rate = yaw_rate_;
      msg.lat = lat_;
      msg.lon = lon_;
      msg.alt_amsl = alt_amsl_;
      msg.alt_rel = alt_rel_;
      msg.vx = vx_;
      msg.vy = vy_;
      msg.vz = vz_;
      msg.airspeed = airspeed_;
      msg.true_airspeed = true_airspeed_;
      msg.groundspeed = groundspeed_;
      msg.climb_rate = climb_rate_;
      msg.throttle = throttle_;
      msg.heading_deg = heading_deg_;
      msg.motor_outputs = motor_outputs_;
      // EKF / GPS / ESC 遥测
      msg.ekf_const_pos_mode = ekf_const_pos_mode_;
      msg.ekf_gps_glitch = ekf_gps_glitch_;
      msg.ekf_accel_error = ekf_accel_error_;
      msg.ekf_pos_vert_agl = ekf_pos_vert_agl_;
      msg.gps_fix_type = gps_fix_type_;
      msg.gps_satellites = gps_satellites_;
      msg.gps_eph = gps_eph_;
      msg.gps_epv = gps_epv_;
      msg.esc_count = esc_count_;
      msg.esc_rpm = esc_rpm_;
      msg.esc_voltage = esc_voltage_;
      msg.esc_current = esc_current_;
      msg.esc_temperature = esc_temperature_;
      msg.battery_voltage = battery_voltage_;
      msg.battery_current = battery_current_;
      msg.battery_remaining = battery_remaining_;

      // 仅在飞控已连接且收到过有效数据时进行异常检测, 避免启动初期/断连时初始值误报
      // (mavros 未连接时也会发布 state, 故以 mavros_connected_ 而非 has_valid_data_ 为准)
      // 在锁内执行, 保证对共享状态(mavros_connected_/battery_has_data_/各测量值)读取一致,
      // 兼容 MultiThreadedExecutor 下 timer 与订阅回调并发执行
      if (mavros_connected_ && has_valid_data_) {
        run_anomaly_detection();
      }
    }
    status_pub_->publish(msg);

    write_csv(msg);
  }

  // 写 CSV 日志
  void write_csv(const airship_msgs::msg::FlightStatus & msg)
  {
    if (!csv_ofs_.is_open()) {
      return;
    }
    // 跨天切分: 本地日期变化时关闭旧文件并打开新一天的文件
    // (否则单文件追加无限增长; 按天分片后由 airship-disk-guard 按 mtime 清理)
    const std::tm now_lt = local_now();
    if (date_key(now_lt) != csv_open_date_) {
      RCLCPP_INFO(this->get_logger(), "CSV 跨天切分: %s -> %s",
        csv_open_date_.c_str(), date_key(now_lt).c_str());
      csv_ofs_.close();
      open_daily_csv(now_lt);
      if (!csv_ofs_.is_open()) {
        return;
      }
    }
    // 磁盘满/只读导致的写入失败检测: ofstream 写入失败会置 failbit 但 is_open() 仍为 true,
    // 若不检测会每帧静默失败。检测到后关闭并告警一次(节流), 避免高频重复失败消耗资源。
    if (!csv_ofs_.good()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "CSV 日志写入失败(磁盘满或只读?), 已停止 CSV 记录(%s)", log_dir_.c_str());
      csv_ofs_.close();
      return;
    }
    char ts_buf[64];
    // 时间戳: 秒 + 9 位纳秒(补零), 保证 CSV 时间列宽一致便于解析
    // sec/nanosec 为 32 位整型, 用 int 转换避免引入 C 类型 long
    std::snprintf(ts_buf, sizeof(ts_buf), "%d.%09d",
      static_cast<int>(msg.header.stamp.sec),
      static_cast<int>(msg.header.stamp.nanosec));
    csv_ofs_
      << ts_buf << ","
      << msg.roll_deg << "," << msg.pitch_deg << "," << msg.yaw_deg << ","
      << -msg.alt_rel << "," << msg.alt_amsl << ","
      << msg.vx << "," << msg.vy << "," << msg.vz << ","
      << msg.airspeed << "," << msg.groundspeed << "," << msg.climb_rate << ","
      << msg.throttle << "," << msg.armed << "," << msg.flight_mode << ","
      << msg.battery_voltage << "," << msg.battery_remaining << ","
      << msg.motor_outputs[0] << "," << msg.motor_outputs[1] << ","
      << msg.motor_outputs[2] << "," << msg.motor_outputs[3] << ","
      << msg.motor_outputs[4] << "," << msg.motor_outputs[5] << ","
      << msg.motor_outputs[6] << "," << msg.motor_outputs[7] << "\n";
    // 节流 flush(约 1Hz), 避免 10Hz 每帧 flush 造成高频磁盘 IO
    if (++csv_write_count_ % 10 == 0) {
      csv_ofs_.flush();
    }
  }

  // ===== 成员 =====
  double pub_rate_hz_;
  double fc_timeout_s_;
  int debounce_n_;
  std::string log_dir_;
  int csv_write_count_ = 0;
  std::string csv_open_date_;   // 当前 CSV 文件对应的 YYYYMMDD (跨天切分判定)

  std::mutex mutex_;
  std::ofstream csv_ofs_;
  std::vector<AlertRule> rules_;
  std::vector<fc_logic::AlertState> alert_states_;  // 与 rules_ 一一对应(去抖/等级状态)
  // 姿态
  float roll_rad_ = 0.0f;
  float pitch_rad_ = 0.0f;
  float yaw_rad_ = 0.0f;
  // 角速度 (rad/s)
  float roll_rate_ = 0.0f;
  float pitch_rate_ = 0.0f;
  float yaw_rate_ = 0.0f;
  // 位置/速度
  double lat_ = 0.0;
  double lon_ = 0.0;
  float alt_amsl_ = 0.0f;
  float alt_rel_ = 0.0f;
  float max_alt_agl_ = 0.0f;   // 历史最高 AGL, 用于"高度过低"告警的起飞判断
  float vx_ = 0.0f;
  float vy_ = 0.0f;
  float vz_ = 0.0f;
  // 空速/真空速/地速/爬升/油门/航向 (安装版 mavros_msgs 无 Airspeed.msg, 指示/真空速同源 VfrHud)
  float airspeed_ = 0.0f;       // 空速/指示空速 [m/s] (VfrHud.airspeed)
  float true_airspeed_ = 0.0f;  // 真空速 TAS [m/s] (VfrHud.airspeed)
  float groundspeed_ = 0.0f;
  float climb_rate_ = 0.0f;
  float throttle_ = 0.0f;
  float heading_deg_ = 0.0f;    // 航向角 [deg] (/mavros/vfr_hud.heading)
  // 电机输出
  std::array<float, 8> motor_outputs_{0.0f};
  // EKF 估计器状态
  bool ekf_const_pos_mode_ = false;
  bool ekf_gps_glitch_ = false;
  bool ekf_accel_error_ = false;
  bool ekf_pos_vert_agl_ = false;
  // GPS 原始数据
  uint8_t gps_fix_type_ = 0;
  uint8_t gps_satellites_ = 0;
  uint16_t gps_eph_ = 0;
  uint16_t gps_epv_ = 0;
  // ESC 遥测 (DroneCAN/CAN 电调, PWM 供电时 esc_count=0)
  uint8_t esc_count_ = 0;
  std::array<float, 10> esc_rpm_{0.0f};
  std::array<float, 10> esc_voltage_{0.0f};
  std::array<float, 10> esc_current_{0.0f};
  std::array<float, 10> esc_temperature_{0.0f};
  // 状态/电池
  bool mavros_connected_ = false;
  bool armed_ = false;
  uint8_t landed_state_ = 0;
  std::string flight_mode_;
  float battery_voltage_ = 0.0f;
  float battery_current_ = 0.0f;
  float battery_remaining_ = 0.0f;
  // 是否已收到过电池数据 (防止未上报时 battery=0 触发 EMERGENCY 误报)
  bool battery_has_data_ = false;
  rclcpp::Time last_msg_stamp_;
  // 是否已收到过飞控有效数据 (防止启动初期初始值 0 触发误告警)
  bool has_valid_data_ = false;

  rclcpp::Publisher<airship_msgs::msg::FlightStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<airship_msgs::msg::FcAlert>::SharedPtr alert_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr actuator_sub_;
  rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr vfr_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps_raw_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ESCStatus>::SharedPtr esc_status_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ESCTelemetry>::SharedPtr esc_telemetry_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FcMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
