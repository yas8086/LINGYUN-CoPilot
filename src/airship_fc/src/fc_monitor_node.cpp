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
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/actuator_control.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>

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
}

// 单个参数的异常检测配置 (阈值分级 + 告警去抖)
struct AlertRule
{
  std::string name;            // 参数名
  float warning;               // 警告阈值 (超过即告警, 单位与 current 相同)
  float critical;              // 严重阈值
  float emergency;             // 危急阈值
  bool upper_side;             // true=超过阈值告警(>); false=低于阈值告警(<)
  uint8_t current_level = 0;   // 当前活跃等级 (内部)
  int debounce_count = 0;      // 去抖计数 (内部)
};

class FcMonitorNode : public rclcpp::Node
{
public:
  explicit FcMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("fc_monitor", options)
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
  }

  // 初始化 CSV 日志
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
    const std::string path = log_dir_ + "/fc_status.csv";
    csv_ofs_.open(path, std::ios::app);
    if (!csv_ofs_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "打开 CSV 失败: %s", path.c_str());
      return;
    }
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
    battery_voltage_ = msg->voltage;
    battery_current_ = msg->current;
    battery_remaining_ = msg->percentage;
    battery_has_data_ = true;
    last_msg_stamp_ = this->now();
  }

  // 计算单个规则当前是否越界及对应等级 (0=正常)
  uint8_t rule_level(const AlertRule & r, float value) const
  {
    if (r.upper_side) {
      if (value > r.emergency) {return kLevelEmergency;}
      if (value > r.critical) {return kLevelCritical;}
      if (value > r.warning) {return kLevelWarning;}
    } else {
      if (value < r.emergency) {return kLevelEmergency;}
      if (value < r.critical) {return kLevelCritical;}
      if (value < r.warning) {return kLevelWarning;}
    }
    return 0;
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
    if (name == "battery_remaining") {return battery_remaining_ * 100.0f;}
    return 0.0f;
  }

  // 异常检测: 遍历规则, 去抖后等级变化时发告警
  void run_anomaly_detection()
  {
    for (auto & r : rules_) {
      // 电池数据尚未上报时跳过电池判据, 避免初始值 0 触发 EMERGENCY 误报
      if (!battery_has_data_ &&
        (r.name == "battery_voltage" || r.name == "battery_remaining"))
      {
        continue;
      }
      const float value = rule_value(r.name);
      const uint8_t level = rule_level(r, value);

      if (level > 0) {
        // 越界: 去抖计数
        r.debounce_count++;
        if (r.debounce_count >= debounce_n_ && level != r.current_level) {
          publish_alert(r, level, value, true);
          r.current_level = level;
        }
      } else {
        // 恢复正常: 若之前处于告警态, 发解除告警
        r.debounce_count = 0;
        if (r.current_level > 0) {
          publish_alert(r, kLevelInfo, value, false);
          r.current_level = 0;
        }
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

  std::mutex mutex_;
  std::ofstream csv_ofs_;
  std::vector<AlertRule> rules_;
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
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FcMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
