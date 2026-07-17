// 灵云01号 伴飞电脑 安全监控节点
// 功能:
//   1. 订阅 /airship/status, 监控飞艇专属安全约束(高度/偏航/俯仰/通信/电池)
//   2. 发布 /airship/safety_status (10Hz)
//   3. 检测到危险时发布 /airship/safety_override (ModeCommand) 自动切换安全模式
// 物理约束:
//   - 中性浮力: 断电后缓慢飘移而非坠落, 悬停推力=0
//   - 无Roll控制: Roll轴依赖气动自稳定, 本节点不处理Roll
//   - 大惯量(Izz=145500): 偏航响应极慢, 偏航5度时气动力矩接近控制极限
//   - 高度硬限位: AS_ALT_MAX=150m, AS_ALT_MIN=2m, AS_ALT_SOFT=140m

#include <rclcpp/rclcpp.hpp>
#include "airship_msgs/msg/airship_status.hpp"
#include "airship_msgs/msg/safety_status.hpp"
#include "airship_msgs/msg/mode_command.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class AirshipSafetyMonitor : public rclcpp::Node
{
public:
  AirshipSafetyMonitor()
  : rclcpp::Node("airship_safety_monitor")
  {
    // ===== 参数声明 (飞艇物理约束阈值) =====
    alt_max_hard_ = this->declare_parameter("alt_max_hard", 150.0);       // 高度硬限位 AS_ALT_MAX
    alt_max_soft_ = this->declare_parameter("alt_max_soft", 140.0);       // 高度软限位 AS_ALT_SOFT
    alt_min_ = this->declare_parameter("alt_min", 2.0);                   // 高度下限 AS_ALT_MIN
    yaw_rate_limit_ = this->declare_parameter("yaw_rate_limit", 0.524);   // AS_YAW_RMAX ~30deg/s
    pitch_limit_ = this->declare_parameter("pitch_limit", 0.2618);        // 最大俯仰角 ±15度
    battery_low_voltage_ = this->declare_parameter("battery_low_voltage", 22.0);
    battery_critical_voltage_ = this->declare_parameter("battery_critical_voltage", 20.0);
    link_timeout_ = this->declare_parameter("link_timeout", 3.0);         // 通信丢失阈值
    publish_rate_hz_ = this->declare_parameter("publish_rate_hz", 10.0);  // 状态发布频率

    // ===== 订阅飞艇综合状态 =====
    status_sub_ = this->create_subscription<airship_msgs::msg::AirshipStatus>(
      "/airship/status", rclcpp::QoS(10),
      std::bind(&AirshipSafetyMonitor::on_status, this, _1));

    // ===== 发布安全状态与覆盖命令 =====
    safety_status_pub_ = this->create_publisher<airship_msgs::msg::SafetyStatus>(
      "/airship/safety_status", rclcpp::QoS(10));
    safety_override_pub_ = this->create_publisher<airship_msgs::msg::ModeCommand>(
      "/airship/safety_override", rclcpp::QoS(10));

    // ===== 定时器: 周期检查并发布 =====
    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AirshipSafetyMonitor::timer_callback, this));

    // 初始化: 节点启动时尚未收到任何状态
    last_status_time_ = this->now();
    has_status_ = false;

    RCLCPP_INFO(this->get_logger(),
      "AirshipSafetyMonitor 启动. rate=%.1fHz alt[min=%.1f soft=%.1f hard=%.1f] "
      "yaw_limit=%.3f pitch_limit=%.3f bat<%.1f/%.1f linkTO=%.1fs",
      publish_rate_hz_, alt_min_, alt_max_soft_, alt_max_hard_,
      yaw_rate_limit_, pitch_limit_,
      battery_low_voltage_, battery_critical_voltage_, link_timeout_);
  }

private:
  // ===== 状态回调: 缓存最新遥测 =====
  void on_status(const airship_msgs::msg::AirshipStatus::SharedPtr msg)
  {
    latest_status_ = *msg;
    last_status_time_ = this->now();
    has_status_ = true;
  }

  // ===== 定时回调: 安全检查 + 发布 =====
  void timer_callback()
  {
    airship_msgs::msg::SafetyStatus safety;
    const rclcpp::Time now = this->now();
    safety.header.stamp = now;

    // ===== 通信安全 =====
    double link_age = (now - last_status_time_).seconds();
    safety.link_age = static_cast<float>(link_age);
    safety.link_lost = (link_age > link_timeout_);

    // ===== 高度安全 =====
    const float altitude = latest_status_.altitude_relative;
    safety.altitude = altitude;
    safety.alt_max = static_cast<float>(alt_max_hard_);
    safety.alt_min = static_cast<float>(alt_min_);
    // 距离最近硬限位的余量(正值=安全, 负值=已超限)
    safety.alt_margin = std::min(
      altitude - static_cast<float>(alt_min_),
      static_cast<float>(alt_max_hard_) - altitude);
    // 软限位检查: 超出软限位即视为违反
    safety.altitude_violation =
      (altitude > static_cast<float>(alt_max_soft_)) ||
      (altitude < static_cast<float>(alt_min_));

    // ===== 偏航安全 =====
    const float yaw_rate = latest_status_.yawspeed;
    safety.yaw_rate = yaw_rate;
    safety.yaw_rate_limit = static_cast<float>(yaw_rate_limit_);
    safety.yaw_rate_exceeded = (std::fabs(yaw_rate) > yaw_rate_limit_);

    // ===== 俯仰安全 =====
    const float pitch = latest_status_.pitch;
    safety.pitch = pitch;
    safety.pitch_limit = static_cast<float>(pitch_limit_);
    safety.pitch_exceeded = (std::fabs(pitch) > pitch_limit_);

    // ===== 电池安全 =====
    const float battery_voltage = latest_status_.battery_voltage;
    safety.battery_voltage = battery_voltage;
    safety.battery_low = (battery_voltage < static_cast<float>(battery_low_voltage_));

    // ===== 综合判断 safe_to_control =====
    // 任一关键违反发生时禁止Offboard控制
    safety.safe_to_control =
      !(safety.altitude_violation || safety.link_lost || safety.battery_low);

    // ===== 计算安全等级 0=正常 1=警告 2=危险 3=紧急 =====
    uint8_t level = 0;

    // 通信安全: 超时>10s 为紧急
    if (safety.link_lost) {
      level = std::max(level, static_cast<uint8_t>(1));
      if (link_age > kLinkCriticalTimeout) {
        level = 3;
      }
    }

    // 高度安全: 超出硬限位 为危险
    const bool altitude_hard_violation =
      (altitude > static_cast<float>(alt_max_hard_)) ||
      (altitude < static_cast<float>(alt_min_));
    if (safety.altitude_violation) {
      level = std::max(level, static_cast<uint8_t>(1));
      if (altitude_hard_violation) {
        level = std::max(level, static_cast<uint8_t>(2));
      }
    }

    // 电池安全: 低于临界电压 为危险
    if (safety.battery_low) {
      level = std::max(level, static_cast<uint8_t>(1));
      if (battery_voltage < static_cast<float>(battery_critical_voltage_)) {
        level = std::max(level, static_cast<uint8_t>(2));
      }
    }

    // 偏航安全: 超限 为警告
    if (safety.yaw_rate_exceeded) {
      level = std::max(level, static_cast<uint8_t>(1));
    }

    // 俯仰安全: 超限 为警告
    if (safety.pitch_exceeded) {
      level = std::max(level, static_cast<uint8_t>(1));
    }

    safety.safety_level = level;

    // ===== 警告信息 =====
    safety.warning_message = build_warning_message(safety, altitude_hard_violation);

    // ===== 自动保护: 发布 ModeCommand =====
    publish_safety_override(safety, altitude_hard_violation, now);

    // ===== 发布安全状态 =====
    safety_status_pub_->publish(safety);

    last_safety_level_ = level;
  }

  // 构建可读警告信息(为空表示无警告)
  std::string build_warning_message(const airship_msgs::msg::SafetyStatus & s, bool alt_hard) const
  {
    if (s.safety_level == 0) {
      return "";
    }
    std::string msg;
    if (s.link_lost) {
      msg += "链路丢失(" + std::to_string(s.link_age) + "s); ";
    }
    if (s.altitude_violation) {
      msg += "高度超限(" + std::to_string(s.altitude) + "m)";
      if (alt_hard) {
        msg += "[硬限位]";
      }
      msg += "; ";
    }
    if (s.yaw_rate_exceeded) {
      msg += "偏航角速度超限(" + std::to_string(s.yaw_rate) + "rad/s); ";
    }
    if (s.pitch_exceeded) {
      msg += "俯仰超限(" + std::to_string(s.pitch) + "rad); ";
    }
    if (s.battery_low) {
      msg += "电池低(" + std::to_string(s.battery_voltage) + "V); ";
    }
    return msg;
  }

  // 自动保护: 当危险/紧急且处于Offboard时发布模式切换或降落命令
  void publish_safety_override(const airship_msgs::msg::SafetyStatus & s,
                                bool altitude_hard_violation,
                                const rclcpp::Time & now)
  {
    // 等级低于2: 不干预, 清除覆盖状态
    if (s.safety_level < 2) {
      override_active_ = false;
      last_override_time_ = rclcpp::Time();
      return;
    }

    // 非Offboard模式: 操作员已接管, 不干预
    if (!latest_status_.offboard_enabled) {
      return;
    }

    // 频率限制: 最多 1/kOverrideInterval Hz, 避免刷屏但保证可靠
    if (override_active_ && (now - last_override_time_).seconds() < kOverrideInterval) {
      return;
    }

    airship_msgs::msg::ModeCommand cmd;
    cmd.header.stamp = now;

    if (s.safety_level == 3) {
      // 紧急: 发布降落命令 (中性浮力飞艇降落为缓慢下降)
      cmd.land = true;
      cmd.mode = airship_msgs::msg::ModeCommand::MODE_LAND;
      RCLCPP_ERROR(this->get_logger(),
        "紧急(level=3) 发布降落命令: %s", s.warning_message.c_str());
    } else {
      // 危险(level=2): 切换出Offboard到手动辅助模式
      cmd.set_mode = true;
      if (altitude_hard_violation) {
        // 高度硬限位超限 → Altitude模式(优先恢复高度控制)
        cmd.mode = airship_msgs::msg::ModeCommand::MODE_ALTITUDE;
      } else {
        // 其他危险(电池超低等) → Position模式
        cmd.mode = airship_msgs::msg::ModeCommand::MODE_POSITION;
      }
      RCLCPP_WARN(this->get_logger(),
        "危险(level=2) 切换到%s模式: %s",
        altitude_hard_violation ? "Altitude" : "Position",
        s.warning_message.c_str());
    }

    safety_override_pub_->publish(cmd);
    override_active_ = true;
    last_override_time_ = now;
  }

  // ===== 参数成员 =====
  double alt_max_hard_;
  double alt_max_soft_;
  double alt_min_;
  double yaw_rate_limit_;
  double pitch_limit_;
  double battery_low_voltage_;
  double battery_critical_voltage_;
  double link_timeout_;
  double publish_rate_hz_;

  // 常量
  static constexpr double kLinkCriticalTimeout = 10.0;  // 通信丢失超过10s 为紧急
  static constexpr double kOverrideInterval = 1.0;      // 覆盖命令最小间隔(s)

  // 状态
  airship_msgs::msg::AirshipStatus latest_status_;
  rclcpp::Time last_status_time_;
  bool has_status_ = false;
  uint8_t last_safety_level_ = 0;
  bool override_active_ = false;
  rclcpp::Time last_override_time_;

  // ROS 接口
  rclcpp::Subscription<airship_msgs::msg::AirshipStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<airship_msgs::msg::SafetyStatus>::SharedPtr safety_status_pub_;
  rclcpp::Publisher<airship_msgs::msg::ModeCommand>::SharedPtr safety_override_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AirshipSafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
