// 灵云01号伴飞电脑 — 飞控数据监控节点 (fc_monitor)
//
// 通过 MAVROS 订阅飞控数据, 聚合为统一的 FlightStatus 发布到 /fc/status。
// 依赖运行中的 MAVROS (fcu_url 配置见 launch)。
//
// MAVROS topic 映射:
//   /mavros/state                       -> online/armed/flight_mode
//   /mavros/imu/data                    -> 姿态角 (四元数 -> 欧拉角)
//   /mavros/global_position/global      -> 经纬度/海拔
//   /mavros/global_position/local       -> 相对高度/速度 (ENU)
//   /mavros/battery                     -> 电压/电流/剩余电量
//
// online 状态: 综合 MAVROS connected 标志与最近收到消息的心跳看门狗。
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>

#include "airship_msgs/msg/flight_status.hpp"
#include "airship_utils/math_utils.hpp"

using std::placeholders::_1;

class FcMonitorNode : public rclcpp::Node
{
public:
  explicit FcMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("fc_monitor", options)
  {
    pub_rate_hz_ = this->declare_parameter("pub_rate_hz", 10.0);
    fc_timeout_s_ = this->declare_parameter("fc_timeout_s", 2.0);

    status_pub_ = this->create_publisher<airship_msgs::msg::FlightStatus>(
      "/fc/status", rclcpp::QoS(10));

    // MAVROS 订阅 (best_effort 以保证高频 IMU 不丢包)
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_state, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_imu, this, _1));
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_gps, this, _1));
    local_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/global_position/local", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_local, this, _1));
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", rclcpp::QoS(10).best_effort(),
      std::bind(&FcMonitorNode::on_battery, this, _1));

    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / pub_rate_hz_)),
      std::bind(&FcMonitorNode::publish_status, this));

    // 统一时间源, 避免默认构造的 rclcpp::Time(系统时间) 与 now()(ROS 时间) 相减报错
    last_msg_stamp_ = this->now();
  }

private:
  void on_state(const mavros_msgs::msg::State::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mavros_connected_ = msg->connected;
    armed_ = msg->armed;
    flight_mode_ = msg->mode;
    last_msg_stamp_ = this->now();
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    airship_utils::quat_to_euler(
      msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z,
      &roll_rad_, &pitch_rad_, &yaw_rad_);
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

  void on_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    battery_voltage_ = msg->voltage;
    battery_current_ = msg->current;
    battery_remaining_ = msg->percentage;
    last_msg_stamp_ = this->now();
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
      msg.roll_deg = roll_rad_ * 180.0f / static_cast<float>(M_PI);
      msg.pitch_deg = pitch_rad_ * 180.0f / static_cast<float>(M_PI);
      msg.yaw_deg = yaw_rad_ * 180.0f / static_cast<float>(M_PI);
      msg.lat = lat_;
      msg.lon = lon_;
      msg.alt_amsl = alt_amsl_;
      msg.alt_rel = alt_rel_;
      msg.vx = vx_;
      msg.vy = vy_;
      msg.vz = vz_;
      msg.battery_voltage = battery_voltage_;
      msg.battery_current = battery_current_;
      msg.battery_remaining = battery_remaining_;
    }
    status_pub_->publish(msg);
  }

  // ===== 成员 =====
  double pub_rate_hz_;
  double fc_timeout_s_;

  std::mutex mutex_;
  // 姿态
  float roll_rad_ = 0.0f;
  float pitch_rad_ = 0.0f;
  float yaw_rad_ = 0.0f;
  // 位置/速度
  double lat_ = 0.0;
  double lon_ = 0.0;
  float alt_amsl_ = 0.0f;
  float alt_rel_ = 0.0f;
  float vx_ = 0.0f;
  float vy_ = 0.0f;
  float vz_ = 0.0f;
  // 状态/电池
  bool mavros_connected_ = false;
  bool armed_ = false;
  std::string flight_mode_;
  float battery_voltage_ = 0.0f;
  float battery_current_ = 0.0f;
  float battery_remaining_ = 0.0f;
  rclcpp::Time last_msg_stamp_;

  rclcpp::Publisher<airship_msgs::msg::FlightStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub_;
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
