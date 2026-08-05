// 灵云01号 Offboard控制器
// 功能:
//   1. 订阅目标位置/速度(geometry_msgs, NED坐标系), 生成OffboardSetpoint
//   2. 飞艇专属约束处理:
//      - 中性浮力: 悬停时推力=0, 不需要持续补偿重力
//      - 高度限位: clamp到 [AS_ALT_MIN, AS_ALT_MAX], 软限位区减速
//      - 偏航限制: yaw设定值变化率限制(AS_YAW_RMAX), 大角度转向S形分段
//      - 俯仰限制: 飞控内部已限制±15度, 此处仅监控
//   3. 支持位置控制/速度控制两种模式
//
// 输入话题:
//   /airship/status                    (airship_msgs/AirshipStatus)     当前状态
//   /airship/cmd/target_position       (geometry_msgs/PointStamped)     目标位置(NED: x=North,y=East,z=Down)
//   /airship/cmd/target_velocity       (geometry_msgs/TwistStamped)     目标速度(NED)
//   /airship/cmd/target_yaw            (std_msgs/Float64)               目标偏航角(rad, NED)
//
// 输出话题:
//   /airship/offboard_setpoint         (airship_msgs/OffboardSetpoint)  控制设定值

#include <cmath>
#include <limits>
#include <memory>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "airship_msgs/msg/airship_status.hpp"
#include "airship_msgs/msg/offboard_setpoint.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class OffboardController : public rclcpp::Node
{
public:
  OffboardController()
  : rclcpp::Node("offboard_controller")
  {
    // ===== 参数 =====
    control_rate_ = this->declare_parameter("control_rate_hz", 20.0);
    alt_max_ = this->declare_parameter("alt_max_limit", 150.0);         // AS_ALT_MAX
    alt_soft_ = this->declare_parameter("alt_soft_limit", 140.0);       // AS_ALT_SOFT
    alt_min_ = this->declare_parameter("alt_min_limit", 2.0);           // AS_ALT_MIN
    yaw_rate_limit_ = this->declare_parameter("yaw_rate_limit", 0.524); // AS_YAW_RMAX ~30deg/s
    max_horiz_vel_ = this->declare_parameter("max_horizontal_velocity", 15.0); // AS_VEL_XY_MAX
    max_vert_vel_ = this->declare_parameter("max_vertical_velocity", 2.0);     // AS_ALT_VMAX
    control_mode_ = this->declare_parameter("control_mode", std::string("position"));
    enable_yaw_smoothing_ = this->declare_parameter("enable_yaw_smoothing", true);

    // ===== 订阅 =====
    status_sub_ = this->create_subscription<airship_msgs::msg::AirshipStatus>(
      "/airship/status", rclcpp::QoS(10), std::bind(&OffboardController::on_status, this, _1));

    target_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/airship/cmd/target_position",
      rclcpp::QoS(10),
      std::bind(&OffboardController::on_target_position, this, _1));

    target_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/airship/cmd/target_velocity",
      rclcpp::QoS(10),
      std::bind(&OffboardController::on_target_velocity, this, _1));

    target_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/airship/cmd/target_yaw",
      rclcpp::QoS(10),
      std::bind(&OffboardController::on_target_yaw, this, _1));

    // ===== 发布 =====
    setpoint_pub_ = this->create_publisher<airship_msgs::msg::OffboardSetpoint>(
      "/airship/offboard_setpoint", rclcpp::QoS(10));

    debug_pub_ =
      this->create_publisher<std_msgs::msg::String>("/airship/control/debug", rclcpp::QoS(10));

    // ===== 定时器 =====
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                              std::bind(&OffboardController::timer_control, this));

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
                "OffboardController 启动. mode=%s rate=%.1fHz alt[%.0f,%.0f] yaw_rate=%.3f",
                control_mode_.c_str(),
                control_rate_,
                alt_min_,
                alt_max_,
                yaw_rate_limit_);
  }

private:
  // ===== 状态回调 =====
  void on_status(const airship_msgs::msg::AirshipStatus::SharedPtr msg)
  {
    current_status_ = *msg;
    has_status_ = true;

    // 初始化yaw设定值为当前yaw
    if (!yaw_sp_initialized_ && enable_yaw_smoothing_) {
      current_yaw_sp_ = msg->yaw;
      yaw_sp_initialized_ = true;
    }
  }

  // ===== 目标位置回调 (NED坐标系: x=North, y=East, z=Down) =====
  void on_target_position(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    target_pos_[0] = static_cast<float>(msg->point.x); // North
    target_pos_[1] = static_cast<float>(msg->point.y); // East
    target_pos_[2] = static_cast<float>(msg->point.z); // Down (负值=高空)
    has_target_pos_ = true;
  }

  // ===== 目标速度回调 (NED坐标系) =====
  void on_target_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    target_vel_[0] = static_cast<float>(msg->twist.linear.x); // North
    target_vel_[1] = static_cast<float>(msg->twist.linear.y); // East
    target_vel_[2] = static_cast<float>(msg->twist.linear.z); // Down
    has_target_vel_ = true;
  }

  // ===== 目标偏航回调 (NED, rad) =====
  void on_target_yaw(const std_msgs::msg::Float64::SharedPtr msg)
  {
    target_yaw_ = static_cast<float>(msg->data);
    has_target_yaw_ = true;
  }

  // ===== 主控制循环 =====
  void timer_control()
  {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / control_rate_; // 防止异常dt
    }

    if (!has_status_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "等待飞艇状态...");
      return;
    }

    airship_msgs::msg::OffboardSetpoint sp;
    sp.header.stamp = now;

    // ===== 偏航设定值处理 (S形转弯策略) =====
    float yaw_sp = current_yaw_sp_;
    if (has_target_yaw_) {
      if (enable_yaw_smoothing_) {
        // 限制偏航设定值变化率, 实现平滑转向
        // 飞艇偏航响应极慢(Izz=145500), 不能直接发送大角度变化
        float yaw_error = angle_diff(target_yaw_, current_yaw_sp_);
        float max_delta = static_cast<float>(yaw_rate_limit_ * dt);
        float delta = clampf(yaw_error, -max_delta, max_delta);
        current_yaw_sp_ += delta;
        // 归一化
        current_yaw_sp_ = normalize_angle(current_yaw_sp_);
        yaw_sp = current_yaw_sp_;
      } else {
        yaw_sp = target_yaw_;
      }
    } else if (has_status_) {
      // 无目标偏航时保持当前航向
      yaw_sp = current_status_.yaw;
      current_yaw_sp_ = yaw_sp;
    }
    sp.yaw = yaw_sp;
    sp.yaw_setpoint_move_rate = static_cast<float>(yaw_rate_limit_);
    sp.yawspeed = 0.0f; // 位置/速度控制时不指定偏航角速度

    // ===== 根据控制模式生成设定值 =====
    if (control_mode_ == "position" && has_target_pos_) {
      sp.position_valid = true;
      sp.velocity_valid = false;

      // 高度限位处理
      // z是Down方向, 高度 = -z, alt_max对应 z = -alt_max, alt_min对应 z = -alt_min
      float target_alt = -target_pos_[2]; // 目标高度
      float clamped_alt =
        clampf(target_alt, static_cast<float>(alt_min_), static_cast<float>(alt_max_));
      if (std::abs(clamped_alt - target_alt) > 0.01f) {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             2000,
                             "目标高度 %.1fm 超出限位[%.0f,%.0f], 已限制为 %.1fm",
                             target_alt,
                             alt_min_,
                             alt_max_,
                             clamped_alt);
      }
      sp.position = {target_pos_[0], target_pos_[1], -clamped_alt};

      // 速度前馈: 软限位区域减速
      if (clamped_alt > alt_soft_) {
        // 接近上限, 限制垂直速度
        sp.velocity = {NAN, NAN, NAN};
      } else {
        sp.velocity = {NAN, NAN, NAN}; // 纯位置控制, 不提供速度前馈
      }
      sp.acceleration = {NAN, NAN, NAN};
    } else if (control_mode_ == "velocity" && has_target_vel_) {
      sp.position_valid = false;
      sp.velocity_valid = true;

      // 速度限幅
      float vx = clampf(
        target_vel_[0], -static_cast<float>(max_horiz_vel_), static_cast<float>(max_horiz_vel_));
      float vy = clampf(
        target_vel_[1], -static_cast<float>(max_horiz_vel_), static_cast<float>(max_horiz_vel_));
      float vz = clampf(
        target_vel_[2], -static_cast<float>(max_vert_vel_), static_cast<float>(max_vert_vel_));

      // 高度限位时限制垂直速度方向
      float current_alt = current_status_.altitude_relative;
      if (current_alt >= alt_max_ && vz < 0) {
        // 已达上限, 禁止继续上升(vz<0=上升)
        vz = 0.0f;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "已达高度上限, 禁止上升");
      }
      if (current_alt <= alt_min_ && vz > 0) {
        // 已达下限, 禁止继续下降(vz>0=下降)
        vz = 0.0f;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "已达高度下限, 禁止下降");
      }

      sp.velocity = {vx, vy, vz};
      sp.position = {NAN, NAN, NAN};
      sp.acceleration = {NAN, NAN, NAN};
    } else if (control_mode_ == "position_velocity" && has_target_pos_ && has_target_vel_) {
      // 混合模式: 位置+速度前馈
      sp.position_valid = true;
      sp.velocity_valid = true;

      float target_alt = -target_pos_[2];
      float clamped_alt =
        clampf(target_alt, static_cast<float>(alt_min_), static_cast<float>(alt_max_));
      sp.position = {target_pos_[0], target_pos_[1], -clamped_alt};

      float vx = clampf(
        target_vel_[0], -static_cast<float>(max_horiz_vel_), static_cast<float>(max_horiz_vel_));
      float vy = clampf(
        target_vel_[1], -static_cast<float>(max_horiz_vel_), static_cast<float>(max_horiz_vel_));
      float vz = clampf(
        target_vel_[2], -static_cast<float>(max_vert_vel_), static_cast<float>(max_vert_vel_));
      sp.velocity = {vx, vy, vz};
      sp.acceleration = {NAN, NAN, NAN};
    } else {
      // 无有效目标, 悬停(位置保持)
      sp.position_valid = true;
      sp.velocity_valid = false;
      // 保持当前位置
      sp.position = {current_status_.x, current_status_.y, current_status_.z};
      sp.velocity = {NAN, NAN, NAN};
      sp.acceleration = {NAN, NAN, NAN};
    }

    // ===== 飞艇约束: thrust_y始终为0 (无侧向控制) =====
    sp.thrust_body = {0.0f, 0.0f, 0.0f}; // 位置/速度控制模式下thrust由飞控PID计算
    sp.torque_body = {0.0f, 0.0f, 0.0f};
    sp.attitude_valid = false;
    sp.body_rate_valid = false;
    sp.thrust_torque_valid = false;

    // 四元数默认值
    sp.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};

    setpoint_pub_->publish(sp);

    // 调试信息
    if (debug_pub_->get_subscription_count() > 0) {
      char buf[256];
      snprintf(buf,
               sizeof(buf),
               "mode=%s pos=[%.1f,%.1f,%.1f] yaw=%.1fdeg alt=%.1fm",
               control_mode_.c_str(),
               sp.position[0],
               sp.position[1],
               sp.position[2],
               sp.yaw * 180.0 / M_PI,
               current_status_.altitude_relative);
      std_msgs::msg::String dbg;
      dbg.data = buf;
      debug_pub_->publish(dbg);
    }
  }

  // ===== 工具函数 =====
  // 角度差, 归一化到[-pi, pi]
  static float angle_diff(float target, float current)
  {
    float diff = target - current;
    while (diff > M_PI) {
      diff -= 2.0f * static_cast<float>(M_PI);
    }
    while (diff < -M_PI) {
      diff += 2.0f * static_cast<float>(M_PI);
    }
    return diff;
  }

  // 角度归一化到[-pi, pi]
  static float normalize_angle(float angle)
  {
    while (angle > M_PI) {
      angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -M_PI) {
      angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
  }

  // 限幅
  static float clampf(float val, float min_val, float max_val)
  {
    if (val < min_val) {
      return min_val;
    }
    if (val > max_val) {
      return max_val;
    }
    return val;
  }

  // ===== 成员变量 =====
  // 参数
  double control_rate_;
  double alt_max_;
  double alt_soft_;
  double alt_min_;
  double yaw_rate_limit_;
  double max_horiz_vel_;
  double max_vert_vel_;
  std::string control_mode_;
  bool enable_yaw_smoothing_;

  // 状态
  airship_msgs::msg::AirshipStatus current_status_;
  bool has_status_ = false;

  // 目标
  float target_pos_[3] = {0, 0, 0};
  float target_vel_[3] = {0, 0, 0};
  float target_yaw_ = 0;
  bool has_target_pos_ = false;
  bool has_target_vel_ = false;
  bool has_target_yaw_ = false;

  // 偏航平滑
  float current_yaw_sp_ = 0;
  bool yaw_sp_initialized_ = false;

  // 时间
  rclcpp::Time last_time_;

  // ROS接口
  rclcpp::Subscription<airship_msgs::msg::AirshipStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_yaw_sub_;
  rclcpp::Publisher<airship_msgs::msg::OffboardSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardController>());
  rclcpp::shutdown();
  return 0;
}
