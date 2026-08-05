// 灵云01号 伴飞电脑 MAVROS桥接节点
// 功能:
//   1. 订阅MAVROS遥测话题, 聚合为 AirshipStatus 发布(NED坐标系, 与飞控一致)
//   2. 订阅 OffboardSetpoint, 转换为 MAVROS 设定值话题(ENU)发布
//   3. 订阅 ModeCommand, 调用 MAVROS 服务(解锁/模式切换/起飞/降落)
// 注意:
//   - MAVROS使用ENU坐标系, PX4内部使用NED, 本节点做 ENU<->NED 转换
//   - 飞艇悬停时 landed=true 属正常(中性浮力, 推力=0)

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "airship_msgs/msg/airship_status.hpp"
#include "airship_msgs/msg/mode_command.hpp"
#include "airship_msgs/msg/motor_status.hpp"
#include "airship_msgs/msg/offboard_setpoint.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// MAVLink PX4 custom_mode 与飞艇模式字符串的映射
// 飞艇自定义模式: 0=Manual 1=Stable 2=Altitude 3=Position 4=Offboard 5=Takeoff 6=Land 7=Failsafe 8=Task
static const std::unordered_map<std::string, uint8_t> kModeStrToId = {
  {"MANUAL", 0},
  {"STABILIZED", 1}, // 飞艇Stable
  {"ALTCTL", 2},     // Altitude
  {"POSCTL", 3},     // Position
  {"OFFBOARD", 4},
  {"AUTO.TAKEOFF", 5},
  {"AUTO.LAND", 6},
  {"AUTO.RTL", 6},     // 返航也映射为Land
  {"AUTO.MISSION", 8}, // Task
};

static const std::unordered_map<uint8_t, std::string> kModeIdToName = {
  {0, "Manual"},
  {1, "Stable"},
  {2, "Altitude"},
  {3, "Position"},
  {4, "Offboard"},
  {5, "Takeoff"},
  {6, "Land"},
  {7, "Failsafe"},
  {8, "Task"},
};

class AirshipBridge : public rclcpp::Node
{
public:
  AirshipBridge()
  : rclcpp::Node("airship_bridge")
  {
    // ===== 参数 =====
    status_rate_ = this->declare_parameter("status_rate_hz", 20.0);
    setpoint_rate_ = this->declare_parameter("setpoint_rate_hz", 20.0);
    alt_max_ = this->declare_parameter("alt_max_limit", 150.0); // AS_ALT_MAX
    alt_min_ = this->declare_parameter("alt_min_limit", 2.0);   // AS_ALT_MIN
    fcu_url_ = this->declare_parameter("fcu_url", std::string(""));

    // ===== 订阅 MAVROS 遥测话题 =====
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state",
      rclcpp::QoS(10).best_effort(),
      std::bind(&AirshipBridge::on_state, this, _1));

    ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state",
      rclcpp::QoS(10).best_effort(),
      std::bind(&AirshipBridge::on_ext_state, this, _1));

    local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose",
      rclcpp::QoS(10).best_effort(),
      std::bind(&AirshipBridge::on_local_pose, this, _1));

    local_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local",
      rclcpp::QoS(10).best_effort(),
      std::bind(&AirshipBridge::on_local_vel, this, _1));

    imu_sub_ =
      this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data",
                                                       rclcpp::QoS(50).best_effort(),
                                                       std::bind(&AirshipBridge::on_imu, this, _1));

    global_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global",
      rclcpp::QoS(10).best_effort(),
      std::bind(&AirshipBridge::on_global_pos, this, _1));

    rel_alt_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/rel_alt",
      rclcpp::QoS(10).best_effort(),
      std::bind(&AirshipBridge::on_rel_alt, this, _1));

    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery",
      rclcpp::QoS(10).best_effort(),
      std::bind(&AirshipBridge::on_battery, this, _1));

    // ===== 订阅伴飞电脑命令 =====
    setpoint_sub_ = this->create_subscription<airship_msgs::msg::OffboardSetpoint>(
      "/airship/offboard_setpoint",
      rclcpp::QoS(10),
      std::bind(&AirshipBridge::on_setpoint, this, _1));

    mode_cmd_sub_ = this->create_subscription<airship_msgs::msg::ModeCommand>(
      "/airship/mode_command",
      rclcpp::QoS(10),
      std::bind(&AirshipBridge::on_mode_command, this, _1));

    // ===== 发布 =====
    status_pub_ =
      this->create_publisher<airship_msgs::msg::AirshipStatus>("/airship/status", rclcpp::QoS(10));

    // ===== 发布到 MAVROS 的设定值话题 =====
    sp_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", rclcpp::QoS(10));

    sp_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", rclcpp::QoS(10));

    sp_attitude_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_attitude/attitude", rclcpp::QoS(10));

    // ===== 服务客户端 =====
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    // ===== 定时器 =====
    auto status_period = std::chrono::duration<double>(1.0 / status_rate_);
    status_timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
                              std::bind(&AirshipBridge::timer_status, this));

    auto sp_period = std::chrono::duration<double>(1.0 / setpoint_rate_);
    setpoint_timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(sp_period),
                              std::bind(&AirshipBridge::timer_setpoint, this));

    // 初始化状态
    status_.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    status_.alt_max_limit = alt_max_;
    status_.alt_min_limit = alt_min_;
    last_data_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
                "AirshipBridge 启动完成. status=%.1fHz sp=%.1fHz alt[%.1f,%.1f]",
                status_rate_,
                setpoint_rate_,
                alt_min_,
                alt_max_);
  }

private:
  // ===== MAVROS 状态回调 =====
  void on_state(const mavros_msgs::msg::State::SharedPtr msg)
  {
    status_.connected = msg->connected;
    status_.armed = msg->armed;
    status_.system_id = msg->sysid;

    // 模式映射
    auto it = kModeStrToId.find(msg->mode);
    if (it != kModeStrToId.end()) {
      status_.nav_state = it->second;
    } else {
      status_.nav_state = 0;
    }
    auto name_it = kModeIdToName.find(status_.nav_state);
    status_.mode_name = (name_it != kModeIdToName.end()) ? name_it->second : "Unknown";
    status_.offboard_enabled = (status_.nav_state == 4);

    if (msg->connected) {
      last_data_time_ = this->now();
    }
  }

  void on_ext_state(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
  {
    // 飞艇悬停时 landed_state 可能报告 ON_GROUND, 这是正确行为
    status_.landed = (msg->landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND);
  }

  void on_local_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // MAVROS发布ENU: x=East, y=North, z=Up
    // 转NED: x=North=y, y=East=x, z=Down=-z
    status_.x = static_cast<float>(msg->pose.position.y);  // North
    status_.y = static_cast<float>(msg->pose.position.x);  // East
    status_.z = static_cast<float>(-msg->pose.position.z); // Down
    last_data_time_ = this->now();
  }

  void on_local_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    // ENU -> NED
    status_.vx = static_cast<float>(msg->twist.linear.y);  // North
    status_.vy = static_cast<float>(msg->twist.linear.x);  // East
    status_.vz = static_cast<float>(-msg->twist.linear.z); // Down
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 姿态四元数 (MAVROS发布ENU, 需转NED)
    // ENU->NED 旋转: q_ned = q_rot * q_enu, 其中 q_rot = (0, sqrt(2)/2, 0, sqrt(2)/2) (绕Y 90度)
    // 简化: 直接从ENU四元数提取欧拉角后转换
    tf2::Quaternion q_enu(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    // 转NED: 旋转180°绕Z(Down)后, 再旋转-90°绕新Y(East)
    // 等价于 q_ned = q_z(180) * q_y(-90) * q_enu ? 实际上使用tf2变换更清晰
    // 这里采用MAVROS内部一致的做法: 直接输出ENU四元数, 但欧拉角转NED
    tf2::Matrix3x3 m(q_enu);
    double roll, pitch, yaw_enu;
    m.getRPY(roll, pitch, yaw_enu);

    // ENU->NED: roll_ned = roll_enu, pitch_ned = pitch_enu, yaw_ned = (pi/2 - yaw_enu)
    status_.roll = static_cast<float>(roll);
    status_.pitch = static_cast<float>(pitch);
    double yaw_ned = M_PI_2 - yaw_enu;
    // 归一化到 [-pi, pi]
    while (yaw_ned > M_PI) {
      yaw_ned -= 2.0 * M_PI;
    }
    while (yaw_ned < -M_PI) {
      yaw_ned += 2.0 * M_PI;
    }
    status_.yaw = static_cast<float>(yaw_ned);
    status_.heading = status_.yaw;

    // 四元数存储NED
    tf2::Quaternion q_ned;
    q_ned.setRPY(status_.roll, status_.pitch, status_.yaw);
    status_.quaternion = {static_cast<float>(q_ned.w()),
      static_cast<float>(q_ned.x()),
      static_cast<float>(q_ned.y()),
      static_cast<float>(q_ned.z())};

    // 角速度 ENU->NED
    // body rates: p_ned = p_enu, q_ned = q_enu, r_ned = -r_enu (近似, 简单镜像)
    status_.rollspeed = static_cast<float>(msg->angular_velocity.x);
    status_.pitchspeed = static_cast<float>(msg->angular_velocity.y);
    status_.yawspeed = static_cast<float>(-msg->angular_velocity.z);
    last_data_time_ = this->now();
  }

  void on_global_pos(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    status_.latitude = msg->latitude;
    status_.longitude = msg->longitude;
    status_.altitude_amsl = static_cast<float>(msg->altitude);
  }

  void on_rel_alt(const std_msgs::msg::Float64::SharedPtr msg)
  {
    status_.altitude_relative = static_cast<float>(msg->data);
    status_.altitude_agl = status_.altitude_relative; // 伴飞电脑无独立测距仪时近似
    // 高度限位检查
    status_.alt_limit_exceeded =
      (status_.altitude_relative > alt_max_) || (status_.altitude_relative < alt_min_);
  }

  void on_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    status_.battery_voltage = static_cast<float>(msg->voltage);
    status_.battery_current = static_cast<float>(msg->current);
    status_.battery_remaining = static_cast<float>(msg->percentage * 100.0f);
  }

  // ===== OffboardSetpoint 回调 (缓存最新设定值) =====
  void on_setpoint(const airship_msgs::msg::OffboardSetpoint::SharedPtr msg)
  {
    latest_setpoint_ = *msg;
    has_setpoint_ = true;
  }

  // ===== ModeCommand 回调 =====
  void on_mode_command(const airship_msgs::msg::ModeCommand::SharedPtr msg)
  {
    if (msg->arm) {
      call_arming(msg->arm);
    }
    if (msg->set_mode) {
      call_set_mode(msg->mode);
    }
    if (msg->takeoff) {
      call_takeoff(msg->takeoff_alt);
    }
    if (msg->land) {
      call_land();
    }
    if (msg->return_to_launch) {
      call_set_mode(6); // 映射为Land
    }
  }

  // ===== 定时发布聚合状态 =====
  void timer_status()
  {
    status_.header.stamp = this->now();
    status_.stamp = status_.header.stamp;
    status_pub_->publish(status_);
  }

  // ===== 定时发布设定值到MAVROS (保证>=2Hz) =====
  void timer_setpoint()
  {
    if (!has_setpoint_ || !status_.offboard_enabled) {
      return;
    }
    const auto & sp = latest_setpoint_;

    // 位置控制: NED -> ENU
    if (sp.position_valid) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      // NED(x=N,y=E,z=D) -> ENU(x=E,y=N,z=U)
      pose.pose.position.x = sp.position[1];  // East
      pose.pose.position.y = sp.position[0];  // North
      pose.pose.position.z = -sp.position[2]; // Up
      // 偏航: NED yaw -> ENU yaw = pi/2 - yaw_ned
      double yaw_enu = M_PI_2 - sp.yaw;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw_enu);
      pose.pose.orientation.w = q.w();
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      sp_position_pub_->publish(pose);
    }

    // 速度控制: NED -> ENU
    if (sp.velocity_valid) {
      geometry_msgs::msg::TwistStamped twist;
      twist.header.stamp = this->now();
      twist.header.frame_id = "map";
      twist.twist.linear.x = sp.velocity[1];  // East
      twist.twist.linear.y = sp.velocity[0];  // North
      twist.twist.linear.z = -sp.velocity[2]; // Up
      // 偏航角速度: NED与ENU符号相反
      twist.twist.angular.z = -sp.yawspeed;
      sp_velocity_pub_->publish(twist);
    }

    // 姿态控制
    if (sp.attitude_valid) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      // NED四元数 -> ENU四元数
      tf2::Quaternion q_ned(sp.quaternion[1], sp.quaternion[2], sp.quaternion[3], sp.quaternion[0]);
      tf2::Matrix3x3 m(q_ned);
      double r, p, y_ned;
      m.getRPY(r, p, y_ned);
      double y_enu = M_PI_2 - y_ned;
      tf2::Quaternion q_enu;
      q_enu.setRPY(r, p, y_enu);
      pose.pose.orientation.w = q_enu.w();
      pose.pose.orientation.x = q_enu.x();
      pose.pose.orientation.y = q_enu.y();
      pose.pose.orientation.z = q_enu.z();
      sp_attitude_pub_->publish(pose);
    }
  }

  // ===== 服务调用 =====
  void call_arming(bool arm)
  {
    if (!arming_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "arming 服务不可用");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    arming_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "请求 %s", arm ? "解锁" : "上锁");
  }

  void call_set_mode(uint8_t mode)
  {
    if (!set_mode_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "set_mode 服务不可用");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0;
    // 飞艇模式字符串(与kModeStrToId反向)
    static const std::unordered_map<uint8_t, std::string> kIdToModeStr = {
      {0, "MANUAL"},
      {1, "STABILIZED"},
      {2, "ALTCTL"},
      {3, "POSCTL"},
      {4, "OFFBOARD"},
      {5, "AUTO.TAKEOFF"},
      {6, "AUTO.LAND"},
      {8, "AUTO.MISSION"},
    };
    auto it = kIdToModeStr.find(mode);
    if (it != kIdToModeStr.end()) {
      req->custom_mode = it->second;
      set_mode_client_->async_send_request(req);
      RCLCPP_INFO(this->get_logger(), "请求切换模式: %s(%u)", it->second.c_str(), mode);
    } else {
      RCLCPP_WARN(this->get_logger(), "未知模式: %u", mode);
    }
  }

  void call_takeoff(float alt)
  {
    if (!takeoff_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "takeoff 服务不可用");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->min_pitch = 0.0f;
    req->yaw = 0.0f;
    req->latitude = 0.0f;
    req->longitude = 0.0f;
    req->altitude = (alt > 0.0f) ? alt : 20.0f; // 默认AS_TAKEOFF_ALT=20m
    takeoff_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "请求起飞到 %.1fm", req->altitude);
  }

  void call_land()
  {
    if (!land_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "land 服务不可用");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->min_pitch = 0.0f;
    req->yaw = 0.0f;
    req->latitude = 0.0f;
    req->longitude = 0.0f;
    req->altitude = 0.0f;
    land_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "请求降落");
  }

  // ===== 成员变量 =====
  // 参数
  double status_rate_;
  double setpoint_rate_;
  double alt_max_;
  double alt_min_;
  std::string fcu_url_;

  // 状态
  airship_msgs::msg::AirshipStatus status_;
  rclcpp::Time last_data_time_;

  // 设定值缓存
  airship_msgs::msg::OffboardSetpoint latest_setpoint_;
  bool has_setpoint_ = false;

  // 订阅 - MAVROS
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_alt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  // 订阅 - 伴飞电脑命令
  rclcpp::Subscription<airship_msgs::msg::OffboardSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<airship_msgs::msg::ModeCommand>::SharedPtr mode_cmd_sub_;

  // 发布
  rclcpp::Publisher<airship_msgs::msg::AirshipStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sp_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr sp_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sp_attitude_pub_;

  // 服务客户端
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

  // 定时器
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AirshipBridge>());
  rclcpp::shutdown();
  return 0;
}
