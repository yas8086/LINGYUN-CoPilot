# Offboard控制设定值
# 伴飞电脑通过此消息控制飞艇，对应uORB的 trajectory_setpoint / vehicle_attitude_setpoint
# 飞艇约束: thrust_y始终为0(无侧向控制), 偏航能力受限

std_msgs/Header header

# ===== 控制模式标志 (对应 offboard_control_mode) =====
bool position_valid         # 位置控制有效
bool velocity_valid         # 速度控制有效
bool acceleration_valid     # 加速度控制有效
bool attitude_valid         # 姿态控制有效
bool body_rate_valid        # 角速率控制有效
bool thrust_torque_valid    # 推力/力矩控制有效

# ===== 位置控制 (NED坐标系) =====
float32[3] position         # 目标位置 [m], NAN表示无效
float32[3] velocity         # 目标速度 [m/s], NAN表示无效
float32[3] acceleration     # 目标加速度 [m/s2], NAN表示无效

# ===== 姿态控制 =====
float32[4] quaternion       # 期望姿态四元数 [w,x,y,z]
float32 yaw                 # 目标偏航角 [rad]
float32 yawspeed            # 目标偏航角速度 [rad/s]

# ===== 推力控制 (机体坐标系) =====
# 飞艇: thrust_x=前进[0,1], thrust_z=升降[-1,1](负=上升), thrust_y必须为0
float32[3] thrust_body      # [thrust_x, 0, thrust_z]
float32[3] torque_body      # [0, torque_y(俯仰), torque_z(偏航)]

# ===== 偏航设定值变化率 =====
float32 yaw_setpoint_move_rate  # 偏航设定值变化率 [rad/s]
