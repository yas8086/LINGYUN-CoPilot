# 电机输出状态 (对应 actuator_motors uORB消息)
# 8个电机归一化输出 [0,1]

std_msgs/Header header

# ===== 8个电机归一化输出 =====
# 升力电机
float32 m0_front_up         # 前1上升 (CCW, 196N, +1.416e-03)
float32 m1_front_down       # 前2下降 (CW,  392N, -2.832e-03)
float32 m2_rear_down        # 后1下降 (CW,  392N, -2.832e-03)
float32 m3_rear_up          # 后2上升 (CCW, 196N, +1.416e-03)
# 推进电机
float32 m4_left_front       # 左前推进 (CCW)
float32 m5_left_rear        # 左后推进 (CW)
float32 m6_right_front      # 右前推进 (CCW)
float32 m7_right_rear       # 右后推进 (CW)

# ===== 4DOF控制量 (控制分配器输入) =====
float32 thrust_x            # 前进推力 [0,1], 0=悬停
float32 thrust_z            # 升降推力 [-1,1], 负=上升
float32 torque_y            # 俯仰力矩 [-0.5,0.5], 正=抬头
float32 torque_z            # 偏航力矩 [-0.5,0.5], 正=右转

# ===== 状态标志 =====
bool takeoff_land_mode      # 是否处于起飞/降落模式(推进电机关闭)
bool propellers_active      # 推进电机是否激活
