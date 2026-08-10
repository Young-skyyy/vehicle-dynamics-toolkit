# 车辆状态消息 — 纵向 + 横向动力学
std_msgs/Header header

# 纵向
float64 vx              # 纵向车速 m/s
float64 ax              # 纵向加速度 m/s²

# 横向
float64 vy              # 侧向速度 m/s
float64 ay              # 侧向加速度 m/s²
float64 yaw_rate        # 横摆角速度 rad/s
float64 steer_angle     # 前轮转角 rad

# 位置
float64 position_x      # 纵向位移 m
float64 position_y      # 侧向位移 m
float64 heading         # 航向角 rad

# 动力总成
float64 engine_rpm      # 发动机转速 rpm
uint8   gear            # 当前档位 (0=空档)
float64 engine_torque   # 发动机扭矩 Nm
float64 throttle        # 油门开度 0~1
float64 brake           # 制动踏板 0~1
