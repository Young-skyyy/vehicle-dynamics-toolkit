"""车辆纵向动力学 ROS2 节点 — 最小闭环仿真

简化模型：F = F_engine - F_drag - F_roll
发布 vehicle/state 到 100Hz
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class VehicleDynamicsNode(Node):
    """纯纵向动力学仿真节点

    订阅:
      /vehicle/throttle  (Float64, 0~1) — 油门开度
      /vehicle/brake     (Float64, 0~1) — 制动踏板

    发布:
      /vehicle/velocity  (Float64, m/s)
      /vehicle/position   (Float64, m)
      /vehicle/accel      (Float64, m/s²)
      /vehicle/engine_rpm (Float64, rpm)
    """

    def __init__(self):
        super().__init__('vehicle_dynamics_node')

        # --- ROS2 参数：仿真步长和车辆参数 ---
        self.declare_parameter('dt', 0.01)          # 100Hz 仿真
        self.declare_parameter('mass', 1500.0)      # 整备质量 kg
        self.declare_parameter('max_torque', 250.0)  # 发动机峰值扭矩 Nm
        self.declare_parameter('max_power_kw', 120.0)  # 发动机最大功率 kW
        self.declare_parameter('cd', 0.30)          # 风阻系数
        self.declare_parameter('frontal_area', 2.2) # 迎风面积 m²
        self.declare_parameter('rolling_coeff', 0.015)  # 滚动阻力系数
        self.declare_parameter('wheel_radius', 0.33)  # 车轮半径 m
        self.declare_parameter('gear_ratio', 3.5)   # 主减速比

        self.dt = self.get_parameter('dt').value
        self.mass = self.get_parameter('mass').value
        self.max_torque = self.get_parameter('max_torque').value
        self.max_power_kw = self.get_parameter('max_power_kw').value

        # --- 物理常量 ---
        self.G = 9.8
        self.RHO_AIR = 1.225

        # --- 车辆状态 ---
        self.v = 0.0           # 车速 m/s
        self.x = 0.0           # 位移 m
        self.throttle = 0.0    # 油门 0~1
        self.brake = 0.0       # 制动 0~1

        # --- Publisher: 发布车辆状态 ---
        self.vel_pub = self.create_publisher(Float64, '/vehicle/velocity', 10)
        self.pos_pub = self.create_publisher(Float64, '/vehicle/position', 10)
        self.accel_pub = self.create_publisher(Float64, '/vehicle/accel', 10)
        self.rpm_pub = self.create_publisher(Float64, '/vehicle/engine_rpm', 10)

        # --- Subscriber: 接收控制指令 ---
        self.throttle_sub = self.create_subscription(
            Float64, '/vehicle/throttle', self.throttle_callback, 10)
        self.brake_sub = self.create_subscription(
            Float64, '/vehicle/brake', self.brake_callback, 10)

        # --- Timer: 主仿真循环 ---
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info(f'VehicleDynamicsNode started @ {1.0 / self.dt:.0f} Hz')

    def throttle_callback(self, msg: Float64):
        self.throttle = max(0.0, min(1.0, msg.data))

    def brake_callback(self, msg: Float64):
        self.brake = max(0.0, min(1.0, msg.data))

    def engine_torque(self, rpm: float) -> float:
        """简化的发动机外特性扭矩曲线（归一化 + 峰值扭矩缩放）

        Args:
            rpm: 发动机转速

        Returns:
            扭矩 Nm
        """
        idle_rpm = 800.0     # 怠速
        max_rpm = 6500.0

        # 怠速最低扭矩（模拟发动机蠕行 + 起步补油）
        # 需克服滚动阻力 m*g*mu = 1500*9.8*0.015 ≈ 220N 起步
        if rpm < idle_rpm and self.throttle > 0:
            idle_tq = self.throttle * self.max_torque * 0.30  # 怠速约30%峰值扭矩
            return idle_tq

        normalized = rpm / max_rpm

        # 归一化扭矩曲线（汽油机典型形状）
        if normalized < 0.1:
            tq_norm = normalized / 0.1 * 0.7
        elif normalized < 0.55:
            tq_norm = 0.7 + (1.0 - 0.7) * (normalized - 0.1) / 0.45
        elif normalized < 0.85:
            tq_norm = 1.0
        else:
            tq_norm = 1.0 - (normalized - 0.85) / 0.15 * 0.5

        return tq_norm * self.max_torque * self.throttle

    def resistance_force(self, v: float) -> float:
        """行驶阻力 = 滚动阻力 + 空气阻力

        Args:
            v: 车速 m/s

        Returns:
            总阻力 N
        """
        F_roll = self.get_parameter('rolling_coeff').value * self.mass * self.G
        F_drag = 0.5 * self.RHO_AIR * self.get_parameter('cd').value \
                 * self.get_parameter('frontal_area').value * v * v
        return F_roll + F_drag

    def step(self):
        """主仿真步进（每个 dt 调用一次）"""
        # 1. 计算发动机转速
        wheel_radius = self.get_parameter('wheel_radius').value
        gear_ratio = self.get_parameter('gear_ratio').value
        rpm = self.v / (2 * 3.14159 * wheel_radius) * gear_ratio * 60.0

        # 2. 驱动力
        T_engine = self.engine_torque(rpm)
        F_drive = T_engine * gear_ratio / wheel_radius

        # 3. 行驶阻力
        F_resist = self.resistance_force(self.v)

        # 4. 制动力
        F_brake = self.brake * self.mass * self.G * 0.8  # 最大减速度约 0.8g

        # 5. 合力 → 加速度
        F_net = F_drive - F_resist - F_brake
        a = F_net / self.mass

        # 6. 欧拉积分
        self.v = max(0.0, self.v + a * self.dt)  # 不能倒退
        self.x += self.v * self.dt

        # 7. 发布状态
        self.vel_pub.publish(Float64(data=self.v))
        self.pos_pub.publish(Float64(data=self.x))
        self.accel_pub.publish(Float64(data=a))
        self.rpm_pub.publish(Float64(data=rpm))


def main(args=None):
    rclpy.init(args=args)
    node = VehicleDynamicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
