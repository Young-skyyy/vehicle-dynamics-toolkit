"""油门指令发布节点 — 用于测试动力学仿真

发布恒定油门开度到 /vehicle/throttle
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ThrottlePublisher(Node):
    """以恒定油门开度驱动仿真"""

    def __init__(self):
        super().__init__('throttle_publisher')

        self.declare_parameter('throttle', 0.3)   # 默认 30% 油门
        self.declare_parameter('publish_hz', 50.0)  # 50Hz 发布

        throttle = self.get_parameter('throttle').value
        hz = self.get_parameter('publish_hz').value

        self.pub = self.create_publisher(Float64, '/vehicle/throttle', 10)
        self.timer = self.create_timer(1.0 / hz, self.publish_throttle)

        self.msg = Float64(data=throttle)
        self.get_logger().info(f'ThrottlePublisher: {throttle*100:.0f}% @ {hz:.0f}Hz')

    def publish_throttle(self):
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrottlePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
