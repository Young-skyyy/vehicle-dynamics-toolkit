#!/usr/bin/env python3
"""控制指令发布 — ROS2 executable
发布 VehicleControl 到 /vehicle/control
"""
import rclpy
from rclpy.node import Node
from vehicle_msgs.msg import VehicleControl


class ControlPublisher(Node):
    def __init__(self):
        super().__init__('control_publisher')
        self.declare_parameter('throttle', 0.5)
        self.declare_parameter('brake', 0.0)
        self.declare_parameter('steer_angle', 0.0)
        self.declare_parameter('publish_hz', 50.0)

        throttle = self.get_parameter('throttle').value
        brake = self.get_parameter('brake').value
        steer = self.get_parameter('steer_angle').value
        hz = self.get_parameter('publish_hz').value

        self.pub = self.create_publisher(VehicleControl, '/vehicle/control', 10)
        self.timer = self.create_timer(1.0 / hz, self.publish_control)
        self.msg = VehicleControl()
        self.msg.throttle = throttle
        self.msg.brake = brake
        self.msg.steer_angle = steer

        self.get_logger().info(
            f'ControlPublisher: throttle={throttle:.0%} brake={brake:.0%} '
            f'steer={steer:.3f}rad @ {hz:.0f}Hz')

    def publish_control(self):
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = ControlPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
