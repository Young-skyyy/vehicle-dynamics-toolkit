"""vehicle_sim.launch.py — 启动动力学节点 + 油门指令节点"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 车辆动力学仿真节点 (100Hz)
        Node(
            package='vehicle_dynamics_node',
            executable='dynamics_node',
            name='vehicle_dynamics_node',
            output='screen',
            parameters=[{
                'dt': 0.01,          # 100Hz
                'mass': 1500.0,
                'max_torque': 250.0,
                'cd': 0.30,
                'frontal_area': 2.2,
                'rolling_coeff': 0.015,
                'wheel_radius': 0.33,
                'gear_ratio': 3.5,
            }]
        ),
        # 油门指令节点 (30% 恒定油门)
        Node(
            package='vehicle_dynamics_node',
            executable='throttle_pub',
            name='throttle_publisher',
            output='screen',
            parameters=[{
                'throttle': 0.5,  # 50% 油门起步
                'publish_hz': 50.0,
            }]
        ),
    ])
