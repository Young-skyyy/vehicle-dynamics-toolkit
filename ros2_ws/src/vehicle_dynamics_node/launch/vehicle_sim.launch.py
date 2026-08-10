"""vehicle_sim.launch.py — C++ 动力学节点 + 控制指令"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── C++ 车辆动力学仿真节点 (100Hz) ──
        Node(
            package='vehicle_dynamics_node',
            executable='dynamics_node',
            name='vehicle_dynamics_node',
            output='screen',
            parameters=[{
                'dt': 0.01,
                'mass': 1500.0,
                'max_torque': 250.0,
                'max_power_kw': 140.0,
                'cd': 0.30,
                'frontal_area': 2.2,
                'rolling_coeff': 0.015,
                'wheel_radius': 0.32,
                'final_drive': 4.06,
                'wheelbase': 2.65,
                'cg_to_front': 1.2,
                'cornering_stiffness_f': 80000.0,
                'cornering_stiffness_r': 70000.0,
                'idle_rpm': 800.0,
                'max_rpm': 6200.0,
            }]
        ),
        # ── 控制指令节点：50% 油门直行 ──
        Node(
            package='vehicle_dynamics_node',
            executable='control_pub',
            name='control_publisher',
            output='screen',
            parameters=[{
                'throttle': 0.5,
                'brake': 0.0,
                'steer_angle': 0.0,
                'publish_hz': 50.0,
            }]
        ),
    ])
