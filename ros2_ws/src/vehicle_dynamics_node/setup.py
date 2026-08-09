from setuptools import find_packages, setup

package_name = 'vehicle_dynamics_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vehicle_sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mnq',
    maintainer_email='young-skyyy@users.noreply.github.com',
    description='Vehicle longitudinal dynamics simulation node for ROS2',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'dynamics_node = vehicle_dynamics_node.dynamics_node:main',
            'throttle_pub = vehicle_dynamics_node.throttle_pub:main',
        ],
    },
)
