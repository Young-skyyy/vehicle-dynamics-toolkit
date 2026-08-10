# Vehicle Dynamics Engineering Toolkit

[![pytest](https://github.com/Young-skyyy/vehicle-dynamics-toolkit/actions/workflows/test.yml/badge.svg)](https://github.com/Young-skyyy/vehicle-dynamics-toolkit/actions/workflows/test.yml)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)](https://en.cppreference.com/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Star](https://img.shields.io/github/stars/Young-skyyy/vehicle-dynamics-toolkit?style=social)](https://github.com/Young-skyyy/vehicle-dynamics-toolkit)

*A vehicle dynamics simulation toolkit — Python for offline analysis, C++ ROS2 for real-time closed-loop simulation.*

![Dashboard](dashboard.png)

*BSFC fuel map + iso-power lines | Steady-state cornering | Turning radius | Step steer transient | IDM ACC*

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 1: Python Simulation Library (离线分析)                │
│  vehicle.py · lateral_dynamics.py · bsfc.py · wltc.py       │
│  can_demo.py · uds.py · plot_dashboard.py                   │
│  237 pytest  ·  GitHub Actions CI                           │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: C++ ROS2 Real-Time Nodes (实时闭环仿真)              │
│  vehicle_dynamics_node (rclcpp, 100Hz)                      │
│  uds_server (ISO 14229 Service, 5 ECUs)                     │
│  vehicle_msgs (ROS2 IDL: State/Control/UDS)                 │
└─────────────────────────────────────────────────────────────┘
```

Layer 2 的 C++ 节点从 Layer 1 的 Python 模型中移植算法，逐秒输出精确一致（偏差 < 0.1%）。

---

## 30-Second Quick Start

### Python (离线分析)

```bash
git clone https://github.com/Young-skyyy/vehicle-dynamics-toolkit.git
cd vehicle-dynamics-toolkit
pip install -r requirements.txt
python vehicle_dynamics.py        # longitudinal + lateral dynamics
python can_demo.py                # CAN bus + UDS diagnostics
python plot_dashboard.py          # 5-in-1 dashboard
```

### ROS2 (实时仿真, 需 Ubuntu 22.04 + ROS2 Humble)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# 车辆动力学仿真
ros2 launch vehicle_dynamics_node vehicle_sim.launch.py

# UDS 诊断服务器
ros2 run uds_server uds_server_node

# 另开终端: UDS 自动化测试
python3 src/uds_server/scripts/uds_test_client.py EMS
```

---

## What's Inside

| Module | Language | Capabilities |
|--------|----------|-------------|
| **Vehicle Dynamics Node** | C++ (rclcpp) | 100Hz 纵向+横向闭环仿真, 5速自动换挡, Pacejka轮胎 |
| **UDS Diagnostic Server** | C++ (rclcpp) | ISO 14229 Service: 0x10/0x11/0x22/0x27/0x19/0x3E, 5 ECUs, S3超时 |
| **Custom ROS2 Messages** | ROS2 IDL | VehicleState (14字段), VehicleControl, UdsRequest |
| **Longitudinal Dynamics** | Python | Engine torque curve, gear ratios, wheel force |
| **Acceleration** | Python | 0–100 km/h WOT, 5-speed automatic shifting |
| **Resistance** | Python | SAE J2263 dynamic rolling resistance: μ(v)=f₀+f₁v+f₄v⁴ |
| **Fuel Consumption** | Python | BSFC map bilinear interpolation + WLTC Class 3 transient |
| **Lateral Dynamics** | Python | 2-DOF bicycle model, understeer gradient, Pacejka Magic Formula |
| **IDM Car-Following** | Python | Time headway T, minimum gap s₀ — parameterized |
| **CAN Bus** | Python | 5 ECUs, Motorola & Intel byte order, DBC export, bus load, error injection |
| **UDS Diagnostics** | Python | Session Control, Read DID/DTC, SecurityAccess Seed&Key, DTC Status Byte |

---

## Tests

**237 pytest cases** (Python) + **10-step UDS ROS2 Service test**, CI-enabled:

```bash
# Python 单元测试
python -m pytest test_vehicle_dynamics.py test_can_demo.py test_uds.py test_can_bus_load.py -v

# ROS2 UDS 诊断服务测试
ros2 run uds_server uds_server_node &
python3 ros2_ws/src/uds_server/scripts/uds_test_client.py EMS
```

---

## Project Structure

```
├── vehicle.py                    # Vehicle physics & powertrain (Python)
├── lateral_dynamics.py           # Lateral dynamics (bicycle model + Pacejka)
├── bsfc.py                       # BSFC fuel map + bilinear interpolation
├── wltc.py                       # WLTC Class 3 transient fuel simulation
├── can_demo.py                   # CAN bus simulation + DBC generation
├── uds.py                        # UDS diagnostic protocol stack (Python ref)
├── vehicle_dynamics.py           # Main demo entry
├── plot_dashboard.py             # 5-in-1 dashboard
├── plotting.py                   # BSFC contour heatmap
├── _constants.py                 # Physical constants
├── _plot_utils.py                # matplotlib Chinese font
├── test_*.py                     # 237 pytest cases
│
├── ros2_ws/                      # ── ROS2 工作空间 (C++ + Python) ──
│   └── src/
│       ├── vehicle_msgs/             # 自定义消息 + 服务 (ROS2 IDL)
│       │   ├── msg/VehicleState.msg
│       │   ├── msg/VehicleControl.msg
│       │   └── srv/UdsRequest.srv
│       │
│       ├── vehicle_dynamics_node/    # C++ 动力学仿真节点
│       │   ├── src/vehicle_dynamics_node.cpp
│       │   ├── scripts/control_pub.py
│       │   └── launch/vehicle_sim.launch.py
│       │
│       └── uds_server/               # C++ UDS 诊断服务器
│           ├── src/uds_server_node.cpp
│           └── scripts/uds_test_client.py
│
├── requirements.txt
├── pyproject.toml
└── .pre-commit-config.yaml
```

---

## Key Techniques

- **Engine torque curve**: normalized WOT curve × peak torque → linear interpolation → gear ratio amplification → wheel force
- **BSFC bilinear interpolation**: 15×12 grid lookup on the engine fuel map
- **SAE J2263 coastdown model**: f₀ + f₁v + f₄v⁴ — 4th-order polynomial rolling resistance
- **2-DOF bicycle model**: slip angles → lateral forces → yaw moment → Euler-integrated transient response
- **Pacejka Magic Formula**: Fy = D·sin(C·arctan(Bα − E(Bα − arctan(Bα))))
- **CAN frame encoding**: 11-bit arbitration ID, 8-byte payload, Motorola & Intel endianness
- **UDS ISO 14229**: DiagnosticSession state machine, SecurityAccess Seed&Key, DTC Status Byte
- **ROS2 rclcpp**: Publisher/Subscriber/Service/Timer 全模式, 100Hz 实时仿真闭环

---

## License

MIT
