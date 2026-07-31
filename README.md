# Vehicle Dynamics Engineering Toolkit

[![pytest](https://github.com/Young-skyyy/vehicle-dynamics-toolkit/actions/workflows/test.yml/badge.svg)](https://github.com/Young-skyyy/vehicle-dynamics-toolkit/actions/workflows/test.yml)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

*A pure-Python vehicle dynamics simulation toolkit — no MATLAB/Simulink required.*

![Dashboard](dashboard.png)

*BSFC fuel map + iso-power lines | Steady-state cornering | Turning radius | Step steer transient | IDM ACC*

---

## Why This Exists

I built this while learning vehicle dynamics. Every textbook formula — SAE J2263, Pacejka tire model, BSFC bilinear interpolation — looks clean on paper but gets messy in code. This repo is my translation layer: **formula → Python → passing tests → visual output**.

If you're a student preparing for an automotive software testing interview, or learning CAN/UDS protocols, you'll find working reference implementations here.

### 面试高频问题 → 本项目如何回答

车辆软件测试面试中，面试官不会只问你"会不会 Python"，而是追问你对物理模型的理解深度。以下每一个问题，本项目都有可运行的代码作为答案：

| 面试官可能问 | 本项目的实现 | 对应文件 |
|---|---|---|
| "SAE J2263 滚动阻力模型和恒定系数有什么区别？为什么用 f₄v⁴ 项？" | 动态滚动阻力 μ(v)=f₀+f₁v+f₄v⁴，4 阶多项式捕捉高速非线性增长，并有 8 条 pytest 用例验证不同速度下的阻力值 | `vehicle.py` → `rolling_coeff_dynamic()` |
| "Pacejka 魔术公式各参数 B/C/D/E 的物理含义？和线性轮胎模型差在哪？" | 完整实现 Fy=D·sin(C·arctan(Bα−E(Bα−arctan(Bα))))，并有 `test_pacejka_vs_linear` 对比两种模型在大侧偏角下的分歧 | `lateral_dynamics.py` → `calc_pacejka_lateral_force()` |
| "BSFC 万有特性图怎么看？怎么查某个工况点的油耗？" | 15×12 双线性插值网格（180 个数据点），含等功率线叠加，dashboard 可直观看出最低油耗区 | `bsfc.py` → `_interpolate_bsfc()` |
| "WLTC 循环中 DFCO 断油和 enrichment 加浓怎么判断？" | 瞬态仿真逐秒判断：减速且转速高于断油阈值→DFCO（fuel=0），大负荷且转速高于加浓阈值→功率加浓（×1.15） | `wltc.py` → `get_wltc_profile()` |
| "CAN 总线 Motorola 和 Intel 字节序的区别？怎么验证编码正确？" | 两种字节序均实现，42 条 pytest 做 encode→decode 往返验证（`assert original == decoded`） | `can_demo.py` |
| "UDS 0x27 SecurityAccess 的 Seed & Key 流程？DTC Status Byte 各位含义？" | 完整状态机实现：会话切换→请求种子→计算密钥→解锁，DTC 状态字节 bit0~bit7 逐位注释 | `uds.py` |
| "不足转向梯度 Kus 怎么算？怎么判断一辆车转向不足还是过度？" | Kus = Wf/Cf − Wr/Cr，正值为不足转向，附特征车速/临界车速计算公式，稳态回转表输出 | `lateral_dynamics.py` → `calc_understeer_gradient()` |
| "IDM 跟车模型的时间间隔 T 增大 0.5s 会怎样？" | 参数化 IDM：T/s₀/b 可调，`car_following_simulation()` 可对比不同参数下的跟车距离曲线 | `vehicle.py` → `idm_acceleration()` |
| "这个函数的边界条件你测了吗？" | **209 条 pytest**，覆盖扭矩插值边界、BSFC 网格外推、CAN 非法 DLC、UDS 负响应码、IDM 收敛、Pacejka vs 线性对比 | 3 个 test_*.py |

---

## 30-Second Quick Start

```bash
git clone https://github.com/Young-skyyy/vehicle-dynamics-toolkit.git
cd vehicle-dynamics-toolkit
pip install -r requirements.txt
python vehicle_dynamics.py        # longitudinal + lateral dynamics
python can_demo.py                # CAN bus + UDS diagnostics
python plot_dashboard.py          # 5-in-1 dashboard (generates dashboard.png)
```

---

## What's Inside

| Module | Capabilities |
|--------|-------------|
| **Longitudinal Dynamics** | Engine torque curve → gear ratios → wheel force (replaces simplified P=Fv) |
| **Acceleration** | 0–100 km/h WOT simulation, 5-speed automatic shifting (92% redline upshift) |
| **Resistance** | SAE J2263 dynamic rolling resistance: μ(v)=f₀+f₁v+f₄v⁴ |
| **Fuel Consumption** | BSFC map bilinear interpolation (180 data points) + WLTC Class 3 transient (DFCO fuel-cut + enrichment) |
| **Lateral Dynamics** | 2-DOF bicycle model / slip angles / understeer gradient / Pacejka Magic Formula |
| **IDM Car-Following** | Time headway T, minimum gap s₀, comfortable deceleration b — parameterized |
| **CAN Bus** | 5 ECUs (EMS/BMS/ABS/TCU/BCM), Motorola & Intel byte order, DBC export, bus load monitoring, error injection |
| **UDS Diagnostics** | ISO 14229: Session Control (0x10), Read DID (0x22), Read DTC (0x19), Security Access (0x27), ECU Reset (0x11), DTC Status Byte |

---

## Tests

**209 pytest cases**, CI-enabled via GitHub Actions:

```bash
python -m pytest test_vehicle_dynamics.py test_can_demo.py test_uds.py -v
```

Covers: torque curve interpolation, BSFC bilinear interpolation, CAN encode/decode roundtrip (Motorola & Intel), UDS positive/negative responses, IDM convergence, Pacejka vs. linear tire model comparison.

---

## Key Techniques

- **Engine torque curve**: normalized WOT curve × peak torque → linear interpolation → gear ratio amplification → wheel force
- **BSFC bilinear interpolation**: 15×12 grid lookup on the engine fuel map
- **SAE J2263 coastdown model**: f₀ + f₁v + f₄v⁴ — 4th-order polynomial rolling resistance
- **2-DOF bicycle model**: slip angles → lateral forces → yaw moment → Euler-integrated transient response
- **Understeer gradient**: Kus = Wf/Cf − Wr/Cr, distinguishing understeer/neutral/oversteer
- **Pacejka Magic Formula**: Fy = D·sin(C·arctan(Bα − E(Bα − arctan(Bα))))
- **CAN frame encoding**: 11-bit arbitration ID, 8-byte payload, Motorola & Intel endianness
- **UDS diagnostic stack**: DiagnosticSession state machine, SecurityAccess Seed&Key, ISO 14229-1 DTC Status Byte

---

## Project Structure

```
├── vehicle.py              # Vehicle physics & powertrain
├── lateral_dynamics.py     # Lateral dynamics (bicycle model + Pacejka)
├── bsfc.py                 # BSFC fuel map + bilinear interpolation
├── wltc.py                 # WLTC Class 3 transient fuel simulation
├── can_demo.py             # CAN bus simulation + DBC generation
├── uds.py                  # UDS diagnostic protocol stack
├── plot_dashboard.py       # 5-in-1 dashboard
├── plotting.py             # BSFC contour heatmap
├── test_vehicle_dynamics.py  # 139 unit tests
├── test_can_demo.py        # 42 CAN tests
├── test_uds.py              # 28 UDS tests
├── _constants.py            # Physical constants
└── requirements.txt
```

---

## License

MIT
