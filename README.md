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

### 工程问题 → 本项目的处理方式

教科书公式在代码里往往比想象中复杂——SAE 标准、Pacejka 模型、CAN 协议，都是"公式写在纸上三行，代码要写三百行"的典型。下表记录了过程中踩过的坑和最终落地方式：

| 工程问题 | 本项目的处理方式 | 代码位置 |
|---|---|---|
| 滚动阻力不能简单地用常数系数 | SAE J2263 动态模型 μ(v)=f₀+f₁v+f₄v⁴，4 阶项捕捉 120 km/h 以上非线性增长，8 条 pytest 验证速度-阻力对应关系 | `vehicle.py` → `rolling_coeff_dynamic()` |
| 线性轮胎模型在大侧偏角下偏离真实曲线 | 同时实现线性模型和 Pacejka 魔术公式 Fy=D·sin(C·arctan(Bα−E(Bα−arctan(Bα))))，`test_pacejka_vs_linear` 量化两种模型的偏差 | `lateral_dynamics.py` → `calc_pacejka_lateral_force()` |
| BSFC 数据是离散网格点，任意工况需要插值 | 15×12 双线性插值（180 个数据点），叠加等功率线，dashboard 可定位最低油耗区 | `bsfc.py` → `_interpolate_bsfc()` |
| WLTC 瞬态仿真中减速≠零油耗 | 逐秒判断工况：减速+转速>断油阈值→DFCO 断油（fuel=0），大负荷+转速>加浓阈值→功率加浓（×1.15） | `wltc.py` → `get_wltc_profile()` |
| CAN 帧编码在 Motorola 和 Intel 字节序下结果不同 | 两种字节序均实现，42 条 pytest 做 encode→decode 往返验证：`assert original == decoded` | `can_demo.py` |
| UDS SecurityAccess 是带状态的多步交互，不是单次请求 | 完整状态机：默认会话→0x10 扩展会话→0x27 请求种子→计算密钥→解锁；DTC Status Byte 逐位注释 | `uds.py` |
| 不足转向梯度正负号容易混淆 | Kus = Wf/Cf − Wr/Cr，Kus>0 不足转向（民用车常态），附特征车速/临界车速公式，稳态回转表可直接输出 | `lateral_dynamics.py` → `calc_understeer_gradient()` |
| IDM 跟车模型参数对行为的影响不是线性的 | T/s₀/b 全部参数化暴露，`car_following_simulation()` 可一键对比不同参数组合下的跟车距离曲线 | `vehicle.py` → `idm_acceleration()` |
| 物理模型容易写出"看起来对但边界爆炸"的代码 | **237 条 pytest** 覆盖：扭矩插值边界、BSFC 网格外推、CAN 负载率计算、UDS 负响应码、IDM 收敛、Pacejka vs 线性对比 | 4 个 test_*.py |

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

> **Note:** The demo scripts use hardcoded vehicle/condition parameters for quick out-of-the-box runs. To switch vehicles (e.g. sedan → SUV) or adjust speed/grade, edit the call arguments at the bottom of `vehicle_dynamics.py`. A CLI wrapper is planned but not yet needed for the current scope.

---

## What's Inside

| Module | Capabilities |
|--------|-------------|
| **Longitudinal Dynamics** | Engine torque curve → gear ratios → wheel force (replaces simplified P=Fv) |
| **Acceleration** | 0–100 km/h WOT simulation, 5-speed automatic shifting (92% redline upshift) |
| **Resistance** | SAE J2263 dynamic rolling resistance: μ(v)=f₀+f₁v+f₄v⁴ |
| **Fuel Consumption** | BSFC map bilinear interpolation (180 data points, indicative — <i>not</i> calibration-grade) + WLTC Class 3 transient (DFCO fuel-cut + enrichment) |
| **Lateral Dynamics** | 2-DOF bicycle model / slip angles / understeer gradient / Pacejka Magic Formula |
| **IDM Car-Following** | Time headway T, minimum gap s₀, comfortable deceleration b — parameterized |
| **CAN Bus** | 5 ECUs (EMS/BMS/ABS/TCU/BCM), Motorola & Intel byte order, DBC export, bus load monitoring, error injection |
| **UDS Diagnostics** | ISO 14229: Session Control (0x10), Read DID (0x22), Read DTC (0x19), Security Access (0x27), ECU Reset (0x11), DTC Status Byte |

---

## Tests

**237 pytest cases**, CI-enabled via GitHub Actions:

```bash
python -m pytest test_vehicle_dynamics.py test_can_demo.py test_uds.py test_can_bus_load.py -v
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
├── vehicle.py                 # Vehicle physics & powertrain
├── lateral_dynamics.py        # Lateral dynamics (bicycle model + Pacejka)
├── bsfc.py                    # BSFC fuel map + bilinear interpolation
├── wltc.py                    # WLTC Class 3 transient fuel simulation
├── can_demo.py                # CAN bus simulation + DBC generation
├── uds.py                     # UDS diagnostic protocol stack
├── vehicle_dynamics.py        # Main demo entry (longitudinal + lateral)
├── plot_dashboard.py          # 5-in-1 dashboard (BSFC + cornering + turning + step steer + ACC)
├── plotting.py                # BSFC contour heatmap
├── run_pacejka_demo.py        # Pacejka vs. linear tire model side-by-side comparison
├── _constants.py              # Physical constants (G, RHO_AIR, etc.)
├── _plot_utils.py             # Cross-platform Chinese font detection for matplotlib
├── test_vehicle_dynamics.py   # 139 unit tests
├── test_can_demo.py           # 42 CAN tests
├── test_uds.py                # 28 UDS tests
├── requirements.txt
├── .pre-commit-config.yaml    # Pre-commit hooks (ruff, whitespace, YAML/TOML validation)
└── pyproject.toml
```

---

## License

MIT
