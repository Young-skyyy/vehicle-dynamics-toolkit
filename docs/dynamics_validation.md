# 动力学修复与跨语言验证

## 修复范围

第一步修复起步、滑行、停车、ACC 和横向输出。第二步建立可执行的 Python/C++ 对比。CAN、UDS、FMU 尚未统一为同一个车辆状态核心，本次不宣称完成这一后续工作。

- `calc_wheel_force`：静止给油时，一挡使用怠速扭矩；这是简化滑动离合器假设，未模拟真实离合器或变矩器。
- `calc_acceleration`：行驶时保留负加速度，静止时阻力不能导致倒车；新增 `brake` 参数，最大制动输入对应 0.8g。
- `simulate_acceleration`：从 0 开始，包含 t=0 状态；返回 `reached_target` 和 `status`，120 秒超时不能冒充达标时间。最终时间步截断到目标速度事件。
- ACC：巡航设定速度独立于前车速度；IDM 仍是控制律，实际执行加速度限制在默认 [-8, 1.4] m/s²。`acc_ms2` 为当前采样点之后的指令，速度不再瞬间归零。碰撞以负间距和 `collision_s` 保留。
- 稳态转向：半径始终为非负大小，方向由横摆角速度符号表示；停车半径为无穷大，临界失稳速度及以上拒绝稳定解。
- 瞬态侧向加速度：`ay = dvy/dt + vx*r = (Fyf+Fyr)/m`。`vx*r` 单独使用只适合稳态；`dvy/dt` 单独使用不是实际侧向加速度。
- 转角为前轮转角，不是方向盘转角；长度 m、时间 s、速度 m/s、角度 rad（名称包含 deg/kmh 的接口除外）。

## 自动对比的运行方式

在仓库根目录安装 `python -m pip install -e ".[test]"`，并准备 C++17 编译器。

```bash
python -m pytest tests -q
python scripts/compare_py_cpp.py --check
```

Windows 可以使用 MinGW/Clang，或 `--compiler` 指向 Zig 的 `zig.exe`。带空格的路径需要引号。编译器及依赖不包含在 Git 源码中。

```bash
python scripts/compare_py_cpp.py --check --compiler "path/to/zig.exe"
```

也可以使用 ROS2 包构建出来的可执行文件：

```bash
colcon build --base-paths ros2_ws/src --packages-up-to vehicle_dynamics_node uds_server
python scripts/compare_py_cpp.py --check --runner install/vehicle_dynamics_node/lib/vehicle_dynamics_node/dynamics_runner
```

输出在 `build/parity/`：`inputs.csv` 是共同输入，`python.csv` 和 `cpp.csv` 是逐步状态，`report.json` 包含最大绝对误差和首批失败位置。

若只比较已有结果：

```bash
python scripts/compare_py_cpp.py --compare build/parity/cpp.csv --ref build/parity/python.csv
```

旧脚本打印的 `--parse-bag` 从未实现，现不再建议使用。`--generate --output` 接受输出目录并生成 CSV，而非旧版 JSON 文件。

## 计算与时间约定

Python 参考是 `simulation.py`，调用修复后的纵向和横向函数。C++ 实现在 `dynamics.hpp`，ROS2 节点与回放器共同包含这个头文件，节点中不再保留第二套公式。

每个控制样本作用于一个固定步长，输出记录是推进后的状态，首条时间是 dt。纵向在停车发生于步长内部时按停车时间计算距离；`ax` 为该步速度变化除以 dt。位置采用世界坐标，转向时 x/y 都进行坐标变换。

横向动力学使用当前纵向速度，显式欧拉子步由横向衰减速率限制。低于 1 m/s 使用运动学近似；它是避免低速奇异的简化切换，不是经过标定的低速轮胎模型。独立的 Python 阶跃转向也支持 RK4 和 Pacejka；本次跨语言对比只覆盖线性轮胎及默认车辆参数。

测试六种工况，每种 dt=0.01 和 0.005 秒，共 12 组、18,600 个样本：静止起步、滑行、制动、左转、右转、转弯制动至停止。相对容差为 0.1%，绝对容差为 1e-6（各字段 SI 单位），判定使用 `abs(actual-reference) <= atol + rtol*abs(reference)`；挡位必须完全一致，时间误差不超过 1e-9 秒。

空结果、缺列、缺样本、重复/错序样本、NaN/Inf、超差均不能通过。CI 同时构建 ROS2 动力学及 UDS 包，并使用 ROS2 包构建的回放器再做一次对比。

## 证据的边界

本次本地检查（2026-10-01，Python 3.12 / Windows）：295 项 pytest 测试通过（包括 cantools 互操作测试），mypy 检查 15 个源码文件通过。使用 Zig 0.16.0 的 C++ 编译器实际构建并运行回放器，12 组工况、18,600 个样本通过，最大车速绝对差约 7.11e-15 m/s。完整 ROS2 构建依赖本机未提供，新增的 GitHub Actions 规则尚未在远端执行。

跨语言一致说明两份实现对约定工况给出了相同结果；物理性质测试则检查起步、滑行、停车、左右对称、瞬态受力和碰撞报告。二者都不等于实车精度验证。

修复真实静止起步后，仓库基准中 Camry 0–100 为 11.6 s、Tiguan 为 10.7 s，超过原有 15% 容差。README 保留 FAIL；尚未进行真实车辆参数标定。

本地验证可执行 C++ 核心，不需要 ROS2。完整 ROS2 节点构建、话题通信、ROS bag 回放、调度截止时间、非默认车辆标定和 FMU 一致性是独立的验证范围，不应从核心对比结果推断它们通过。
