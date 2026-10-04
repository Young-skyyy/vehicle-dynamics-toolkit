# 动力学修复与跨语言验证

## 修复范围

物理模型已修复起步、滑行、停车、ACC 和横向输出。当前新增独立解析基准验证，并通过 DynamicsECU 和原生 FMU 接通物理状态、控制输入与 CAN/UDS。C++ 核心由 ROS2、回放器和 FMU 共用。

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

Python 参考是 `simulation.py`，调用修复后的纵向和横向函数。C++ 实现在 `native/dynamics_core.hpp`，ROS2 节点与回放器共同包含这个头文件，节点中不再保留第二套公式。

每个控制样本作用于一个固定步长，输出记录是推进后的状态，首条时间是 dt。纵向在停车发生于步长内部时按停车时间计算距离；`ax` 为该步速度变化除以 dt。位置采用世界坐标，转向时 x/y 都进行坐标变换。

横向动力学使用当前纵向速度，显式欧拉子步由横向衰减速率限制。低于 1 m/s 使用运动学近似；它是避免低速奇异的简化切换，不是经过标定的低速轮胎模型。独立的 Python 阶跃转向也支持 RK4 和 Pacejka；本次跨语言对比只覆盖线性轮胎及默认车辆参数。

测试六种工况，每种 dt=0.01 和 0.005 秒，共 12 组、18,600 个样本：静止起步、滑行、制动、左转、右转、转弯制动至停止。相对容差为 0.1%，绝对容差为 1e-6（各字段 SI 单位），判定使用 `abs(actual-reference) <= atol + rtol*abs(reference)`；挡位必须完全一致，时间误差不超过 1e-9 秒。

空结果、缺列、缺样本、重复/错序样本、NaN/Inf、超差均不能通过。CI 同时构建 ROS2 动力学及 UDS 包，并使用 ROS2 包构建的回放器再做一次对比。

## 独立物理基准

`python scripts/validate_physics.py` 输出 `build/physics/report.json`，34 项检查
记录单位、预期值、实际值、误差和固定容差。预期值不调用生产代码的受力或积分函数：

- 无阻力匀速：v=v0，x=v0*t。
- 仅滚阻滑行/停车：a=μg，v=max(v0-a*t,0)，距离按实际运动时间积分。
- 仅空气阻力：k=ρ*Cd*A/(2m)，v=v0/(1+k*v0*t)，x=ln(1+k*v0*t)/k。
- 仅制动：a=0.8g*brake，停车距离 v0²/(2a)，包含步长内停车。
- 线性自行车稳态：r=v*δ/[L+(m/L)*(b/Cf-a/Cr)*v²]，
  vy=b*r-m*v²*r*a/(L*Cr)，ay=v*r；三个速度、左右两个方向。
- 空阻积分步长 .02/.01/.005 s，误差随步长减半呈一阶收敛。

使用模型约定 g=9.8 m/s²、ρ=1.225 kg/m³。匀速/恒减速度容差只允许浮点误差；
空阻容差允许显式欧拉截断误差；稳态转向在无纵向阻力、小转角下持续 10 s。
这些验证只适用于上述假设，不代表轮胎非线性、急转或实车标定精度。

## 物理状态与协议/FMUs 联调

```bash
python scripts/check_fmu_install.py --compiler c++ --output build/fmu/report.json
```

相同六种工况、两种步长共 18,600 个样本，逐字段对照 DynamicsModel、DynamicsECU
和原生 FMU；每组首步及每 100 步进行 CAN/UDS 读取，共 198 次。车速 CAN/UDS
编码精度为 0.01 km/h；RPM 的 CAN 精度为 0.25、UDS 精度为 1。量化误差不当成
物理模型误差。其他状态沿用上述 0.1% + 1e-6 容差，挡位精确相等、时间误差
不超过 1e-9 s。FMI 非法输入、错时、未初始化推进、终止后推进和重置同时检查。

FMU 是默认参考车辆，不支持任意 Python Vehicle 参数导出。新版控制输入 0..1，
新增转角、初速度和 SI 状态，旧版 FMU 需重建。温度/SOC/噪声轮速等仍是辅助信号。
CAN 延时期间物理 ECU 在下次观察或推进时持有上一控制追赶共享时钟，详见架构说明。

## 实车数据与来源

原 Camry/Civic/Tiguan 的数值缺少具体配置、可靠链接与测试条件，保留为历史兼容数据，
统一标记 UNVERIFIED，不再计算所谓实车通过率，也不再把多车型制动均值作为实车基准。

`validation.PUBLISHED_SPECIFICATIONS` 记录 [大众官方技术规格](https://www.volkswagen-newsroom.com/en/the-new-tiguan-allspace-test-drives-7543/technical-data-7556)
中的 2021 Tiguan Allspace 1.5 TSI 110 kW 六挡手动版本、发布日期和核验日期。
其中 0–100 km/h 为 10.3 s；该页没有完整传动比、外特性扭矩曲线及轮胎参数，
缺失项显式列出。厂商规格不能替代连续实测日志，不能据此宣称车型已标定。

实测对照入口：

```bash
python scripts/compare_measured_trace.py --reference measured.csv --actual simulated.csv --metadata measurement.json
```

两个 CSV 都含 `t`（秒）和要核验的 SI 字段，时间严格递增、样本数相同，
时间差不超过 1e-9 s。拒绝空文件、缺列、重复/错序时间、NaN/Inf，禁止隐式删样本
或插值。JSON 需提供以下结构，数值阈值由试验要求预先确定：

```json
{
  "kind": "measured",
  "source": "试验报告编号、实测日志路径或来源链接",
  "vehicle": "年份、车型、配置",
  "configuration": {"mass_kg": 1500, "input_log": "试验控制输入日志"},
  "conditions": {"road": "实测路面", "payload_kg": 0, "tire": "型号/压力", "temperature_degC": 20},
  "units": {"vx": "m/s", "yaw_rate": "rad/s"},
  "limits": {
    "vx": {"max_abs_error": 0.5, "rmse": 0.2},
    "yaw_rate": {"max_abs_error": 0.02, "rmse": 0.01}
  }
}
```

上面只是字段格式和阈值示例，不是实际车辆记录或项目达标要求。报告计算最大绝对误差、
RMSE、逐字段判定和两个 CSV 的 SHA-256，保留来源声明。来源由调用者提供，程序
不会自动认证其真实性；模型必须复用相同车型参数、工况和控制时间线。
仓库没有实测数据，因此当前实车精度状态仍为 NOT_VALIDATED。

## 证据的边界

独立解析基准说明约定的物理方程和数值方法满足这些受控工况；跨语言及适配器一致性
说明物理结果经不同接口读取时没有被另一套信号公式替换。它们都不是实车验证。

本地 Windows 已执行 pytest、mypy、C++ 核心回放和原生 FMU/CAN/UDS 联调；
报告存放在 build 下，不提交编译产物。完整 ROS2 节点构建、话题通信、ROS bag、
调度截止时间、非默认车型与真实试验精度仍需各自的验证。
