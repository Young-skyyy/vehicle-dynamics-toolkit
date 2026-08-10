# Changelog

## [2.0.0] — 2026-08-10

### 重大变更

- **Python 包重构**：从扁平文件结构迁移到 `src/vehicle_dynamics_toolkit/` 命名空间包
  - `_constants.py` 合并到 `vehicle.py`，消除单文件双常量的反模式
  - 所有模块 import 改为相对导入 (`from .vehicle import ...`)
  - 新增 `__init__.py` 导出完整公共 API
  - 新增 `__main__.py` 支持 `python -m vehicle_dynamics_toolkit` 入口
  - `pyproject.toml` 配置 `setuptools.packages.find` + `[project.scripts]` entry point
- **测试迁移**：测试文件从根目录迁移到 `tests/` 目录，240 条测试全部通过
- **`vehicle_dynamics.py` 移除**：根目录 god module 被 `__main__.py` 替代

### 新增

- **实车基准校验** (`validation.py`)
  - 3 款实车对标数据（Camry 2.0L / Civic 1.5T / Tiguan 2.0T），均标注来源
  - `validate_acceleration()` / `validate_braking()` / `validate_fuel_consumption()` 校验函数
  - `print_validation_report()` 格式化对比表格 + `explain_discrepancy()` 差异归因
- **RK4 积分器** (`lateral_dynamics.py`)
  - `simulate_step_steer()` 新增 `method="euler"|"rk4"` 参数
  - `compare_integrators.py` 验证脚本：dt=0.01 时两者稳态偏差均 <0.002%
- **C++ jitter 监控** (`vehicle_dynamics_node.cpp`)
  - 每次 `step()` 测量实际间隔 vs 期望 dt，输出 jitter (μs)
  - 指数移动平均 + 滑动窗口 P99 统计，每秒日志 + 每 10 秒完整报告
  - 可选 ROS2 发布到 `/vehicle/jitter_us` topic（`publish_jitter` 参数）

### CI 升级

- 新增 `generate-ref` job：生成 Python 参考输出并上传 artifact
- 新增 `ros2-build` job：在 ubuntu-22.04 + ROS2 Humble 上编译 C++ 包
- Python 测试改为 `pip install -e ".[test]"` + `pytest tests/ -v`
- mypy 类型检查目标从根目录改为 `src/vehicle_dynamics_toolkit/`

## [0.4.0] — 2026-08-10

### 新增

- **ROS2 集成** (`ros2_ws/`)
  - **C++ 车辆动力学节点** (`vehicle_dynamics_node`): rclcpp 生产级实时仿真，100Hz
    - 纵向模型：发动机外特性扭矩曲线插值 + 5 速自动换挡（目标转速 2000RPM）+ SAE J2263 行驶阻力
    - 横向模型：自行车模型（2-DOF）+ 线性/Pacejka 轮胎 + 横摆角速度 + 侧向位移
    - 发布 `VehicleState`（vx/vy/ax/ay/yaw_rate/gear/engine_rpm/steer_angle 等 14 字段）
    - 订阅 `VehicleControl`（throttle/brake/steer_angle），兼容旧 `Float64` throttle 接口
    - 15 个 ROS2 parameter 全可配置
  - **自定义消息** (`vehicle_msgs`): ROS2 IDL → C++ / Python 双语言
    - `VehicleState.msg`: 14 字段完整车辆状态
    - `VehicleControl.msg`: 油门 + 制动 + 转向
  - **控制指令节点** (`control_pub`): Python 驱动仿真闭环
  - 构建系统：ament_cmake (C++) + rosidl (IDL 消息生成)
- **WSL2 部署**：Ubuntu 22.04 + ROS2 Humble 完整环境

### 验证 (10s 50% 油门直行)
- 车速 29.1 m/s (105 km/h)，档位 5，RPM 2749
- 横向集成就绪（steer_angle=0 时 vy=0, yaw_rate=0）

## [0.3.0] — 2026-07-31

### 新增

- **CAN 总线负载率计算 + 丢包算法** (`can_bus_load_demo.py`)
  - 30 个 ECU 的真实整车 CAN 网络负载模拟
  - 位填充估算的 frame_bits() 公式（`47 + 8×N + (47+8×N)//10`）
  - simulate() 返回结构化 dict，print_simulation_result() 独立负责显示
  - BCM 故障注入演示（50ms → 1ms 周期疯狂发包）
  - simulate() 内 3 级 logger（INFO/WARNING/DEBUG）

- **28 条新 pytest** (`test_can_bus_load.py`)
  - frame_bits 手算验证、simulate() 负载率/超载/丢包算法全覆盖
  - 边界测试：空 ECU 列表、零阈值、零周期、JSON 序列化

- **CI 多版本矩阵** (`.github/workflows/test.yml`)
  - Python 3.10 / 3.11 / 3.12 并行测试
  - 新增 mypy 类型检查 job

- **车辆参数常量** (`_constants.py`)
  - `DEFAULT_CG_FRONT_RATIO = 0.45`：消除 vehicle.py 中的魔法数字

### 重构

- **DTC_DATABASE 合并**：`uds.py` 为唯一数据源（含 status byte），`can_demo.py` 从 `uds` import
- **UDS 诊断演示分离**：`run_diagnostic_session()` 返回 `list[dict]`，`print_diagnostic_session()` 显示
- **can_demo.py 解耦**：`__main__` 中的动态 `from uds import ...` 提升为顶部正规 import
- **wltc.py 消除重复**：提取 `_driver_p_controller()` 替代 `simulate_transient_cycle` 与 `simulate_wltc` 中重复的 ~24 行 P-controller
- **vehicle.py 懒加载**：`car_sedan/suv/truck` 改为 `__getattr__`，避免 `import vehicle` 时的实例化开销
- **can_demo.py 去重**：`calc_frame_bits` 改为 `from can_bus_load_demo import frame_bits`
- **simulate_step_steer 返回类型统一**：`list[tuple]` → `list[dict]`（time/vy/yaw_rate_rad/yaw_rate_deg/lateral_acc_g）

### 修复

- CAN 负载率演示：故障注入不再原地污染全局 `ECUS` 列表
- `frame_bits` 注释与代码不一致（注释写"填充位≈数据字节数"，代码是 `(47+data_bits)//10`）
- 波特率代码用 250kbps 但 print 写 "500 kbps" → 统一用变量
- `print_simulation_result` 中未使用的 `bus_speed` 局部变量
- `wltc.py` `simulate_wltc()` 中 `speed_error` 未定义（P-controller 提取后丢失）
- `vehicle_dynamics.py` 中 dict 解包错误（`simulate_step_steer` 改为 dict 后遗留的 tuple 解包）

### 文档

- README：测试数量 209 → 237，测试文件 3 → 4
- 测试 `docstring` 修正：`simulate_dtc_check` 已返回 dict 不再 print-only

## [0.2.0] — 2026-07-30

- Pacejka 魔术公式轮胎模型（线性/Pacejka 可切换）
- WLTC Class 3 瞬态油耗仿真
- CAN 总线多 ECU 仿真器（5 ECU + DBC/ASC/错误帧/DTC）
- UDS ISO 14229 诊断协议栈
- 209 条 pytest + GitHub Actions CI
