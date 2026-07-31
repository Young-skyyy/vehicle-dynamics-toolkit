# Changelog

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
