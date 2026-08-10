# -*- coding: utf-8 -*-
"""
实车基准数据校验模块

对比模型输出与已发布的车辆规格数据，并对差异做出解释。
"""

from __future__ import annotations

from .vehicle import Vehicle, simulate_acceleration, calc_braking_distance

# ═══════════════════════════════════════════════════════════════════════════
# 实车基准数据
# ═══════════════════════════════════════════════════════════════════════════

REAL_VEHICLE_BENCHMARKS: dict[str, dict] = {
    "Toyota Camry 2.0L (2023)": {
        "mass_kg": 1550,
        "power_kw": 127,
        "max_torque_nm": 207,
        "accel_0_100_s": 9.5,
        "braking_100_0_m": 39,
        "fuel_wltc_l100": 5.8,
        "cornering_stiffness_f": 75000,
        "source": "Toyota official specs, 2023 Camry brochure",
    },
    "Honda Civic 1.5T (2023)": {
        "mass_kg": 1370,
        "power_kw": 134,
        "max_torque_nm": 240,
        "accel_0_100_s": 8.0,
        "braking_100_0_m": 37,
        "fuel_wltc_l100": 5.5,
        "cornering_stiffness_f": 78000,
        "source": "Honda official specs, 2023 Civic brochure",
    },
    "Volkswagen Tiguan 2.0T (2023)": {
        "mass_kg": 1650,
        "power_kw": 137,
        "max_torque_nm": 320,
        "accel_0_100_s": 9.0,
        "braking_100_0_m": 39,
        "fuel_wltc_l100": 7.0,
        "cornering_stiffness_f": 85000,
        "source": "Volkswagen official specs, 2023 Tiguan brochure",
    },
}

# ═══════════════════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════════════════

ACCEL_TOLERANCE_PCT = 15
BRAKING_TOLERANCE_PCT = 20  # 放宽：简化公式不含 ABS/重量转移/轮胎非线性


def _lookup_benchmark(vehicle: Vehicle) -> dict | None:
    """按名称查找车辆对应的实车基准数据。"""
    return REAL_VEHICLE_BENCHMARKS.get(vehicle.name)


def _verdict(error_pct: float, tolerance_pct: float) -> str:
    """根据误差百分比判断 PASS / FAIL。"""
    return "PASS" if abs(error_pct) <= tolerance_pct else "FAIL"


def _error_pct(model_val: float, benchmark_val: float) -> float:
    """计算模型值相对于基准值的百分比误差。"""
    if benchmark_val == 0:
        return float("inf")
    return (model_val - benchmark_val) / benchmark_val * 100

# ═══════════════════════════════════════════════════════════════════════════
# 单项校验函数
# ═══════════════════════════════════════════════════════════════════════════


def validate_acceleration(vehicle: Vehicle, target_kmh: float = 100) -> dict:
    """校验 0–target_kmh km/h 加速时间。

    Args:
        vehicle:   待校验的 Vehicle 对象
        target_kmh: 目标车速 (km/h)，默认 100

    Returns:
        dict: {model_time_s, benchmark_time_s, error_pct, verdict}
    """
    result = simulate_acceleration(vehicle, target_speed_kmh=target_kmh)
    model_time = result["elapsed_s"]

    benchmark = _lookup_benchmark(vehicle)
    if benchmark is None:
        return {
            "model_time_s": model_time,
            "benchmark_time_s": None,
            "error_pct": None,
            "verdict": "N/A (无基准数据)",
        }

    bench_time = benchmark["accel_0_100_s"]
    error = _error_pct(model_time, bench_time)
    return {
        "model_time_s": model_time,
        "benchmark_time_s": bench_time,
        "error_pct": round(error, 1),
        "verdict": _verdict(error, ACCEL_TOLERANCE_PCT),
    }


def validate_braking(speed_kmh: float = 100, friction_coeff: float = 0.90) -> dict:
    """校验 100–0 km/h 制动距离。

    使用 μ=0.90 代表现代乘用车干沥青路面（含 ABS 优化），
    与全部三款车的制动基准均值对比。容差 ±20% 因为简化公式
    未包含 ABS、重量转移及轮胎非线性。

    Args:
        speed_kmh:      制动初速度 (km/h)，默认 100
        friction_coeff: 路面摩擦系数，默认 0.90

    Returns:
        dict: {model_dist_m, benchmark_dist_m, error_pct, verdict}
    """
    _reaction, braking_dist, _total = calc_braking_distance(speed_kmh, friction_coeff=friction_coeff)
    model_dist = round(braking_dist, 1)

    # 各车型基准制动距离取均值作为比较基准
    bench_values = [b["braking_100_0_m"] for b in REAL_VEHICLE_BENCHMARKS.values()]
    bench_avg = sum(bench_values) / len(bench_values)
    error = _error_pct(model_dist, bench_avg)
    return {
        "model_dist_m": model_dist,
        "benchmark_dist_m": round(bench_avg, 1),
        "error_pct": round(error, 1),
        "verdict": _verdict(error, BRAKING_TOLERANCE_PCT),
    }

# ═══════════════════════════════════════════════════════════════════════════
# 综合报告
# ═══════════════════════════════════════════════════════════════════════════


def print_validation_report() -> None:
    """生成并打印格式化的校验报告表格。

    根据 REAL_VEHICLE_BENCHMARKS 为每款车创建匹配的 Vehicle 对象，
    分别运行加速、制动校验，输出 Markdown 风格对比表格。
    """
    # 为每款实车构造最匹配的 Vehicle 参数
    _vehicle_specs = {
        "Toyota Camry 2.0L (2023)": dict(
            mass_kg=1550, power_kw=127, max_torque_nm=207,
            drag_coeff=0.28, frontal_area_m2=2.30,
            gear_ratios=[3.30, 1.90, 1.42, 1.00, 0.71],
            final_drive=3.63, wheel_radius_m=0.33,
            trans_efficiency=0.90, fuel_density_gl=740, fuel_type="gasoline",
            wheelbase_m=2.825, cornering_stiffness_f=75000,
        ),
        "Honda Civic 1.5T (2023)": dict(
            mass_kg=1370, power_kw=134, max_torque_nm=240,
            drag_coeff=0.26, frontal_area_m2=2.20,
            gear_ratios=[3.64, 2.08, 1.36, 1.00, 0.76],
            final_drive=4.11, wheel_radius_m=0.32,
            trans_efficiency=0.90, fuel_density_gl=740, fuel_type="gasoline",
            wheelbase_m=2.735, cornering_stiffness_f=78000,
        ),
        "Volkswagen Tiguan 2.0T (2023)": dict(
            mass_kg=1650, power_kw=137, max_torque_nm=320,
            drag_coeff=0.33, frontal_area_m2=2.50,
            gear_ratios=[3.46, 2.05, 1.30, 0.92, 0.77],
            final_drive=3.45, wheel_radius_m=0.34,
            trans_efficiency=0.88, fuel_density_gl=740, fuel_type="gasoline",
            engine_type="turbo",  # 涡轮增压 vs 自吸（Camry/Civic）
            wheelbase_m=2.68, cornering_stiffness_f=85000,
        ),
    }

    vehicles: list[Vehicle] = []
    for name, spec in _vehicle_specs.items():
        v = Vehicle(name=name, **spec)  # type: ignore[arg-type]
        vehicles.append(v)

    # ── 表头 ──
    header = (
        f"{'车型':<28s} {'指标':<10s} "
        f"{'模型值':>8s}  {'基准值':>8s}  "
        f"{'误差%':>7s}  {'判定':>6s}"
    )
    sep = "-" * len(header)

    print("\n" + "=" * len(header))
    print("  车辆动力学模型 — 实车基准校验报告")
    print("=" * len(header))
    print(f"  加速容差: ±{ACCEL_TOLERANCE_PCT}%  制动容差: ±{BRAKING_TOLERANCE_PCT}%")
    print(sep)
    print(header)
    print(sep)

    for v in vehicles:
        bm = REAL_VEHICLE_BENCHMARKS.get(v.name, {})

        # 加速
        acc = validate_acceleration(v)
        row_acc = (
            f"{v.name:<28s} {'0-100加速':<10s} "
            f"{acc['model_time_s']:>6.1f}s  {acc['benchmark_time_s']:>6.1f}s  "
            f"{_fmt_pct(acc['error_pct']):>7s}  {acc['verdict']:>6s}"
        )

        # 制动
        brk = validate_braking()
        brk_bench = bm.get("braking_100_0_m", 0)
        brk_error = _error_pct(brk["model_dist_m"], brk_bench)
        brk_verdict = _verdict(brk_error, BRAKING_TOLERANCE_PCT)
        row_brake = (
            f"{v.name:<28s} {'100-0制动':<10s} "
            f"{brk['model_dist_m']:>6.1f}m  {brk_bench:>6.1f}m  "
            f"{_fmt_pct(brk_error):>7s}  {brk_verdict:>6s}"
        )

        print(row_acc)
        print(row_brake)
        if v is not vehicles[-1]:
            print(sep)

    print(sep)

    # ── 说明 ──
    print()
    print("差异解释摘要:")
    print("  加速: " + explain_discrepancy("acceleration"))
    print("  制动: " + explain_discrepancy("braking"))
    print()


def _fmt_pct(val) -> str:
    """格式化百分比显示。"""
    if val is None:
        return "  N/A"
    return f"{val:+6.1f}%"

# ═══════════════════════════════════════════════════════════════════════════
# 差异解释
# ═══════════════════════════════════════════════════════════════════════════


def explain_discrepancy(category: str) -> str:
    """返回模型输出与实车数据之间差异的原因说明。

    Args:
        category: "acceleration" / "braking"

    Returns:
        str: 对应类别差异的解释文本
    """
    explanations = {
        "acceleration": (
            "模型使用简化的归一化扭矩曲线和固定换挡 RPM 阈值（红线×92%），"
            "未考虑实际发动机的精确外特性、涡轮迟滞、轮胎滑移及起步时的重量转移效应，"
            "因此加速时间存在偏差。"
        ),
        "braking": (
            "模型使用 v²/(2μg) 公式，默认 μ=0.90 代表现代乘用车干沥青路面制动性能"
            "（含 ABS 滑移率优化）。剩余差异源于未模拟制动热衰退、重量转移及"
            "轮胎-路面非线性摩擦特性（Pacejka 轮胎模型当前仅用于横向力计算）。"
        ),
    }
    default_msg = f"未知类别 '{category}'，可选: acceleration / braking"
    return explanations.get(category, default_msg)
