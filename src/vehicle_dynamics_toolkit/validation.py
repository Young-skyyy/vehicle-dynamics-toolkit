# -*- coding: utf-8 -*-
"""
基准来源管理与模型对照（历史数据未核验）

对比模型输出与已发布的车辆规格数据，并对差异做出解释。
"""

from __future__ import annotations

from .vehicle import Vehicle, simulate_acceleration, calc_braking_distance
from .lateral_dynamics import calc_understeer_gradient

# ═══════════════════════════════════════════════════════════════════════════
# 历史基准数据（未核验，仅保留兼容）
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
        "kus_deg_per_g": 2.5,  # family sedan: 2-3 deg/g (Gillespie, 1992)
        "source": "Legacy source claim unverified; do not use as validated evidence",
    },
    "Honda Civic 1.5T (2023)": {
        "mass_kg": 1370,
        "power_kw": 134,
        "max_torque_nm": 240,
        "accel_0_100_s": 8.0,
        "braking_100_0_m": 37,
        "fuel_wltc_l100": 5.5,
        "cornering_stiffness_f": 78000,
        "kus_deg_per_g": 1.8,  # sportier compact: 1.5-2.0 deg/g
        "source": "Legacy source claim unverified; do not use as validated evidence",
    },
    "Volkswagen Tiguan 2.0T (2023)": {
        "mass_kg": 1650,
        "power_kw": 137,
        "max_torque_nm": 320,
        "accel_0_100_s": 9.0,
        "braking_100_0_m": 39,
        "fuel_wltc_l100": 7.0,
        "cornering_stiffness_f": 85000,
        "kus_deg_per_g": 2.0,  # compact SUV: 2-3 deg/g, depends on tire/suspension
        "source": "Legacy source claim unverified; do not use as validated evidence",
    },
}

for _benchmark in REAL_VEHICLE_BENCHMARKS.values():
    _benchmark["evidence"] = "UNVERIFIED"
    _benchmark["source"] = "Legacy values: no traceable trim, source URL or matched test conditions"

# Verified primary source, specific trim and dated publication. This is a
# manufacturer specification, NOT measured telemetry or a calibrated model.
PUBLISHED_SPECIFICATIONS = {
    "Volkswagen Tiguan Allspace 1.5 TSI 110 kW 6-speed manual (2021)": {
        "mass_kg_min": 1571, "power_kw": 110, "max_torque_nm": 250,
        "accel_0_100_s": 10.3,
        "source_url": "https://www.volkswagen-newsroom.com/en/the-new-tiguan-allspace-test-drives-7543/technical-data-7556",
        "publication_date": "2021-10-11", "checked_date": "2026-10-04",
        "evidence": "manufacturer_specification",
        "conditions": "Manufacturer figure; minimum kerb weight, payload/test environment not specified here",
        "missing_model_parameters": ["gear ratios", "final drive", "torque curve", "drag area", "tire stiffness"],
    },
}

# ═══════════════════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════════════════

ACCEL_TOLERANCE_PCT = 15
BRAKING_TOLERANCE_PCT = 20  # legacy compatibility only; not a validation threshold
LATERAL_TOLERANCE_PCT = 25  # legacy compatibility only; no measured lateral target


def _lookup_benchmark(vehicle: Vehicle) -> dict | None:
    """按名称查找车辆对应的实车基准数据。"""
    return REAL_VEHICLE_BENCHMARKS.get(vehicle.name)


def _verdict(error_pct: float, tolerance_pct: float) -> str:
    """历史容差不再用于实车判定：缺少可追溯来源与条件。"""
    return "UNVERIFIED (历史基准来源/测试条件未核验)"


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
    if not result["reached_target"]:
        return {
            "model_time_s": model_time, "benchmark_time_s": None,
            "error_pct": None, "verdict": "INCOMPLETE (未达到目标车速)",
        }

    benchmark = _lookup_benchmark(vehicle)
    if benchmark is None or target_kmh != 100:
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

    μ 是调用者假设；不把多车型均值当作实车校验，也不宣称模拟 ABS。

    Args:
        speed_kmh:      制动初速度 (km/h)，默认 100
        friction_coeff: 路面摩擦系数，默认 0.90

    Returns:
        dict: {model_dist_m, benchmark_dist_m, error_pct, verdict}
    """
    _reaction, braking_dist, _total = calc_braking_distance(speed_kmh, friction_coeff=friction_coeff)
    model_dist = round(braking_dist, 1)

    # A fleet average is not a braking baseline for a particular vehicle.
    return {"model_dist_m": model_dist, "benchmark_dist_m": None,
            "error_pct": None, "verdict": "UNVERIFIED (缺少车型与路面条件匹配的实测制动数据)"}



def validate_lateral(vehicle: Vehicle) -> dict:
    """校验不足转向梯度 (Kus)。

    模型通过 cornering_stiffness_f/r 计算 Kus = Wf/Cf - Wr/Cr，
    与车辆类别的典型范围对比。

    Args:
        vehicle: 待校验的 Vehicle 对象

    Returns:
        dict: {model_kus_deg_per_g, benchmark_kus_deg_per_g, error_pct, verdict}
    """
    _, kus_deg = calc_understeer_gradient(vehicle)
    model_kus = round(kus_deg, 2)

    benchmark = _lookup_benchmark(vehicle)
    if benchmark is None or "kus_deg_per_g" not in benchmark:
        return {
            "model_kus_deg_per_g": model_kus,
            "benchmark_kus_deg_per_g": None,
            "error_pct": None,
            "verdict": "N/A (无基准数据)",
        }

    bench_kus = benchmark["kus_deg_per_g"]
    error = _error_pct(model_kus, bench_kus)
    return {
        "model_kus_deg_per_g": model_kus,
        "benchmark_kus_deg_per_g": bench_kus,
        "error_pct": round(error, 1),
        "verdict": _verdict(error, LATERAL_TOLERANCE_PCT),
    }


# ═══════════════════════════════════════════════════════════════════════════
# 综合报告
# ═══════════════════════════════════════════════════════════════════════════


def print_validation_report() -> None:
    """Print evidence levels without manufacturing a real-car PASS claim."""
    from .physical_validation import physics_report
    report = physics_report()
    print(f"Analytical verification: {report['checks']} checks; passed={report['passed']}")
    print("Real-vehicle accuracy: NOT_VALIDATED (no measured traces supplied)")
    for name, benchmark in PUBLISHED_SPECIFICATIONS.items():
        print(f"Manufacturer specification: {name}; 0-100={benchmark['accel_0_100_s']} s")
        print("Source:", benchmark["source_url"])
    print("Legacy Camry/Civic/Tiguan values: UNVERIFIED; excluded from PASS/FAIL evidence")


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
            "距离工具使用 v²/(2μg)，μ=0.90 只是调用者假设；物理状态模型使用 0.8g 制动上限。"
            "没有模拟 ABS 滑移控制、制动热衰退、重量转移及"
            "轮胎-路面非线性摩擦特性（Pacejka 轮胎模型当前仅用于横向力计算）。"
        ),
        "lateral": (
            "侧偏刚度和横摆惯量是模型参数，参考车辆分别使用 80000/70000 N/rad。"
            "历史车辆类别目标没有实测来源。"
            "差异来自侧偏刚度非实测数据、简化自行车模型忽略侧倾/载荷转移效应。"
        ),
    }
    default_msg = f"未知类别 '{category}'，可选: acceleration / braking"
    return explanations.get(category, default_msg)
