# -*- coding: utf-8 -*-
"""
BSFC 万有特性油耗模型：双线性插值查表 → L/100km
"""

from __future__ import annotations

import math

from .vehicle import KMH_TO_MS, SECONDS_PER_HOUR, SECONDS_PER_MINUTE
from .vehicle import Vehicle, calc_resistance, car_sedan, car_suv, car_truck


# ---- BSFC Map 定义 ----
# 横轴: 发动机转速 (RPM)
# 纵轴: 扭矩负荷比 (实际扭矩 / 最大扭矩)
# 值:  燃油消耗率 (g/kWh)，越小越省油
#
# 数据参考典型 2.0L 自然吸气汽油机台架测试
# 300+ 数据点，在原始 8×7 网格基础上通过双线性插值加密

_BSFC_RPM_GRID = [800, 1000, 1200, 1500, 1800, 2100, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 5800, 6200]
_BSFC_LOAD_GRID = [0.05, 0.10, 0.15, 0.22, 0.30, 0.40, 0.50, 0.60, 0.70, 0.78, 0.85, 1.0]

# 汽油机 BSFC map (15×12 = 180 数据点)
_BSFC_GASOLINE = [
    #  800  1000  1200  1500  1800  2100  2500  3000  3500  4000  4500  5000  5500  5800  6200  <- RPM
    [ 580,  530,  480,  440,  400,  383,  360,  370,  380,  405,  430,  465,  500,  526,  560],  # 5% load
    [ 490,  448,  405,  374,  342,  331,  315,  322,  330,  350,  370,  402,  435,  463,  500],  # 10% load
    [ 400,  365,  330,  308,  285,  279,  270,  275,  280,  295,  310,  340,  370,  400,  440],  # 15% load
    [ 358,  330,  302,  285,  269,  263,  255,  259,  264,  276,  289,  316,  342,  368,  403],  # 22% load
    [ 310,  290,  270,  260,  250,  245,  238,  242,  245,  255,  265,  288,  310,  331,  360],  # 30% load
    [ 292,  276,  260,  252,  245,  241,  236,  239,  242,  251,  260,  279,  298,  316,  340],  # 40% load
    [ 275,  262,  250,  245,  240,  237,  233,  236,  240,  248,  255,  270,  285,  300,  320],  # 50% load
    [ 270,  259,  248,  243,  239,  236,  233,  236,  240,  248,  255,  269,  282,  296,  315],  # 60% load
    [ 265,  255,  245,  242,  238,  236,  233,  236,  240,  248,  255,  268,  280,  293,  310],  # 70% load
    [ 263,  255,  247,  243,  240,  239,  237,  240,  244,  253,  262,  276,  291,  306,  326],  # 78% load
    [ 262,  255,  248,  245,  242,  241,  240,  244,  248,  258,  268,  284,  300,  317,  340],  # 85% load
    [ 275,  268,  260,  258,  255,  255,  255,  262,  270,  282,  295,  315,  335,  354,  380],  # 100% load
]

# 柴油机 BSFC map（12×12 = 144 数据点，整体比汽油机低 30-40 g/kWh）
_BSFC_DIESEL_RPM = [600, 800, 1000, 1200, 1500, 1800, 2000, 2400, 2800, 3200, 3500, 4000]
_BSFC_DIESEL = [
    #  600   800  1000  1200  1500  1800  2000  2400  2800  3200  3500  4000  <- RPM
    [ 480,  430,  380,  352,  310,  298,  290,  295,  300,  323,  340,  400],  # 5% load
    [ 400,  360,  320,  300,  270,  260,  252,  258,  262,  282,  298,  350],  # 10% load
    [ 320,  290,  260,  248,  230,  221,  215,  220,  225,  242,  255,  300],  # 15% load
    [ 287,  263,  239,  229,  215,  207,  202,  207,  211,  225,  236,  277],  # 22% load
    [ 250,  232,  215,  208,  198,  192,  188,  192,  195,  206,  215,  250],  # 30% load
    [ 238,  222,  208,  202,  194,  189,  185,  188,  192,  202,  210,  242],  # 40% load
    [ 225,  212,  200,  196,  190,  185,  182,  185,  188,  198,  205,  235],  # 50% load
    [ 222,  210,  199,  195,  189,  185,  182,  186,  189,  199,  206,  236],  # 60% load
    [ 218,  208,  198,  194,  188,  184,  182,  186,  190,  200,  208,  238],  # 70% load
    [ 216,  208,  199,  195,  190,  187,  185,  190,  194,  206,  214,  247],  # 78% load
    [ 215,  208,  200,  197,  192,  190,  188,  193,  198,  211,  220,  255],  # 85% load
    [ 228,  222,  215,  212,  208,  206,  205,  212,  218,  233,  245,  285],  # 100% load
]


def _interpolate_bsfc(rpm: float, load_ratio: float, fuel_type: str = "gasoline") -> float:
    """在 BSFC map 中双线性插值，返回 (rpm, load) 对应的 g/kWh"""
    if fuel_type == "diesel":
        rpm_grid = _BSFC_DIESEL_RPM
        bsft_map = _BSFC_DIESEL
    else:
        rpm_grid = _BSFC_RPM_GRID
        bsft_map = _BSFC_GASOLINE

    # 钳制到 map 边界内
    rpm = max(rpm_grid[0], min(rpm_grid[-1], rpm))
    load_ratio = max(_BSFC_LOAD_GRID[0], min(_BSFC_LOAD_GRID[-1], load_ratio))

    # 找转速区间
    i_rpm = 0
    for i in range(len(rpm_grid) - 1):
        if rpm_grid[i] <= rpm <= rpm_grid[i + 1]:
            i_rpm = i
            break
    # 找负荷区间
    i_load = 0
    for i in range(len(_BSFC_LOAD_GRID) - 1):
        if _BSFC_LOAD_GRID[i] <= load_ratio <= _BSFC_LOAD_GRID[i + 1]:
            i_load = i
            break

    # 双线性插值
    rpm_low, rpm_high = rpm_grid[i_rpm], rpm_grid[i_rpm + 1]
    load_low, load_high = _BSFC_LOAD_GRID[i_load], _BSFC_LOAD_GRID[i_load + 1]

    q11 = bsft_map[i_load][i_rpm]
    q12 = bsft_map[i_load][i_rpm + 1]
    q21 = bsft_map[i_load + 1][i_rpm]
    q22 = bsft_map[i_load + 1][i_rpm + 1]

    t_rpm = (rpm - rpm_low) / (rpm_high - rpm_low) if rpm_high != rpm_low else 0
    t_load = (load_ratio - load_low) / (load_high - load_low) if load_high != load_low else 0

    return (q11 * (1 - t_rpm) * (1 - t_load) +
            q12 * t_rpm * (1 - t_load) +
            q21 * (1 - t_rpm) * t_load +
            q22 * t_rpm * t_load)


def calc_fuel_consumption(vehicle: Vehicle, speed_kmh: float, distance_km: float) -> float:
    """基于 BSFC 万有特性 map 计算油耗（取整版，用于展示）。"""
    return round(_calc_l100_raw(vehicle, speed_kmh) * distance_km / 100, 2)


def _calc_l100_raw(vehicle: Vehicle, speed_kmh: float) -> float:
    """返回 L/100km 的精确值（不取整，供内部累加使用）"""
    gear = vehicle.select_gear(speed_kmh)
    if gear == 0:
        return 0.0
    speed_ms = speed_kmh * KMH_TO_MS
    gear_ratio = vehicle.gear_ratios[gear - 1]
    total_ratio = gear_ratio * vehicle.final_drive
    wheel_rps = speed_ms / (2 * math.pi * vehicle.wheel_radius)
    engine_rpm = wheel_rps * total_ratio * SECONDS_PER_MINUTE
    resistance = calc_resistance(vehicle, speed_ms)
    wheel_torque = resistance * vehicle.wheel_radius
    engine_torque = wheel_torque / (total_ratio * vehicle.trans_efficiency)
    load_ratio = max(0.01, min(1.0, engine_torque / vehicle.max_torque))
    bsfc = _interpolate_bsfc(engine_rpm, load_ratio, vehicle.fuel_type)
    engine_power_kw = engine_torque * engine_rpm * 2 * math.pi / SECONDS_PER_MINUTE / 1000
    fuel_mass_rate = bsfc * engine_power_kw / SECONDS_PER_HOUR  # g/s
    fuel_vol_rate = fuel_mass_rate / vehicle.fuel_density  # L/s
    return fuel_vol_rate * (SECONDS_PER_HOUR * 100 / speed_kmh)  # L/100km


def calc_fuel_table() -> list[dict]:
    """BSFC 模型：各车速下档位、转速、负荷和油耗，返回结构化数据。

    Returns:
        list[dict]: 每个 (车型, 车速) 组合的油耗明细
    """
    cars = [car_sedan, car_suv, car_truck]
    speeds = [20, 30, 50, 70, 90, 110, 120]

    results = []
    for car in cars:
        for v in speeds:
            gear = car.select_gear(v)
            speed_ms = v * KMH_TO_MS
            gear_ratio = car.gear_ratios[gear - 1] if gear > 0 else 0
            total_ratio = gear_ratio * car.final_drive

            if gear > 0:
                wheel_rps = speed_ms / (2 * math.pi * car.wheel_radius)
                engine_rpm = wheel_rps * total_ratio * SECONDS_PER_MINUTE
                resistance = calc_resistance(car, speed_ms)
                engine_torque = resistance * car.wheel_radius / (total_ratio * car.trans_efficiency)
                load_ratio = min(1.0, engine_torque / car.max_torque)
                bsfc = _interpolate_bsfc(engine_rpm, max(0.01, min(1.0, load_ratio)), car.fuel_type)
                # 直接用已算出的 bsfc/转速/扭矩 求 L/100km，避免 _calc_l100_raw 重复查表
                engine_power_kw = engine_torque * engine_rpm * 2 * math.pi / 60 / 1000
                fuel_mass_rate = bsfc * engine_power_kw / SECONDS_PER_HOUR  # g/s
                fuel_vol_rate = fuel_mass_rate / car.fuel_density  # L/s
                l100 = fuel_vol_rate * (SECONDS_PER_HOUR * 100 / v)  # L/100km
            else:
                engine_rpm, bsfc, l100 = car.idle_rpm, 580, 0
                load_ratio = 0

            results.append({
                "car_name": car.name,
                "speed_kmh": v,
                "gear": gear,
                "rpm": round(engine_rpm),
                "load_pct": round(load_ratio, 4),
                "bsfc_gkwh": round(bsfc),
                "l100km": round(l100, 1),
            })

    return results
