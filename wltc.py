# -*- coding: utf-8 -*-
"""
WLTC 瞬态油耗仿真：加速加浓、减速断油(DFCO)、怠速油耗
"""

from __future__ import annotations

import math

from _constants import G, KMH_TO_MS, MS_TO_KMH, SECONDS_PER_HOUR, SECONDS_PER_MINUTE
from vehicle import Vehicle, calc_resistance, get_engine_torque
from bsfc import _interpolate_bsfc, _calc_l100_raw, calc_fuel_consumption


# WLTC Class 3 关键拐点，线性插值生成 1Hz 速度曲线
_WLTC_WAYPOINTS = [
    # Phase 1: Low
    (0,0),(11,0),(15,12.7),(23,12.7),(28,0),(33,0),(38,18.3),(48,18.3),(51,0),
    (56,0),(61,24.1),(66,24.1),(71,0),(76,0),(81,28.5),(96,28.5),(101,12.5),
    (106,0),(111,0),(116,32.5),(121,0),(126,0),(131,32.2),(141,32.2),(146,0),
    (151,0),(156,35.8),(176,35.8),(181,27.6),(186,12.6),(191,0),(196,0),
    (201,35.6),(216,35.6),(221,0),(226,0),(231,38.4),(251,38.4),(256,29.8),
    (261,19.7),(266,0),(271,0),(276,40.3),(296,40.3),(301,32.1),(306,22.1),
    (311,0),(316,0),(321,41.2),(341,41.2),(346,33.0),(351,25.1),(356,0),
    (361,0),(366,41.8),(381,41.8),(386,31.4),(391,21.2),(396,0),(401,0),
    (406,44.1),(416,44.1),(421,32.9),(426,21.5),(431,0),(436,0),(441,45.5),
    (456,45.5),(461,34.7),(466,23.1),(471,0),(476,0),(481,47.2),(496,47.2),
    (501,35.8),(506,24.3),(511,0),(516,0),(521,50.9),(531,50.9),(536,37.9),
    (541,26.3),(546,0),(551,0),(556,52.4),(571,52.4),(576,40.0),(581,29.1),
    (586,0),
    # Phase 2: Medium
    (590,0),(593,0),(600,49.0),(615,49.0),(620,38.5),(625,28.1),(630,0),
    (635,0),(642,52.1),(652,52.1),(657,41.3),(662,31.1),(667,0),(672,0),
    (679,55.4),(694,55.4),(699,44.0),(704,33.8),(709,0),(714,0),(721,58.6),
    (736,58.6),(741,46.0),(746,35.4),(751,0),(756,0),(763,61.0),(783,61.0),
    (788,48.5),(793,37.6),(798,0),(803,0),(810,63.5),(830,63.5),(835,50.1),
    (840,39.5),(845,0),(850,0),(857,65.4),(877,65.4),(882,52.3),(887,41.1),
    (892,0),(897,0),(904,67.0),(924,67.0),(929,52.9),(934,42.0),(939,0),
    (944,0),(951,68.8),(971,68.8),(976,54.2),(981,43.5),(986,0),(991,0),
    (998,70.3),(1018,70.3),(1022,0),
    # Phase 3: High
    (1023,0),(1026,0),(1035,75.0),(1055,75.0),(1060,60.1),(1065,48.1),
    (1070,36.0),(1075,0),(1080,0),(1089,71.0),(1109,71.0),(1114,56.2),
    (1119,44.8),(1124,33.2),(1129,0),(1134,0),(1143,77.5),(1163,77.5),
    (1168,61.0),(1173,49.0),(1178,36.8),(1183,0),(1188,0),(1197,83.0),
    (1217,83.0),(1222,65.8),(1227,53.1),(1232,40.1),(1237,0),(1242,0),
    (1251,88.5),(1271,88.5),(1276,69.9),(1281,56.8),(1286,43.2),(1291,0),
    (1296,0),(1305,92.0),(1325,92.0),(1330,73.3),(1335,59.0),(1340,45.2),
    (1345,0),(1350,0),(1359,95.5),(1379,95.5),(1384,76.0),(1389,61.2),
    (1394,46.8),(1399,0),(1404,0),(1413,97.0),(1433,97.0),(1438,77.0),
    (1443,62.3),(1448,48.0),(1453,0),(1458,0),(1467,97.4),(1477,97.4),
    # Phase 4: Extra High
    (1478,97.4),(1481,0),(1484,0),(1494,104.0),(1514,104.0),(1519,83.0),
    (1524,67.2),(1529,51.1),(1534,0),(1537,0),(1547,110.5),(1566,110.5),
    (1571,88.5),(1576,71.2),(1581,53.8),(1586,0),(1589,0),(1599,118.0),
    (1617,118.0),(1622,94.5),(1627,76.3),(1632,57.8),(1637,0),(1640,0),
    (1650,124.0),(1670,124.0),(1675,99.2),(1680,80.3),(1685,61.5),(1690,0),
    (1693,0),(1703,129.5),(1723,129.5),(1728,103.5),(1733,83.6),(1738,63.5),
    (1743,0),(1746,0),(1756,131.3),(1776,131.3),(1781,105.0),(1786,85.0),
    (1791,65.2),(1796,45.0),(1800,0),
]
_WLTC_DURATION = 1800  # 秒


def _driver_p_controller(vehicle: Vehicle, target_speed: float,
                         speed: float) -> tuple[float, float]:
    """P-controller 驾驶员模型：根据速度误差计算油门开度和制动力。

    三个区域：
      - 加速 (target > speed): throttle = kp_accel * error + base
      - 减速 (target < speed): brake = kp_brake * |error|
      - 巡航 (target ≈ speed): 维持平衡油门，抵消行驶阻力

    Returns:
        (throttle, brake) — 均在 [0, 1] 范围内
    """
    speed_error = target_speed - speed
    if speed_error > 0.1:
        # 加速
        throttle = min(1.0, 0.15 * speed_error + 0.05)
        brake = 0.0
    elif speed_error < -0.1:
        # 减速
        throttle = 0.0
        brake = min(1.0, -0.2 * speed_error)
    else:
        # 巡航或停车
        if target_speed < 0.5 and speed < 0.5:
            throttle = 0.0
            brake = 0.3 if speed > 0.05 else 0.0
        else:
            resistance = calc_resistance(vehicle, max(speed, 0.1))
            cruise_torque = resistance * vehicle.wheel_radius
            gear = vehicle.select_gear(speed * MS_TO_KMH)
            if gear > 0:
                total_ratio = vehicle.gear_ratios[gear - 1] * vehicle.final_drive
                engine_torque_needed = cruise_torque / (total_ratio * vehicle.trans_efficiency)
                throttle = min(1.0, engine_torque_needed / vehicle.max_torque + 0.02)
            else:
                throttle = 0.02
            brake = 0.0
    return throttle, brake


def get_wltc_profile() -> list[float]:
    """从关键拐点线性插值生成 WLTC 1Hz 速度曲线 (km/h), 返回 list[float] len=1801"""
    profile = [0.0] * (_WLTC_DURATION + 1)
    idx = 0
    for i in range(len(_WLTC_WAYPOINTS) - 1):
        t0, v0 = _WLTC_WAYPOINTS[i]
        t1, v1 = _WLTC_WAYPOINTS[i + 1]
        duration = t1 - t0
        for dt in range(duration + 1):
            if idx <= _WLTC_DURATION:
                ratio = dt / duration if duration > 0 else 1.0
                profile[idx] = round(v0 + (v1 - v0) * ratio, 1)
                idx += 1
    return profile


def get_wltc_summary() -> dict:
    """WLTC 工况概要，返回结构化数据。

    Returns:
        dict: {
            "phases":           list[dict]  各阶段概要,
            "total_distance_km": float,
            "profile":          list[float],
        }
    """
    profile = get_wltc_profile()
    phases = [
        ("Phase 1 (Low)",  0, 589),
        ("Phase 2 (Medium)",  590, 1022),
        ("Phase 3 (High)",  1023, 1477),
        ("Phase 4 (Extra High)",  1478, 1800),
    ]
    phase_list = []
    total_dist = 0
    for name, start, end in phases:
        seg = profile[start:end + 1]
        max_v = max(seg)
        dist_km = sum(v * KMH_TO_MS for v in seg) / 1000
        total_dist += dist_km
        phase_list.append({
            "name": name,
            "start_s": start,
            "end_s": end,
            "max_speed_kmh": max_v,
            "distance_km": round(dist_km, 2),
        })

    return {
        "phases": phase_list,
        "total_distance_km": round(total_dist, 2),
        "profile": profile,
    }


def simulate_transient_cycle(vehicle: Vehicle, cycle: list | None = None,
                             dt: float = 0.1, verbose: bool = True) -> tuple[float, float, float, float]:
    """瞬态油耗仿真：驾驶员模型(P控制) → 车辆动力学 → BSFC查表 → 油耗累计

    Args:
        verbose: 是否打印进度，默认 True（向后兼容）
    """
    if cycle is None:
        raise ValueError("cycle must be provided")
    total_time = sum(phase[1] for phase in cycle)
    steps = int(total_time / dt)

    # 车辆状态
    speed = 0.0          # m/s
    distance = 0.0       # m
    total_fuel_L = 0.0   # 累计油耗 (L)
    steady_fuel_L = 0.0  # 稳态估算累计 (假设瞬间到达目标速度并巡航)
    gear = 0
    engine_rpm = vehicle.idle_rpm

    # 构建逐秒目标车速序列
    targets = []
    for _, duration, target_kmh in cycle:
        targets.extend([target_kmh * KMH_TO_MS] * int(duration / dt))

    if verbose:
        print(f"\n{'='*95}")
        print(f"  瞬态油耗仿真 — {vehicle.name} — 简易城市工况 ({total_time}s)")
        print(f"{'='*95}")
        print(f"{'时间':>6}  {'目标':>5}  {'实际':>5}  {'油门':>5}  {'档位':>3}  "
              f"{'转速':>6}  {'负荷':>5}  {'BSFC':>5}  {'瞬态油耗':>8}  {'累计':>7}")
        print(f"{'s':>6}  {'km/h':>5}  {'km/h':>5}  {'%':>5}  {'':>3}  "
              f"{'rpm':>6}  {'%':>5}  {'g/kWh':>5}  {'L/100km':>8}  {'L':>7}")
        print("-" * 95)

    idx = 0
    last_print = -1.0
    steady_last_speed = -1  # 稳态估算只在新目标车速变化时计算一次

    for step in range(steps):
        sim_time = step * dt
        target_speed = targets[min(step, len(targets) - 1)]
        target_kmh = target_speed * MS_TO_KMH

        # ---- 驾驶员模型 (P 控制器) ----
        throttle, brake = _driver_p_controller(vehicle, target_speed, speed)

        # ---- 车辆动力学 ----
        if throttle > 0.05 and speed < 1:
            gear = 1  # 起步强制 1 档
        else:
            gear = vehicle.select_gear(speed * MS_TO_KMH)
        if gear > 0:
            total_ratio = vehicle.gear_ratios[gear - 1] * vehicle.final_drive
            engine_rpm = max(vehicle.idle_rpm,
                             speed / (2 * math.pi * vehicle.wheel_radius) * total_ratio * 60)
            resistance = calc_resistance(vehicle, max(speed, 0.1))
            engine_torque = get_engine_torque(engine_rpm, throttle, vehicle.torque_curve)
            wheel_torque = engine_torque * total_ratio * vehicle.trans_efficiency
            wheel_force = wheel_torque / vehicle.wheel_radius

            # 制动力
            if brake > 0:
                brake_force = brake * vehicle.mass * G * 0.8  # 最大减速度 0.8g
                wheel_force -= brake_force

            net_force = wheel_force - resistance
            acceleration = net_force / vehicle.mass
        else:
            engine_rpm = vehicle.idle_rpm
            acceleration = 0
            if brake > 0:
                acceleration = -brake * G * 0.8

        # 更新速度
        speed = max(0, speed + acceleration * dt)
        distance += speed * dt

        # ---- 瞬态油耗计算 ----
        if gear > 0 and throttle > 0.01:
            load_ratio = min(1.0, engine_torque / vehicle.max_torque)
            bsfc = _interpolate_bsfc(engine_rpm, max(0.01, load_ratio), vehicle.fuel_type)

            # 加速加浓修正：加速度越大，喷油越浓
            if acceleration > 0.1:
                enrich_factor = 1.0 + min(0.35, acceleration * 0.5)
                bsfc *= enrich_factor

            engine_power_kw = engine_torque * engine_rpm * 2 * math.pi / 60 / 1000
            fuel_mass_rate = bsfc * engine_power_kw / SECONDS_PER_HOUR  # g/s
            fuel_vol_rate = fuel_mass_rate / vehicle.fuel_density  # L/s
            total_fuel_L += fuel_vol_rate * dt

            # 瞬时百公里油耗
            if speed > 0.1:
                inst_l100 = fuel_vol_rate * (SECONDS_PER_HOUR * 100 / (speed * MS_TO_KMH))
            else:
                inst_l100 = 0
        else:
            # 减速断油 (DFCO)：收油门且转速高于怠速 → 断油
            if engine_rpm > vehicle.idle_rpm + 300 and throttle < 0.01 and speed > 1:
                bsfc = 0
                inst_l100 = 0
            else:
                # 怠速油耗
                bsfc = _interpolate_bsfc(vehicle.idle_rpm, 0.05, vehicle.fuel_type)
                idle_power = vehicle.idle_rpm * vehicle.max_torque * 0.05 * 2 * math.pi / 60 / 1000
                fuel_rate = bsfc * idle_power / SECONDS_PER_HOUR / vehicle.fuel_density
                total_fuel_L += fuel_rate * dt
                inst_l100 = 99.9 if speed < 0.5 else fuel_rate * (SECONDS_PER_HOUR * 100 / (speed * MS_TO_KMH))

        # ---- 稳态估算（仅目标车速变化时记录一次 BSFC 查表值） ----
        if abs(target_kmh - steady_last_speed) > 2:
            steady_fuel_L += calc_fuel_consumption(vehicle, target_kmh, 0)  # 先不做累计，只做参考
            steady_last_speed = target_kmh

        # ---- 每秒打印 ----
        if verbose and sim_time - last_print >= 1.0:
            print(f"{sim_time:5.0f}s  {target_kmh:4.0f}   {speed*3.6:4.0f}   "
                  f"{throttle*100:4.0f}%  {gear:>2}档  "
                  f"{engine_rpm:5.0f}  {engine_torque/vehicle.max_torque*100 if gear>0 and throttle>0.01 else 0:4.0f}%  "
                  f"{bsfc if gear>0 else 580:4.0f}  "
                  f"{inst_l100:7.1f}  {total_fuel_L:6.3f}")
            last_print = sim_time

    # 稳态估算：按每个阶段车速巡航的油耗求和
    steady_total = 0
    for phase_name, duration, target_kmh in cycle:
        if target_kmh > 0:
            seg_dist = target_kmh * KMH_TO_MS * duration / 1000  # km
            steady_total += calc_fuel_consumption(vehicle, target_kmh, seg_dist)

    avg_L100 = total_fuel_L / (distance / 100000) if distance > 0 else 0
    if verbose:
        print(f"\n结果: 总油耗 {total_fuel_L:.3f}L | 总里程 {distance:.0f}m | 平均 {avg_L100:.1f} L/100km")
        print(f"      稳态估算: {steady_total:.3f}L (仅算各阶段匀速巡航) | 瞬态比稳态多 {total_fuel_L-steady_total:.3f}L")
    return total_fuel_L, distance, avg_L100, steady_total


def simulate_wltc(vehicle: Vehicle, dt: float = 0.2,
                  verbose: bool = True) -> tuple[float, float, float, float]:
    """
    运行 WLTC Class 3 完整循环（1800 秒）并对比瞬态 vs 稳态油耗。

    因周期长达 1800s，每 30 秒打印一次状态快照，
    并对加速/巡航/减速/怠速阶段的燃油分配做分析。

    Args:
        verbose: 是否打印进度，默认 True（向后兼容）
    """

    def format_phase_desc(start_s, end_s, profile):
        if start_s >= len(profile):
            return "", 0
        seg = profile[start_s:min(end_s + 1, len(profile))]
        avg_v = sum(seg) / len(seg) if seg else 0
        max_v = max(seg) if seg else 0
        labels = [(589,"Low"),(1022,"Med"),(1477,"Hi"),(1800,"ExHi")]
        phase_name = next((n for t,n in labels if end_s <= t), "")
        return f"{phase_name}", max_v

    wltc = get_wltc_profile()
    total_steps = int(_WLTC_DURATION / dt)
    print_interval = 30  # 每 30 秒打印

    speed = 0.0
    distance = 0.0
    total_fuel_L = 0.0
    gear = 0
    engine_rpm = vehicle.idle_rpm

    # 分段统计
    phase_fuel = {"Low": 0.0, "Med": 0.0, "Hi": 0.0, "ExHi": 0.0}
    phase_dist = {"Low": 0.0, "Med": 0.0, "Hi": 0.0, "ExHi": 0.0}
    accel_fuel = cruise_fuel = decel_fuel = idle_fuel = 0.0

    if verbose:
        print(f"\n{'='*100}")
        print(f"  WLTC Class 3 瞬态仿真 — {vehicle.name} — 1800s 标准循环")
        print(f"{'='*100}")
        print(f"{'时间':>6}  {'目标':>5}  {'实际':>5}  {'油门':>5}  {'档位':>3}  "
              f"{'转速':>6}  {'BSFC':>5}  {'瞬时油耗':>8}  {'累计':>7}  {'阶段'}")
        print(f"{'s':>6}  {'km/h':>5}  {'km/h':>5}  {'%':>5}  {'':>3}  "
              f"{'rpm':>6}  {'g/kWh':>5}  {'L/100km':>8}  {'L':>7}")
        print("-" * 100)

    last_print = -print_interval
    throttle = 0.0
    brake = 0.0
    last_accel = 0.0

    for step in range(total_steps):
        sim_time = step * dt
        t_idx = int(sim_time)
        if t_idx >= len(wltc):
            break
        target_kmh = wltc[t_idx]
        target_speed = target_kmh * KMH_TO_MS

        # ---- 驾驶员模型 ----
        throttle, brake = _driver_p_controller(vehicle, target_speed, speed)

        # ---- 车辆动力学 ----
        if throttle > 0.05 and speed < 1:
            gear = 1
        else:
            gear = vehicle.select_gear(speed * MS_TO_KMH)

        if gear > 0:
            total_ratio = vehicle.gear_ratios[gear - 1] * vehicle.final_drive
            engine_rpm = max(vehicle.idle_rpm,
                             speed / (2 * math.pi * vehicle.wheel_radius) * total_ratio * 60)
            resistance = calc_resistance(vehicle, max(speed, 0.1))
            engine_torque = get_engine_torque(engine_rpm, throttle, vehicle.torque_curve)
            wheel_torque = engine_torque * total_ratio * vehicle.trans_efficiency
            wheel_force = wheel_torque / vehicle.wheel_radius
            if brake > 0:
                wheel_force -= brake * vehicle.mass * G * 0.8
            net_force = wheel_force - resistance
            acceleration = net_force / vehicle.mass
        else:
            engine_rpm = vehicle.idle_rpm
            acceleration = 0
            if brake > 0:
                acceleration = -brake * G * 0.8

        speed = max(0, speed + acceleration * dt)
        distance += speed * dt

        # ---- 瞬态油耗 ----
        inst_l100 = 0
        bsfc = 0
        if gear > 0 and throttle > 0.01:
            load_ratio = min(1.0, engine_torque / vehicle.max_torque)
            bsfc = _interpolate_bsfc(engine_rpm, max(0.01, load_ratio), vehicle.fuel_type)
            if acceleration > 0.1:
                bsfc *= 1.0 + min(0.35, acceleration * 0.5)
            power_kw = engine_torque * engine_rpm * 2 * math.pi / 60 / 1000
            fuel_rate = bsfc * power_kw / SECONDS_PER_HOUR / vehicle.fuel_density
            total_fuel_L += fuel_rate * dt
            if speed > 0.1:
                inst_l100 = fuel_rate * (SECONDS_PER_HOUR * 100 / (speed * MS_TO_KMH))

            # 工况分类统计
            if acceleration > 0.2:
                accel_fuel += fuel_rate * dt
            elif abs(speed_error) < 0.3:
                cruise_fuel += fuel_rate * dt
            else:
                decel_fuel += fuel_rate * dt  # 轻微减速但仍在供油
        else:
            if engine_rpm > vehicle.idle_rpm + 300 and throttle < 0.01 and speed > 1:
                bsfc = 0
                fuel_rate = 0.0
            else:
                bsfc = _interpolate_bsfc(vehicle.idle_rpm, 0.05, vehicle.fuel_type)
                idle_power = vehicle.idle_rpm * vehicle.max_torque * 0.05 * 2 * math.pi / 60 / 1000
                fuel_rate = bsfc * idle_power / SECONDS_PER_HOUR / vehicle.fuel_density
                total_fuel_L += fuel_rate * dt
                idle_fuel += fuel_rate * dt
                inst_l100 = 99.9 if speed < 0.5 else fuel_rate * (SECONDS_PER_HOUR * 100 / (speed * MS_TO_KMH))

        # 按阶段累计
        if t_idx < 590:
            phase_fuel["Low"] += fuel_rate * dt
            phase_dist["Low"] += speed * dt
        elif t_idx < 1023:
            phase_fuel["Med"] += fuel_rate * dt
            phase_dist["Med"] += speed * dt
        elif t_idx < 1478:
            phase_fuel["Hi"] += fuel_rate * dt
            phase_dist["Hi"] += speed * dt
        else:
            phase_fuel["ExHi"] += fuel_rate * dt
            phase_dist["ExHi"] += speed * dt

        last_accel = acceleration

        # 每 30 秒打印
        if verbose and sim_time - last_print >= print_interval:
            phase_label, _ = format_phase_desc(t_idx - 1, t_idx, wltc)
            print(f"{sim_time:5.0f}s  {target_kmh:4.0f}   {speed*3.6:4.0f}   "
                  f"{throttle*100:4.0f}%  {gear:>2}档  "
                  f"{engine_rpm:5.0f}  {bsfc:4.0f}  "
                  f"{inst_l100:7.1f}  {total_fuel_L:6.3f}  {phase_label}")
            last_print = sim_time

    # ---- 稳态估算（不经过 round，避免微距截断）----
    steady_total = 0
    for t in range(0, _WLTC_DURATION):
        v = wltc[t] if t < len(wltc) else 0
        if v > 0.5:
            l100 = _calc_l100_raw(vehicle, v)
            dist_km = v / SECONDS_PER_HOUR  # 1秒行驶的公里数
            steady_total += l100 * dist_km / 100

    avg_L100 = total_fuel_L / (distance / 100000) if distance > 0 else 0

    if verbose:
        print(f"\n{'='*100}")
        print(f"  WLTC 仿真结果")
        print(f"{'='*100}")
        print(f"  总油耗: {total_fuel_L:.3f}L  |  总里程: {distance/1000:.2f}km  |  平均: {avg_L100:.1f} L/100km")
        if steady_total > 0:
            print(f"  稳态估算: {steady_total:.3f}L  |  瞬态比稳态多 {total_fuel_L-steady_total:.3f}L ({(total_fuel_L/steady_total-1)*100:+.0f}%)")
        else:
            print(f"  稳态估算: {steady_total:.3f}L")
        print(f"\n  各阶段油耗:")
        phase_names = [("Low (0-589s)", "Low"), ("Med (590-1022s)", "Med"),
                       ("Hi (1023-1477s)", "Hi"), ("ExHi (1478-1800s)", "ExHi")]
        for label, key in phase_names:
            d = phase_dist[key] / 1000
            l100 = phase_fuel[key] / (d / 100) if d > 0 else 0
            print(f"    {label:<18} {phase_fuel[key]:.3f}L  {d:.2f}km  {l100:.1f} L/100km")
        print(f"\n  工况分配: 加速 {accel_fuel:.3f}L | 巡航 {cruise_fuel:.3f}L | 减速 {decel_fuel:.3f}L | 怠速 {idle_fuel:.3f}L")
    return total_fuel_L, distance, avg_L100, steady_total
