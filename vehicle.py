# -*- coding: utf-8 -*-
"""
车辆基本参数 + 行驶阻力 + 加速/制动/跟车
"""

from __future__ import annotations

import math

from _constants import G, RHO_AIR, KMH_TO_MS, MS_TO_KMH, DEFAULT_ROLLING_COEFF, DEFAULT_CG_FRONT_RATIO


# ---- 发动机外特性扭矩曲线 ----

# 归一化扭矩曲线（以最大扭矩为 1.0），参考典型 2.0L NA 汽油机
_NORMALIZED_TORQUE = {
    800: 0.30, 1000: 0.50, 1500: 0.70, 2000: 0.86,
    2500: 0.93, 3000: 0.97, 3500: 1.00, 4000: 0.99,
    4500: 0.95, 5000: 0.88, 5500: 0.78, 6000: 0.67,
}


def _make_default_torque_curve(max_torque_nm: float, idle_rpm: float = 800,
                                max_rpm: float = 6000) -> dict[int, float]:
    """从归一化曲线 + 最大扭矩生成外特性扭矩曲线 (rpm → Nm)。"""
    curve = {}
    for rpm, ratio in _NORMALIZED_TORQUE.items():
        if idle_rpm <= rpm <= max_rpm:
            curve[rpm] = round(ratio * max_torque_nm, 1)
    # 确保怠速和红线在曲线里
    if idle_rpm not in curve:
        curve[int(idle_rpm)] = round(0.30 * max_torque_nm, 1)
    if max_rpm not in curve:
        curve[int(max_rpm)] = round(0.67 * max_torque_nm, 1)
    return dict(sorted(curve.items()))


def _interp_torque_curve(rpm: float, curve: dict[int, float]) -> float:
    """在扭矩曲线中线性插值，返回对应转速的扭矩 (Nm)。"""
    rpms = list(curve.keys())
    if rpm <= rpms[0]:
        return curve[rpms[0]]
    if rpm >= rpms[-1]:
        return curve[rpms[-1]]
    for i in range(len(rpms) - 1):
        if rpms[i] <= rpm <= rpms[i + 1]:
            t = (rpm - rpms[i]) / (rpms[i + 1] - rpms[i])
            return curve[rpms[i]] + t * (curve[rpms[i + 1]] - curve[rpms[i]])
    return curve[rpms[-1]]  # fallback


def get_engine_torque(rpm: float, throttle: float,
                      torque_curve: dict[int, float]) -> float:
    """返回发动机在当前转速和油门开度下的输出扭矩 (Nm)。

    Args:
        rpm:          发动机转速
        throttle:     油门开度 0~1
        torque_curve: 外特性扭矩曲线 {rpm: torque_Nm}
    """
    rpms = list(torque_curve.keys())
    rpm = max(rpms[0], min(rpm, rpms[-1]))
    max_tq = _interp_torque_curve(rpm, torque_curve)
    return max(0.0, throttle * max_tq)


class Vehicle:
    """一辆车的物理参数 + 动力总成参数"""

    def __init__(self,
                 name: str,
                 mass_kg: float,
                 power_kw: float,
                 drag_coeff: float = 0.3,
                 frontal_area_m2: float = 2.2,
                 max_torque_nm: float = 180,
                 idle_rpm: float = 800,
                 max_rpm: float = 6000,
                 gear_ratios: list[float] | None = None,
                 final_drive: float = 4.0,
                 wheel_radius_m: float = 0.32,
                 trans_efficiency: float = 0.90,
                 fuel_density_gl: float = 740,
                 fuel_type: str = "gasoline",
                 torque_curve: dict[int, float] | None = None,
                 # 横向动力学参数
                 wheelbase_m: float | None = None,
                 cg_to_front_m: float | None = None,
                 cornering_stiffness_f: float | None = None,
                 cornering_stiffness_r: float | None = None,
                 yaw_inertia: float | None = None,
                 # Pacejka 魔术公式轮胎参数（前轴）
                 pacejka_B_f: float | None = None,
                 pacejka_C_f: float | None = None,
                 pacejka_D_f: float | None = None,
                 pacejka_E_f: float | None = None,
                 # Pacejka 魔术公式轮胎参数（后轴）
                 pacejka_B_r: float | None = None,
                 pacejka_C_r: float | None = None,
                 pacejka_D_r: float | None = None,
                 pacejka_E_r: float | None = None):
        self.name: str = name
        self.mass: float = mass_kg
        self.power: float = power_kw * 1000          # 发动机功率（W）
        self.cd: float = drag_coeff
        self.area: float = frontal_area_m2
        self.rolling_coeff: float = DEFAULT_ROLLING_COEFF
        # 动力总成参数
        self.max_torque: float = max_torque_nm       # 发动机最大扭矩（Nm）
        self.idle_rpm: float = idle_rpm
        self.max_rpm: float = max_rpm
        self.gear_ratios: list[float] = gear_ratios or [3.55, 2.11, 1.42, 1.00, 0.78]
        self.final_drive: float = final_drive        # 主减速比
        self.wheel_radius: float = wheel_radius_m    # 轮胎滚动半径（m）
        self.trans_efficiency: float = trans_efficiency  # 传动效率
        self.fuel_density: float = fuel_density_gl   # 燃油密度（g/L），汽油 740，柴油 840
        self.fuel_type: str = fuel_type
        # 发动机外特性扭矩曲线 {rpm: Nm}，未提供时根据 max_torque 自动生成
        self.torque_curve: dict[int, float] = (
            torque_curve or
            _make_default_torque_curve(max_torque_nm, idle_rpm, max_rpm)
        )
        # 横向动力学参数
        self.wheelbase: float = wheelbase_m or 2.65          # 轴距（m），典型轿车
        self.cg_to_front: float = cg_to_front_m or self.wheelbase * DEFAULT_CG_FRONT_RATIO  # 质心到前轴距离（m）
        self.cg_to_rear: float = self.wheelbase - self.cg_to_front         # 质心到后轴距离（m）
        # 侧偏刚度 magnitude（N/rad），存储正值。侧向力公式 Fy = -Cα × α，负号在 calc_cornering_forces 中体现
        self.cornering_stiffness_f: float = cornering_stiffness_f or 80000
        self.cornering_stiffness_r: float = cornering_stiffness_r or 70000
        # 横摆转动惯量（kg·m²），估算公式 Iz ≈ m × a × b
        self.yaw_inertia: float = yaw_inertia or self.mass * self.cg_to_front * self.cg_to_rear
        # ---- Pacejka 魔术公式轮胎参数 ----
        # D = 峰值侧向力 ≈ 轴荷 (N/rad)；B·C·D = cornering_stiffness，确保小侧偏角时与线性模型一致
        # 估计前后轴荷
        _Wf_axle = self.mass * G * self.cg_to_rear / self.wheelbase
        _Wr_axle = self.mass * G * self.cg_to_front / self.wheelbase
        self.pacejka_C_f: float = pacejka_C_f or 1.3
        self.pacejka_D_f: float = pacejka_D_f or _Wf_axle
        self.pacejka_B_f: float = pacejka_B_f or (self.cornering_stiffness_f / (self.pacejka_C_f * self.pacejka_D_f))
        self.pacejka_E_f: float = pacejka_E_f or 0.0
        self.pacejka_C_r: float = pacejka_C_r or 1.3
        self.pacejka_D_r: float = pacejka_D_r or _Wr_axle
        self.pacejka_B_r: float = pacejka_B_r or (self.cornering_stiffness_r / (self.pacejka_C_r * self.pacejka_D_r))
        self.pacejka_E_r: float = pacejka_E_r or 0.0

    def select_gear(self, speed_kmh: float) -> int:
        """根据车速选择合适档位（简化的经济性换挡策略）"""
        if speed_kmh <= 0:
            return 0
        speed_ms = speed_kmh * KMH_TO_MS
        # 目标发动机转速 1500-2500 RPM 为经济区间
        target_rpm = 2000
        best_gear = 1
        best_diff = float("inf")
        for g, ratio in enumerate(self.gear_ratios, start=1):
            rpm = speed_ms * ratio * self.final_drive / (2 * math.pi * self.wheel_radius) * 60
            # 不能低于怠速，不能高于红线
            if rpm < self.idle_rpm or rpm > self.max_rpm:
                continue
            diff = abs(rpm - target_rpm)
            if diff < best_diff:
                best_diff = diff
                best_gear = g
        return best_gear

    def __repr__(self) -> str:
        return f"{self.name} ({self.mass}kg, {self.power/1000:.0f}kW)"


# 三辆预置车辆的懒加载缓存（避免 import 时的参数计算开销）
_VEHICLE_CACHE: dict[str, Vehicle] = {}


def _init_default_vehicles():
    """首次访问时创建三辆预置 Vehicle 实例。"""
    global _VEHICLE_CACHE
    _VEHICLE_CACHE["car_sedan"] = Vehicle(
        "普通轿车", 1500, 100, drag_coeff=0.28,
        max_torque_nm=180, idle_rpm=800, max_rpm=6200,
        gear_ratios=[3.55, 2.11, 1.42, 1.00, 0.78],
        final_drive=4.06, wheel_radius_m=0.32, trans_efficiency=0.90,
        fuel_density_gl=740, fuel_type="gasoline",
        wheelbase_m=2.65, cg_to_front_m=1.2,
        cornering_stiffness_f=80000, cornering_stiffness_r=70000,
    )
    _VEHICLE_CACHE["car_suv"] = Vehicle(
        "SUV", 2000, 140, drag_coeff=0.35, frontal_area_m2=2.7,
        max_torque_nm=250, idle_rpm=700, max_rpm=6000,
        gear_ratios=[3.83, 2.36, 1.55, 1.00, 0.79],
        final_drive=3.89, wheel_radius_m=0.36, trans_efficiency=0.88,
        fuel_density_gl=740, fuel_type="gasoline",
        wheelbase_m=2.75, cg_to_front_m=1.3,
        cornering_stiffness_f=90000, cornering_stiffness_r=75000,
    )
    _VEHICLE_CACHE["car_truck"] = Vehicle(
        "重型卡车", 15000, 300, drag_coeff=0.65, frontal_area_m2=7.0,
        max_torque_nm=1000, idle_rpm=600, max_rpm=4000,
        gear_ratios=[5.50, 3.20, 1.90, 1.00, 0.73],
        final_drive=4.30, wheel_radius_m=0.52, trans_efficiency=0.85,
        fuel_density_gl=840, fuel_type="diesel",
        wheelbase_m=5.0, cg_to_front_m=2.5,
        cornering_stiffness_f=200000, cornering_stiffness_r=180000,
    )


def get_default_vehicles() -> dict[str, Vehicle]:
    """返回三辆预置车辆 dict，首次调用时创建。"""
    if not _VEHICLE_CACHE:
        _init_default_vehicles()
    return dict(_VEHICLE_CACHE)


# 模块级类型声明（供 IDE/类型检查器识别），运行时通过 __getattr__ 懒加载
car_sedan: Vehicle
car_suv: Vehicle
car_truck: Vehicle


def __getattr__(name: str):
    """模块级懒加载：首次访问 car_sedan/car_suv/car_truck 时才创建实例。"""
    if name in ("car_sedan", "car_suv", "car_truck"):
        if not _VEHICLE_CACHE:
            _init_default_vehicles()
        return _VEHICLE_CACHE[name]
    raise AttributeError(f"module 'vehicle' has no attribute '{name}'")


def rolling_coeff_dynamic(speed_ms: float) -> float:
    """SAE J2263 滑行阻力: μ(v) = f₀ + f₁·(v/100) + f₄·(v/100)⁴"""
    v = speed_ms * MS_TO_KMH            # m/s → km/h
    f0 = 0.010                    # 截距项
    f1 = 0.005                    # 速度一次项
    f4 = 0.002                    # 速度四次项
    return f0 + f1 * (v / 100) + f4 * (v / 100) ** 4


def calc_resistance(vehicle: Vehicle, speed_ms: float, dynamic_rr: bool = False) -> float:
    """计算总阻力(N): 滚动阻力 + 空气阻力, dynamic_rr 切换 SAE J2263 动态模型"""
    if dynamic_rr:
        coeff = rolling_coeff_dynamic(speed_ms)
    else:
        coeff = vehicle.rolling_coeff

    rolling = coeff * vehicle.mass * G

    # 空气阻力：F = 0.5 × ρ × Cd × A × v²
    aero = 0.5 * RHO_AIR * vehicle.cd * vehicle.area * speed_ms ** 2

    return rolling + aero


def calc_wheel_force(vehicle: Vehicle, speed_ms: float,
                     throttle: float = 1.0,
                     gear_override: int = 0) -> float:
    """计算轮端驱动力 (N)：发动机扭矩 → 变速箱 → 主减速器 → 车轮。

    Args:
        vehicle:      车辆对象
        speed_ms:     当前车速 (m/s)
        throttle:     油门开度 0~1，默认 1.0（全油门）
        gear_override: 强制档位，0 = 自动选档
    """
    if speed_ms <= 0.1:
        return 0.0

    speed_kmh = speed_ms * MS_TO_KMH
    gear = gear_override if gear_override > 0 else vehicle.select_gear(speed_kmh)
    if gear == 0:
        return 0.0

    gear_ratio = vehicle.gear_ratios[gear - 1]
    total_ratio = gear_ratio * vehicle.final_drive

    # 发动机转速
    wheel_rps = speed_ms / (2 * math.pi * vehicle.wheel_radius)
    engine_rpm = wheel_rps * total_ratio * 60

    # 扭矩查曲线
    engine_torque = get_engine_torque(engine_rpm, throttle, vehicle.torque_curve)

    # 轮端扭矩和驱动力
    wheel_torque = engine_torque * total_ratio * vehicle.trans_efficiency
    return wheel_torque / vehicle.wheel_radius


def calc_acceleration(vehicle: Vehicle, speed_ms: float,
                      throttle: float = 1.0,
                      gear_override: int = 0) -> float:
    """计算车辆在当前速度下的加速度（m/s²）。

    基于发动机扭矩曲线 + 变速箱速比，替代原来简化的 P=Fv 模型。
    """
    resistance = calc_resistance(vehicle, speed_ms)
    wheel_force = calc_wheel_force(vehicle, speed_ms, throttle, gear_override)
    net_force = wheel_force - resistance
    return max(0.0, net_force / vehicle.mass)


def simulate_acceleration(vehicle: Vehicle, target_speed_kmh: float = 100,
                          dt: float = 0.1) -> dict:
    """模拟车辆从 0 全油门加速到目标速度，含自动换挡。"""
    target = target_speed_kmh * KMH_TO_MS
    speed = 1.5  # m/s，~5 km/h 起步
    distance = 0.0
    time_elapsed = 0.0
    gear = 1
    shift_rpm = vehicle.max_rpm * 0.92  # 92% 红线换挡

    time_series: list[float] = []
    speed_series: list[float] = []
    acc_series: list[float] = []
    dist_series: list[float] = []

    while speed < target and time_elapsed < 120:
        # 计算当前档位下的发动机转速
        total_ratio = vehicle.gear_ratios[gear - 1] * vehicle.final_drive
        wheel_rps = speed / (2 * math.pi * vehicle.wheel_radius)
        engine_rpm = wheel_rps * total_ratio * 60

        # 到达换挡转速且还有更高档位 → 升档
        if engine_rpm >= shift_rpm and gear < len(vehicle.gear_ratios):
            gear += 1

        acc = calc_acceleration(vehicle, speed, throttle=1.0, gear_override=gear)
        speed += acc * dt
        distance += speed * dt
        time_elapsed += dt

        time_series.append(round(time_elapsed, 2))
        speed_series.append(round(speed * MS_TO_KMH, 2))
        acc_series.append(round(acc, 4))
        dist_series.append(round(distance, 2))

    return {
        "time": time_series,
        "speed_kmh": speed_series,
        "acc_ms2": acc_series,
        "distance_m": dist_series,
        "elapsed_s": round(time_elapsed, 1),
        "total_dist_m": round(distance, 1),
    }


def calc_braking_distance(speed_kmh: float, friction_coeff: float = 0.7,
                          reaction_time: float = 1.5) -> tuple[float, float, float]:
    """计算制动总距离 = 反应距离 + 制动距离"""
    speed_ms = speed_kmh * KMH_TO_MS

    # 反应距离 = 速度 × 反应时间
    reaction_dist = speed_ms * reaction_time

    # 制动距离 = v² / (2 × μ × g)
    braking_dist = speed_ms ** 2 / (2 * friction_coeff * G)

    total = reaction_dist + braking_dist
    return reaction_dist, braking_dist, total


def calc_grade_power(vehicle: Vehicle, speed_ms: float, grade_percent: float = 5) -> float:
    """爬坡功率: m·g·sin(θ)·v (W)"""
    grade_rad = math.atan(grade_percent / 100)
    grade_force = vehicle.mass * G * math.sin(grade_rad)
    return grade_force * speed_ms


def calc_power_to_weight(vehicle: Vehicle) -> tuple[float, float]:
    """比功率 = 发动机最大功率 / 整车质量 (W/kg)"""
    watt_per_kg = vehicle.power / vehicle.mass
    kw_per_ton = watt_per_kg  # W/kg == kW/ton（因为 1 kW / 1000 kg = 1 W/kg）
    return watt_per_kg, kw_per_ton


def calc_aero_drag_power(vehicle: Vehicle, speed_ms: float) -> float:
    """风阻功率 0.5·ρ·Cd·A·v³ (W)"""
    aero_force = 0.5 * RHO_AIR * vehicle.cd * vehicle.area * speed_ms ** 2
    return aero_force * speed_ms


def calc_power_breakdown(vehicle: Vehicle, speed_kmh: float = 100,
                         grade_percent: float = 5) -> dict:
    """车辆在指定工况下的各功率分解，返回结构化数据。"""
    speed_ms = speed_kmh * KMH_TO_MS

    rolling_power_const = vehicle.rolling_coeff * vehicle.mass * G * speed_ms
    rolling_power_dyn = rolling_coeff_dynamic(speed_ms) * vehicle.mass * G * speed_ms
    aero_power = calc_aero_drag_power(vehicle, speed_ms)
    grade_power = calc_grade_power(vehicle, speed_ms, grade_percent)
    total_resistance_power = rolling_power_const + aero_power + grade_power
    total_dyn = rolling_power_dyn + aero_power + grade_power
    wpk, kpt = calc_power_to_weight(vehicle)

    return {
        "vehicle_name": vehicle.name,
        "speed_kmh": speed_kmh,
        "grade_percent": grade_percent,
        "engine_max_power_w": vehicle.power,
        "power_to_weight_wpk": wpk,
        "power_to_weight_kpt": kpt,
        "rolling_power_const_w": rolling_power_const,
        "rolling_power_dyn_w": rolling_power_dyn,
        "aero_power_w": aero_power,
        "grade_power_w": grade_power,
        "total_power_const_w": total_resistance_power,
        "total_power_dyn_w": total_dyn,
        "power_utilization_pct": total_resistance_power / vehicle.power * 100,
    }


def calc_braking_table() -> list[dict]:
    """计算不同车速下的制动距离，返回结构化数据。"""
    results: list[dict] = []
    for v in [30, 50, 60, 80, 100, 120]:
        rd, bd, td = calc_braking_distance(v)
        results.append({
            "speed_kmh": v,
            "reaction_dist_m": round(rd, 1),
            "braking_dist_m": round(bd, 1),
            "total_dist_m": round(td, 1),
        })
    return results


def idm_acceleration(v_ego: float, v_leader: float, gap: float,
                     v0: float | None = None,
                     T: float = 1.5, s0: float = 2.0,
                     a: float = 1.4, b: float = 2.0,
                     delta: int = 4) -> float:
    """IDM (Intelligent Driver Model) 跟车加速度。

    a_idm = a * [1 - (v/v0)^δ - (s*(v, Δv) / s)²]

    其中 s*(v, Δv) = s0 + v·T + v·Δv / (2·√(a·b))

    Args:
        v_ego:    自车速度 (m/s)
        v_leader: 前车速度 (m/s)
        gap:      实际间距 (m)
        v0:       期望速度 (m/s)，默认取 v_leader（跟车模式）
        T:        安全时距 (s)
        s0:       最小停车间距 (m)
        a:        最大加速度 (m/s²)
        b:        舒适减速度 (m/s²，正值)
        delta:    加速度指数

    Returns:
        float: 加速度 (m/s²)，正值加速、负值减速
    """
    if v0 is None:
        v0 = v_leader  # 跟车模式下期望速度 = 前车速度

    if v_ego <= 0 and v0 <= 0:
        return 0.0

    v_ego = max(v_ego, 0.01)  # 避免除零
    dv = v_ego - v_leader      # 速度差，正值 = 自车更快（接近前车）

    # 期望安全间距
    s_star = s0 + max(0, v_ego * T + v_ego * dv / (2 * math.sqrt(a * b)))

    # 自由加速项
    free_road = 1.0 - (v_ego / max(v0, 0.1)) ** delta

    # 交互制动项
    interaction = (s_star / max(gap, 0.1)) ** 2

    return a * (free_road - interaction)


def car_following_simulation(lead_speed_kmh: float = 60,
                             follower_speed_kmh: float = 70,
                             initial_gap_m: float = 30,
                             duration_s: float = 30,
                             dt: float = 0.1) -> dict:
    """IDM 跟车仿真：前车匀速，后车用 IDM 跟随。

    Returns:
        dict: {time, gap_m, follower_kmh, leader_kmh, acc_ms2, status, collision_s}
    """
    lead_speed = lead_speed_kmh * KMH_TO_MS
    follower_speed = follower_speed_kmh * KMH_TO_MS

    lead_pos = 0.0
    follower_pos = -initial_gap_m
    gap = initial_gap_m

    time_series: list[float] = []
    gap_series: list[float] = []
    speed_series: list[float] = []
    acc_series: list[float] = []
    status_series: list[str] = []
    collision_time: float | None = None

    t = 0.0
    while t <= duration_s:
        # IDM 加速度
        acc = idm_acceleration(follower_speed, lead_speed, gap,
                               v0=lead_speed, T=1.5, s0=2.0, a=1.4, b=2.0)

        # 欧拉积分更新
        follower_speed = max(0.0, follower_speed + acc * dt)
        lead_pos += lead_speed * dt
        follower_pos += follower_speed * dt
        gap = lead_pos - follower_pos

        # 状态判定
        if gap > 15:
            status = "安全"
        elif gap > 5:
            status = "警告"
        else:
            status = "危险！"

        time_series.append(round(t, 2))
        gap_series.append(round(gap, 1))
        speed_series.append(round(follower_speed * MS_TO_KMH, 1))
        acc_series.append(round(acc, 3))
        status_series.append(status)

        if gap <= 0 and collision_time is None:
            collision_time = round(t, 2)

        t += dt

    return {
        "time": time_series,
        "gap_m": gap_series,
        "follower_kmh": speed_series,
        "leader_kmh": lead_speed_kmh,
        "acc_ms2": acc_series,
        "status": status_series,
        "collision_s": collision_time,
    }


def acc_simulation(lead_profile: list[tuple[float, float]] | None = None,
                   follower_v0_kmh: float = 50,
                   initial_gap_m: float = 40,
                   dt: float = 0.1) -> dict:
    """ACC 场景仿真：前车做变速工况，后车用 IDM 自适应巡航。

    Args:
        lead_profile: 前车速度曲线 [(时间s, 速度km/h), ...]，默认：加速→巡航→减速
        follower_v0_kmh: 后车初始速度 (km/h)
        initial_gap_m: 初始间距 (m)

    Returns:
        dict: {time, gap_m, follower_kmh, leader_kmh, acc_ms2}
    """
    if lead_profile is None:
        # 默认工况：前车 0→80 加速，80 巡航，80→0 减速
        lead_profile = [
            (0, 0), (5, 40), (10, 80), (20, 80), (30, 40), (35, 0),
        ]

    # 构建前车逐秒速度曲线（线性插值）
    total_time = lead_profile[-1][0]
    steps = int(total_time / dt)
    lead_speeds: list[float] = []
    seg_idx = 0
    for i in range(steps + 1):
        sim_time = i * dt
        while seg_idx < len(lead_profile) - 2 and sim_time > lead_profile[seg_idx + 1][0]:
            seg_idx += 1
        t0, v0 = lead_profile[seg_idx]
        t1, v1 = lead_profile[seg_idx + 1]
        duration = t1 - t0
        ratio = (sim_time - t0) / duration if duration > 0 else 1.0
        lead_speeds.append(max(0, v0 + (v1 - v0) * ratio) * KMH_TO_MS)

    follower_speed = follower_v0_kmh * KMH_TO_MS
    lead_pos = 0.0
    follower_pos = -initial_gap_m

    time_series: list[float] = []
    gap_series: list[float] = []
    follower_spd: list[float] = []
    leader_spd: list[float] = []
    acc_series: list[float] = []

    for i in range(steps + 1):
        sim_time = i * dt
        lead_v = lead_speeds[i]

        # 前车期望速度（IDM 用前车当前速度作为 v0，模拟跟车）
        gap = lead_pos - follower_pos
        # 如果前车太远（gap > 100m），切换到自由巡航模式
        v_desired = lead_v if gap < 100 else follower_v0_kmh * KMH_TO_MS
        acc = idm_acceleration(follower_speed, lead_v, gap,
                               v0=v_desired, T=1.5, s0=2.0, a=1.4, b=2.0)

        follower_speed = max(0.0, follower_speed + acc * dt)
        lead_pos += lead_v * dt
        follower_pos += follower_speed * dt

        time_series.append(round(sim_time, 2))
        gap_series.append(round(lead_pos - follower_pos, 1))
        follower_spd.append(round(follower_speed * MS_TO_KMH, 1))
        leader_spd.append(round(lead_v * MS_TO_KMH, 1))
        acc_series.append(round(acc, 3))

    return {
        "time": time_series,
        "gap_m": gap_series,
        "follower_kmh": follower_spd,
        "leader_kmh": leader_spd,
        "acc_ms2": acc_series,
    }
