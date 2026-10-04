# -*- coding: utf-8 -*-
"""
横向动力学 —— 自行车模型（Bicycle Model）
从直线运动扩展到平面转向：侧偏角、横摆角速度、不足/过度转向
"""

from __future__ import annotations

import math

from .vehicle import G, KMH_TO_MS, _validate_time_grid
from .vehicle import Vehicle


def calc_slip_angles(vehicle: Vehicle, vx_ms: float, vy_ms: float,
                     yaw_rate: float, steer_angle_rad: float) -> tuple[float, float]:
    """前后轮侧偏角 αf = (vy+a·r)/vx - δ, αr = (vy-b·r)/vx (rad)"""
    if not math.isfinite(vx_ms) or vx_ms <= 0:
        raise ValueError("slip angles require a finite positive longitudinal speed")
    a = vehicle.cg_to_front
    b = vehicle.cg_to_rear
    alpha_f = (vy_ms + a * yaw_rate) / vx_ms - steer_angle_rad
    alpha_r = (vy_ms - b * yaw_rate) / vx_ms
    return alpha_f, alpha_r


def calc_cornering_forces(vehicle: Vehicle, alpha_f: float,
                          alpha_r: float) -> tuple[float, float]:
    """线性轮胎模型: Fy = -Cα × α (N)

    Cα 为侧偏刚度 magnitude（正值）。负号体现物理规律：侧偏角为正时侧向力为负（反向）。
    """
    Fyf = -vehicle.cornering_stiffness_f * alpha_f
    Fyr = -vehicle.cornering_stiffness_r * alpha_r
    return Fyf, Fyr


def calc_pacejka_lateral_force(B: float, C: float, D: float, E: float,
                                alpha: float) -> float:
    """Pacejka 魔术公式 — 纯侧偏工况轮胎侧向力 (N)。

    Fy(α) = D · sin(C · arctan(B·α - E·(B·α - arctan(B·α))))

    Args:
        B: 刚度因子，B·C·D = 小侧偏角下的侧偏刚度 (1/rad)
        C: 形状因子（典型值 1.1~1.5）
        D: 峰值因子 ≈ 轴荷 (N)
        E: 曲率因子，控制峰值附近的曲率
        alpha: 侧偏角 (rad)

    Returns:
        侧向力 (N)，正值表示与侧偏角同向。
        calc_slip_angles 已保证 α 正负含义正确，此处直接计算 magnitude。
    """
    Bx = B * alpha
    Fy = D * math.sin(C * math.atan(Bx - E * (Bx - math.atan(Bx))))
    return float(Fy)


def calc_pacejka_cornering_forces(vehicle: Vehicle, alpha_f: float,
                                   alpha_r: float) -> tuple[float, float]:
    """Pacejka 魔术公式轮胎模型: 前后轴侧向力 (N)。

    返回的 Fy 为带符号的侧向力：
    - 侧偏角 > 0（正侧向滑动）→ Fy > 0（力与 α 同向）
    - 公式本身输出 magnitude，这里根据 α 的符号赋予方向。
    """
    # Pacejka 公式的输入是侧偏角的绝对值，输出是力的大小
    # 侧向力方向与侧偏角同向（在 SAE 坐标系中）
    Fyf_mag = calc_pacejka_lateral_force(
        vehicle.pacejka_B_f, vehicle.pacejka_C_f,
        vehicle.pacejka_D_f, vehicle.pacejka_E_f, alpha_f)
    Fyr_mag = calc_pacejka_lateral_force(
        vehicle.pacejka_B_r, vehicle.pacejka_C_r,
        vehicle.pacejka_D_r, vehicle.pacejka_E_r, alpha_r)

    # 在 SAE 坐标系中：侧偏角为正时侧向力应为负（与转向方向相反）
    # 2-DOF 状态方程中 dvy = (Fyf+Fyr)/m - vx·r，Fy 取负正符合物理
    return -Fyf_mag, -Fyr_mag


def calc_understeer_gradient(vehicle: Vehicle) -> tuple[float, float]:
    """不足转向梯度 Kus = Wf/Cf - Wr/Cr (rad/g, deg/g)

    Cf, Cr 为侧偏刚度 magnitude（正值）。Kus > 0 = 不足转向，Kus < 0 = 过度转向。
    """
    m = vehicle.mass
    a = vehicle.cg_to_front
    b = vehicle.cg_to_rear
    L = vehicle.wheelbase
    Cf = vehicle.cornering_stiffness_f
    Cr = vehicle.cornering_stiffness_r

    Wf = m * G * b / L   # 前轴载荷（N）
    Wr = m * G * a / L   # 后轴载荷（N）

    kus_rad_per_g = Wf / Cf - Wr / Cr          # rad/g
    kus_deg_per_g = kus_rad_per_g * 180 / math.pi  # deg/g
    return kus_rad_per_g, kus_deg_per_g


def calc_characteristic_speed(vehicle: Vehicle) -> float:
    """计算不足转向特征车速（km/h），仅对不足转向有效"""
    _, kus_deg = calc_understeer_gradient(vehicle)
    if kus_deg <= 0:
        return float("inf")
    L = vehicle.wheelbase
    # v_char = sqrt(g·L / Kus)  where Kus in rad/g
    kus_rad, _ = calc_understeer_gradient(vehicle)
    v_char_ms = math.sqrt(G * L / kus_rad)
    return v_char_ms / KMH_TO_MS


def calc_critical_speed(vehicle: Vehicle) -> float:
    """计算过度转向临界车速（km/h），仅对过度转向有效

    超过此车速，车辆将失稳（横摆角速度趋于无穷）
    """
    _, kus_deg = calc_understeer_gradient(vehicle)
    if kus_deg >= 0:
        return float("inf")
    L = vehicle.wheelbase
    kus_rad, _ = calc_understeer_gradient(vehicle)
    v_crit_ms = math.sqrt(-G * L / kus_rad)
    return v_crit_ms / KMH_TO_MS


def calc_steady_state_cornering(vehicle: Vehicle, vx_kmh: float,
                                steer_angle_deg: float) -> dict:
    """稳态转向；半径为非负大小，转向方向由 yaw_rate 的符号表示。

    静止或直行时半径为 inf。过度转向临界速度及以上没有稳定稳态解。
    steer_angle_deg 是前轮转角，不是方向盘转角。
    """
    if not math.isfinite(vx_kmh) or vx_kmh < 0 or not math.isfinite(steer_angle_deg):
        raise ValueError("speed must be finite and non-negative; steer must be finite")
    vx = vx_kmh * KMH_TO_MS
    delta = math.radians(steer_angle_deg)
    L = vehicle.wheelbase
    kus_rad, kus_deg = calc_understeer_gradient(vehicle)

    # 稳态横摆角速度
    denominator = L + kus_rad * vx ** 2 / G
    if denominator <= 1e-12:
        raise ValueError("no stable steady state at or above critical speed")
    r = vx / denominator * delta  # rad/s

    # 侧向加速度
    ay = vx * r  # m/s²

    # 转弯半径
    curvature = r / vx if vx > 0 else 0.0  # 1/m
    radius = 1 / abs(curvature) if abs(curvature) > 1e-12 else float("inf")

    return {
        "speed_kmh": vx_kmh,
        "steer_deg": steer_angle_deg,
        "yaw_rate_deg_s": math.degrees(r),
        "lateral_acc_g": ay / G,
        "turn_radius_m": radius,
        "kus_deg_per_g": kus_deg,
    }


def _step_derivatives(vehicle: Vehicle, vx: float, vy: float, r: float,
                       delta: float, m: float, Iz: float, a: float, b: float,
                       Cf: float, Cr: float, tire_model: str
                       ) -> tuple[float, float]:
    """Compute d/dt [vy, r] at current state for 2-DOF bicycle model."""
    alpha_f = (vy + a * r) / vx - delta if vx > 0 else 0.0
    alpha_r = (vy - b * r) / vx if vx > 0 else 0.0

    if tire_model == "pacejka":
        Fyf, Fyr = calc_pacejka_cornering_forces(vehicle, alpha_f, alpha_r)
    else:
        Fyf = -Cf * alpha_f
        Fyr = -Cr * alpha_r

    dvy = (Fyf + Fyr) / m - vx * r
    dr = (a * Fyf - b * Fyr) / Iz
    return dvy, dr


def simulate_step_steer(vehicle: Vehicle, vx_kmh: float,
                        steer_angle_deg: float, duration_s: float = 5,
                        dt: float = 0.01,
                        tire_model: str = "linear",
                        method: str = "euler") -> list[dict]:
    """阶跃转向瞬态响应：给定车速和前轮转角，仿真横摆响应

    使用 2-DOF 自行车模型。
    状态变量: [vy, r]（侧向速度、横摆角速度）

    Args:
        vehicle:          车辆对象
        vx_kmh:           纵向车速 (km/h)
        steer_angle_deg:  前轮转角 (deg)
        duration_s:       仿真时长 (s)
        dt:               积分步长 (s)
        tire_model:       轮胎模型 "linear"（线性 Fy=-Cα·α）或 "pacejka"（魔术公式）
        method:           数值积分方法 "euler"（欧拉法）或 "rk4"（四阶 Runge-Kutta）

    Returns:
        list[dict]，每步含: time, vy, yaw_rate_rad, yaw_rate_deg, lateral_acc_g
    """
    _validate_time_grid(dt, duration_s)
    if not math.isfinite(vx_kmh) or vx_kmh < 0 or not math.isfinite(steer_angle_deg):
        raise ValueError("invalid speed or steering angle")
    if method not in ("euler", "rk4") or tire_model not in ("linear", "pacejka"):
        raise ValueError("unsupported integrator or tire model")
    vx = vx_kmh * KMH_TO_MS
    delta = math.radians(steer_angle_deg)
    m = vehicle.mass
    Iz = vehicle.yaw_inertia
    a = vehicle.cg_to_front
    b = vehicle.cg_to_rear
    Cf = vehicle.cornering_stiffness_f
    Cr = vehicle.cornering_stiffness_r

    # 初始状态
    vy = 0.0
    r = 0.0

    history: list[dict] = []
    steps = max(1, math.ceil(duration_s / dt - 1e-12)) if duration_s > 0 else 0
    for index in range(steps + 1):
        t = min(index * dt, duration_s) if index < steps else duration_s
        dvy, _ = _step_derivatives(
            vehicle, vx, vy, r, delta, m, Iz, a, b, Cf, Cr, tire_model)
        history.append({
            "time": round(t, 4),
            "vy": round(vy, 6),
            "yaw_rate_rad": round(r, 6),
            "yaw_rate_deg": round(math.degrees(r), 3),
            "lateral_acc_g": round((dvy + vx * r) / G, 6),
        })

        if t >= duration_s:
            break
        h = min(dt, duration_s - t)
        vy, r = integrate_lateral(vehicle, vx, vy, r, delta, h, tire_model, method)

    return history


def integrate_lateral(vehicle: Vehicle, vx: float, vy: float, r: float,
                      delta: float, dt: float, tire_model: str = "linear",
                      method: str = "euler") -> tuple[float, float]:
    """Integrate [vy, r] using substeps limited by the lateral relaxation rate.

    This avoids the explicit integrator's low-speed instability. It does not
    extend the bicycle model to reverse driving or combined tire slip.
    """
    if vx <= 0:
        return 0.0, 0.0
    m, iz = vehicle.mass, vehicle.yaw_inertia
    a, b = vehicle.cg_to_front, vehicle.cg_to_rear
    cf, cr = vehicle.cornering_stiffness_f, vehicle.cornering_stiffness_r
    rate = (cf + cr) / (m * vx) + (a * a * cf + b * b * cr) / (iz * vx)
    steps = max(1, math.ceil(dt * rate / 0.2))
    h = dt / steps

    def derivatives(y: float, yaw: float) -> tuple[float, float]:
        return _step_derivatives(vehicle, vx, y, yaw, delta, m, iz, a, b, cf, cr, tire_model)

    for _ in range(steps):
        k1y, k1r = derivatives(vy, r)
        if method == "rk4":
            k2y, k2r = derivatives(vy + h * k1y / 2, r + h * k1r / 2)
            k3y, k3r = derivatives(vy + h * k2y / 2, r + h * k2r / 2)
            k4y, k4r = derivatives(vy + h * k3y, r + h * k3r)
            vy += h * (k1y + 2 * k2y + 2 * k3y + k4y) / 6
            r += h * (k1r + 2 * k2r + 2 * k3r + k4r) / 6
        else:
            vy += h * k1y
            r += h * k1r
    return vy, r


def _classify_steer(kus_deg_per_g: float) -> str:
    """按不足转向梯度分类"""
    if kus_deg_per_g > 0.2:
        return "不足转向（稳定）"
    elif kus_deg_per_g < -0.2:
        return "过度转向（不稳定）"
    else:
        return "中性转向"


def analyze_lateral(vehicle: Vehicle) -> dict:
    """横向动力学综合分析，返回结构化数据。"""
    kus_rad, kus_deg = calc_understeer_gradient(vehicle)
    steer_type = _classify_steer(kus_deg)
    v_char = calc_characteristic_speed(vehicle)
    v_crit = calc_critical_speed(vehicle)

    return {
        "vehicle_name": vehicle.name,
        "wheelbase_m": vehicle.wheelbase,
        "cg_to_front_m": vehicle.cg_to_front,
        "cg_to_rear_m": vehicle.cg_to_rear,
        "cg_front_pct": vehicle.cg_to_front / vehicle.wheelbase * 100,
        "cg_rear_pct": vehicle.cg_to_rear / vehicle.wheelbase * 100,
        "cornering_stiffness_f": vehicle.cornering_stiffness_f,
        "cornering_stiffness_r": vehicle.cornering_stiffness_r,
        "yaw_inertia": vehicle.yaw_inertia,
        "kus_rad_per_g": kus_rad,
        "kus_deg_per_g": kus_deg,
        "steer_type": steer_type,
        "characteristic_speed_kmh": v_char,
        "critical_speed_kmh": v_crit,
    }


def calc_steady_cornering_table(vehicle: Vehicle) -> list[dict]:
    """不同车速下稳态转向响应对比表，返回结构化数据。"""
    results: list[dict] = []
    for v in [30, 60, 90, 120, 150]:
        result = calc_steady_state_cornering(vehicle, v, steer_angle_deg=3)
        results.append(result)
    return results


def calc_step_steer_response(vehicle: Vehicle, vx_kmh: float = 80,
                             steer_deg: float = 3,
                             tire_model: str = "linear",
                             method: str = "euler") -> dict:
    """阶跃转向瞬态响应，返回结构化数据。

    Args:
        tire_model: "linear" 或 "pacejka"
        method:     积分方法 "euler" 或 "rk4"
    """
    history = simulate_step_steer(vehicle, vx_kmh, steer_deg, duration_s=3,
                                   tire_model=tire_model, method=method)
    result = calc_steady_state_cornering(vehicle, vx_kmh, steer_deg)
    final = history[-1]
    final_r = final["yaw_rate_deg"]
    final_ay = final["lateral_acc_g"]

    return {
        "history": history,
        "steady_yaw_rate": result["yaw_rate_deg_s"],
        "steady_lateral_acc": result["lateral_acc_g"],
        "final_yaw_rate": final_r,
        "final_lateral_acc": final_ay,
        "tire_model": tire_model,
        "method": method,
    }
