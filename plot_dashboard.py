# -*- coding: utf-8 -*-
"""
BSFC 万有特性 + 横向动力学 + 跟车/ACC 五合一汇总图
五面板：发动机效率 Map / 稳态转向 / 转弯半径 / 阶跃瞬态 / ACC 跟车
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.interpolate import RectBivariateSpline

from bsfc import _BSFC_RPM_GRID, _BSFC_LOAD_GRID, _BSFC_GASOLINE
from vehicle import car_sedan, car_following_simulation, acc_simulation
from lateral_dynamics import (
    calc_steady_state_cornering,
    simulate_step_steer,
)
from _plot_utils import setup_chinese_font, get_label
from _constants import KMH_TO_MS


def plot_dashboard(vehicle=None, save_path=None):
    """五合一仪表盘: BSFC Map + 稳态转向 + 转弯半径 + 阶跃瞬态 + ACC 跟车"""
    if vehicle is None:
        vehicle = car_sedan

    # 中文字体
    has_font, _ = setup_chinese_font()

    fig = plt.figure(figsize=(18, 14))
    title_text = f"{vehicle.name} — 动力总成 & 横向动力学 & IDM 跟车 综合分析" if has_font else \
                 f"{vehicle.name} — Powertrain & Lateral & IDM Analysis"
    fig.suptitle(title_text, fontsize=16, fontweight="bold", y=0.99)

    gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.30,
                          height_ratios=[1, 1, 0.85])

    # (0,0): BSFC Map
    ax1 = fig.add_subplot(gs[0, 0])
    _draw_bsfc_panel(ax1)

    # (0,1): 稳态转向响应
    ax2 = fig.add_subplot(gs[0, 1])
    _draw_steady_cornering_panel(ax2, vehicle)

    # (1,0): 转弯半径 vs 车速
    ax3 = fig.add_subplot(gs[1, 0])
    _draw_turn_radius_panel(ax3, vehicle)

    # (1,1): 阶跃转向瞬态响应
    ax4 = fig.add_subplot(gs[1, 1])
    _draw_step_steer_panel(ax4, vehicle)

    # (2,:): ACC 跟车响应（跨两列）
    ax5 = fig.add_subplot(gs[2, :])
    _draw_acc_panel(ax5, vehicle)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    if save_path is None:
        save_path = "dashboard.png"

    plt.savefig(save_path, dpi=150)
    print(f"[仪表盘已保存] {save_path}")
    plt.close()


def _draw_bsfc_panel(ax):
    """BSFC 等高线图 + 等功率线"""
    rpm = np.array(_BSFC_RPM_GRID)
    load_pct = np.array(_BSFC_LOAD_GRID) * 100
    bsfc_data = np.array(_BSFC_GASOLINE)

    spline = RectBivariateSpline(load_pct, rpm, bsfc_data, kx=3, ky=3)
    rpm_fine = np.linspace(rpm[0], rpm[-1], 200)
    load_fine = np.linspace(load_pct[0], load_pct[-1], 150)
    R_fine, L_fine = np.meshgrid(rpm_fine, load_fine)
    bsfc_fine = spline(load_fine, rpm_fine)

    levels = [220, 240, 260, 280, 300, 330, 360, 400, 450, 500]
    cs = ax.contourf(R_fine, L_fine, bsfc_fine, levels=levels, cmap="RdYlGn_r", alpha=0.85)
    ax.contour(R_fine, L_fine, bsfc_fine, levels=levels, colors="black", linewidths=0.3)

    # 等功率线：P = T × ω → load% = P / (Tmax × RPM × 2π/60)
    # 参考 2.0L NA 汽油机 Tmax=180 Nm
    max_torque = 180   # Nm
    rpm_range = np.linspace(1000, 6200, 200)
    power_levels_kw = [10, 20, 40, 60, 80, 100]

    for pk in power_levels_kw:
        # load = P / (Tmax × rpm × 2π/60), load% = load × 100
        load_vals = pk * 60000 / (max_torque * rpm_range * 2 * math.pi) * 100
        # 截断超出 view 的部分
        mask = (load_vals >= 0) & (load_vals <= 105)
        ax.plot(rpm_range[mask], load_vals[mask],
                color="white", linewidth=0.8, linestyle="--", alpha=0.5)

        # 在曲线末端标注功率值
        valid_rpm = rpm_range[mask]
        valid_load = load_vals[mask]
        if len(valid_rpm) > 0:
            mid_i = len(valid_rpm) // 2
            ax.annotate(f"{pk}kW", (valid_rpm[mid_i], valid_load[mid_i] + 1),
                        fontsize=7, color="white", alpha=0.7, ha="center")

    # 三个典型工况点
    points = [
        (800, 5, get_label("怠速"), "red"),
        (2500, 75, get_label("经济巡航"), "darkgreen"),
        (5500, 90, get_label("全油门"), "darkred"),
    ]
    for r, l, label, color in points:
        ax.plot(r, l, "o", color=color, markersize=10, markeredgecolor="white", markeredgewidth=1.5)
        ax.annotate(label, (r + 100, l + 3), fontsize=8, color=color, fontweight="bold")

    ax.set_xlabel(get_label("发动机转速 (RPM)"))
    ax.set_ylabel(get_label("扭矩负荷比 (%)"))
    ax.set_title(get_label("BSFC 万有特性 Map + 等功率线"), fontweight="bold")
    fig = ax.figure
    fig.colorbar(cs, ax=ax, label="BSFC (g/kWh)", shrink=0.8)


def _draw_steady_cornering_panel(ax, vehicle):
    """稳态转向响应 双Y轴 + 中性转向参考"""
    speeds = np.linspace(10, 150, 30)
    yaw_rates = []
    lateral_accs = []

    for v in speeds:
        r = calc_steady_state_cornering(vehicle, v, steer_angle_deg=3)
        yaw_rates.append(r["yaw_rate_deg_s"])
        lateral_accs.append(r["lateral_acc_g"])

    color1 = "#2c7bb6"
    color2 = "#d7191c"
    color3 = "#7f7f7f"

    ax2_twin = ax.twinx()

    line1, = ax.plot(speeds, yaw_rates, color=color1, linewidth=2, label=get_label("横摆角速度"))
    line2, = ax2_twin.plot(speeds, lateral_accs, color=color2, linewidth=2, linestyle="--", label=get_label("侧向加速度"))

    # 中性转向参考线：r_neutral = vx / L × δ
    L = vehicle.wheelbase
    delta = math.radians(3)
    r_neutral = [math.degrees((v * KMH_TO_MS) / L * delta) for v in speeds]
    line3, = ax.plot(speeds, r_neutral, color=color3, linewidth=1, linestyle=":",
                     label=get_label("中性转向(参考)"))

    ax.set_xlabel(get_label("车速 (km/h)"))
    ax.set_ylabel(get_label("横摆角速度 (deg/s)"), color=color1)
    ax2_twin.set_ylabel(get_label("侧向加速度 (g)"), color=color2)
    ax.tick_params(axis="y", labelcolor=color1)
    ax2_twin.tick_params(axis="y", labelcolor=color2)

    # 图例
    lines = [line1, line2, line3]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc="upper left", fontsize=8)

    ax.set_title(get_label("稳态转向响应 (方向盘 3°)"), fontweight="bold")
    ax.grid(True, alpha=0.3)


def _draw_turn_radius_panel(ax, vehicle):
    """转弯半径 vs 车速 + 中性转向参考"""
    speeds = np.linspace(10, 150, 30)
    radii = [calc_steady_state_cornering(vehicle, v, steer_angle_deg=3)["turn_radius_m"]
             for v in speeds]

    ax.plot(speeds, radii, color="#1b7837", linewidth=2.5)
    ax.fill_between(speeds, radii, alpha=0.15, color="#1b7837")

    # 中性转向理论半径：R_neutral = L / δ
    L = vehicle.wheelbase
    delta = math.radians(3)
    r_neutral = L / delta
    ax.axhline(y=r_neutral, color="#d7191c", linestyle="--", alpha=0.7, linewidth=1.5)

    has_font, _ = setup_chinese_font()
    neutral_label = f"中性转向半径 {r_neutral:.1f}m" if has_font else f"Neutral Radius {r_neutral:.1f}m"
    ax.annotate(neutral_label,
                xy=(120, r_neutral + 1.5), fontsize=8, color="#d7191c", fontweight="bold")

    # 标注不足转向趋势
    ax.annotate(get_label("不足转向特征"),
                xy=(100, radii[20]), fontsize=9,
                color="#1b7837", fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.8))

    ax.set_xlabel(get_label("车速 (km/h)"))
    ax.set_ylabel(get_label("转弯半径 (m)"))
    ax.set_title(get_label("转弯半径 vs 车速 + 中性转向参考"), fontweight="bold")
    ax.grid(True, alpha=0.3)


def _draw_step_steer_panel(ax, vehicle):
    """阶跃转向瞬态响应 + 上升时间/超调/调节时间"""
    has_font, _ = setup_chinese_font()  # 有缓存，不会重复检测
    history = simulate_step_steer(vehicle, vx_kmh=80, steer_angle_deg=3, duration_s=3)

    times = [h[0] for h in history]
    r_deg = [h[3] for h in history]

    # 稳态理论值
    result = calc_steady_state_cornering(vehicle, 80, 3)
    r_steady = result["yaw_rate_deg_s"]

    ax.plot(times, r_deg, color="#2c7bb6", linewidth=2)
    steady_label = f"稳态 {r_steady:.1f}" if has_font else f"Steady {r_steady:.1f}"
    ax.axhline(y=r_steady, color="gray", linestyle="--", alpha=0.7, label=steady_label)

    # 上升时间（达到稳态 90%）
    target = 0.9 * r_steady
    rise_idx = next((i for i, r in enumerate(r_deg) if r >= target), None)
    t_rise = times[rise_idx] if rise_idx else None
    if t_rise:
        ax.axvline(x=t_rise, color="#d7191c", linestyle=":", alpha=0.6)
        rise_label = f"90%上升 {t_rise:.2f}s" if has_font else f"90% Rise {t_rise:.2f}s"
        ax.annotate(rise_label,
                    xy=(t_rise + 0.1, r_steady * 0.3),
                    fontsize=8, color="#d7191c")

    # 超调量
    r_max = max(r_deg)
    overshoot_pct = (r_max - r_steady) / r_steady * 100 if r_steady > 0 else 0
    if overshoot_pct > 0.5:
        ax.axhline(y=r_max, color="#d7191c", linestyle=":", alpha=0.4)
        os_label = f"超调 {overshoot_pct:.1f}%" if has_font else f"Overshoot {overshoot_pct:.1f}%"
        ax.annotate(os_label,
                    xy=(times[r_deg.index(r_max)], r_max),
                    xytext=(times[r_deg.index(r_max)] + 0.3, r_max + 1),
                    fontsize=8, color="#d7191c",
                    arrowprops=dict(arrowstyle="->", color="#d7191c", lw=1))

    # 调节时间（进入 ±2% 带且不再跳出）
    band = 0.02 * r_steady
    settled_idx = None
    for i in range(len(times) - 1, -1, -1):
        if abs(r_deg[i] - r_steady) > band:
            settled_idx = i + 1 if i + 1 < len(times) else None
            break
    if settled_idx and settled_idx < len(times):
        t_settle = times[settled_idx]
        ax.axvline(x=t_settle, color="#2c7bb6", linestyle=":", alpha=0.5)
        settle_label = f"调节 ±2% {t_settle:.2f}s" if has_font else f"Settle ±2% {t_settle:.2f}s"
        ax.annotate(settle_label,
                    xy=(t_settle + 0.1, r_steady * 0.65),
                    fontsize=8, color="#2c7bb6")

    ax.set_xlabel(get_label("时间 (s)"))
    ax.set_ylabel(get_label("横摆角速度 (deg/s)"))
    ax.set_title(get_label("阶跃转向瞬态响应 (80km/h, 3°)"), fontweight="bold")
    ax.grid(True, alpha=0.3)


def _draw_acc_panel(ax, vehicle):
    """IDM 跟车 / ACC 自适应巡航响应"""
    has_font, _ = setup_chinese_font()
    # 用 ACC 场景数据
    data = acc_simulation()

    times = data["time"]
    ax_twin = ax.twinx()

    color_vf = "#2c7bb6"
    color_vl = "#7f7f7f"
    color_gap = "#d7191c"

    # 后车速度（左Y轴）
    line1, = ax.plot(times, data["follower_kmh"], color=color_vf, linewidth=1.8,
                     label=get_label("后车速度"))
    line2, = ax.plot(times, data["leader_kmh"], color=color_vl, linewidth=1.5,
                     linestyle="--", label=get_label("前车速度"))

    # 间距（右Y轴）
    line3, = ax_twin.plot(times, data["gap_m"], color=color_gap, linewidth=1.2,
                          linestyle=":", alpha=0.7, label=get_label("间距 (m)"))

    ax.set_xlabel(get_label("时间 (s)"))
    ax.set_ylabel("速度 (km/h)", color=color_vf)
    ax_twin.set_ylabel("间距 (m)", color=color_gap)
    ax.tick_params(axis="y", labelcolor=color_vf)
    ax_twin.tick_params(axis="y", labelcolor=color_gap)

    # 图例
    lines = [line1, line2, line3]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc="upper right", fontsize=8)
    ax.set_title(get_label("IDM 自适应巡航 (ACC) 响应"), fontweight="bold")
    ax.grid(True, alpha=0.3)


if __name__ == "__main__":
    plot_dashboard()
