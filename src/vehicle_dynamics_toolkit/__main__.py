# -*- coding: utf-8 -*-
"""Entry point for ``python -m vehicle_dynamics_toolkit`` — runs full demo."""
import logging

from .vehicle import (
    car_sedan, car_suv, car_truck,
    simulate_acceleration, calc_braking_table, calc_power_breakdown,
)
from .lateral_dynamics import (
    analyze_lateral, calc_steady_cornering_table, calc_step_steer_response,
)


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger(__name__)
    logger.info("开始车辆动力学演示")
    print("""
╔══════════════════════════════════════════════╗
║         车辆动力学仿真                         ║
║         Python + 物理建模                     ║
╚══════════════════════════════════════════════╝
    """)

    # ── 纵向动力学 ──
    logger.info("阶段 1/2：计算纵向动力学（加速、制动、功率）")
    acc_result = simulate_acceleration(car_sedan, target_speed_kmh=100)
    print(f"  加速到 100 km/h: {acc_result['elapsed_s']:.1f}s, {acc_result['total_dist_m']:.0f}m")

    braking_data = calc_braking_table()
    print(f"\n  制动距离表 (120km/h): 反应 {braking_data[-1]['reaction_dist_m']:.1f}m + 制动 {braking_data[-1]['braking_dist_m']:.1f}m")

    for car, speed, grade in [
        (car_sedan, 100, 5), (car_suv, 100, 5), (car_truck, 80, 3),
    ]:
        pb = calc_power_breakdown(car, speed_kmh=speed, grade_percent=grade)
        print(f"  {pb['vehicle_name']} @ {pb['speed_kmh']}km/h: 功率利用率 {pb['power_utilization_pct']:.1f}%")

    # ── 横向动力学 ──
    logger.info("阶段 2/2：计算横向动力学（稳态转向、阶跃转向）")
    lat_data = analyze_lateral(car_sedan)
    print(f"\n  横向动力学: {lat_data['steer_type']} (Kus={lat_data['kus_deg_per_g']:.3f} deg/g)")
    print(f"  特征车速: {lat_data['characteristic_speed_kmh']:.0f} km/h")

    cornering = calc_steady_cornering_table(car_sedan)
    print(f"  稳态转向 (120km/h): 转弯半径 {cornering[-1]['turn_radius_m']:.1f}m")

    steer = calc_step_steer_response(car_sedan, vx_kmh=80, steer_deg=3)
    print(f"  阶跃转向终值: r={steer['final_yaw_rate']:.2f} deg/s")


if __name__ == "__main__":
    main()
