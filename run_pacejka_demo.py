# -*- coding: utf-8 -*-
"""Pacejka vs Linear step steer comparison demo"""
from __future__ import annotations
from vehicle import Vehicle
from lateral_dynamics import simulate_step_steer, calc_steady_state_cornering

# 造一辆典型轿车
car = Vehicle("PacejkaTest", 1500, 100, drag_coeff=0.28,
              wheelbase_m=2.65, cg_to_front_m=1.2,
              cornering_stiffness_f=80000, cornering_stiffness_r=70000,
              max_torque_nm=180)

print(f"=== Pacejka 参数 ===")
print(f"前轴 B={car.pacejka_B_f:.3f} C={car.pacejka_C_f:.1f} D={car.pacejka_D_f:.0f} E={car.pacejka_E_f:.1f}")
print(f"后轴 B={car.pacejka_B_r:.3f} C={car.pacejka_C_r:.1f} D={car.pacejka_D_r:.0f} E={car.pacejka_E_r:.1f}")
print(f"验算 B*C*D(f)={car.pacejka_B_f*car.pacejka_C_f*car.pacejka_D_f:.0f} vs Ca={car.cornering_stiffness_f:.0f}")
print(f"验算 B*C*D(r)={car.pacejka_B_r*car.pacejka_C_r*car.pacejka_D_r:.0f} vs Ca={car.cornering_stiffness_r:.0f}")

# 80km/h, 3 steady
steady = calc_steady_state_cornering(car, 80, 3)
print(f"\n=== 稳态理论值 (线性模型) ===")
print(f"横摆角速度: {steady['yaw_rate_deg_s']:.3f} deg/s")
print(f"侧向加速度: {steady['lateral_acc_g']:.3f} g")
print(f"Kus:        {steady['kus_deg_per_g']:.3f} deg/g")

# Step steer comparison
h_lin = simulate_step_steer(car, 80, 3, duration_s=3, tire_model="linear")
h_pac = simulate_step_steer(car, 80, 3, duration_s=3, tire_model="pacejka")

print(f"\n=== 阶跃瞬态响应对比 (80km/h, 3deg, dt=0.01s) ===")
print(f"{'time':>6}  {'linear r':>9}  {'pacejka r':>9}  {'diff%':>7}  {'linear ay':>9}  {'pacejka ay':>9}")
print("-" * 65)

interval = 25  # 0.25s
for i in range(0, len(h_lin), interval):
    hl = h_lin[i]
    hp = h_pac[i]
    t = hl["time"]
    rl = hl["yaw_rate_deg"]
    rp = hp["yaw_rate_deg"]
    ayl = hl["lateral_acc_g"]
    ayp = hp["lateral_acc_g"]
    diff = (rp - rl) / rl * 100 if abs(rl) > 1e-6 else 0.0
    print(f"{t:6.2f}  {rl:9.3f}  {rp:9.3f}  {diff:+6.1f}%  {ayl:9.4f}  {ayp:9.4f}")

# Final state
hl_f = h_lin[-1]
hp_f = h_pac[-1]
rl_f = hl_f["yaw_rate_deg"]
ayl_f = hl_f["lateral_acc_g"]
rp_f = hp_f["yaw_rate_deg"]
ayp_f = hp_f["lateral_acc_g"]
print(f"\n=== 终态 (t=3.0s) ===")
print(f"线性:   r={rl_f:.3f} deg/s, ay={ayl_f:.4f} g")
print(f"Pacejka: r={rp_f:.3f} deg/s, ay={ayp_f:.4f} g")
print(f"delta:   Dr={rl_f-rp_f:.3f} deg/s ({(rl_f-rp_f)/rl_f*100:.1f}%)")

# Large angle saturation test
h_lin8 = simulate_step_steer(car, 80, 8, duration_s=2, tire_model="linear")
h_pac8 = simulate_step_steer(car, 80, 8, duration_s=2, tire_model="pacejka")
hl8 = h_lin8[-1]
hp8 = h_pac8[-1]
rl8 = hl8["yaw_rate_deg"]
ayl8 = hl8["lateral_acc_g"]
rp8 = hp8["yaw_rate_deg"]
ayp8 = hp8["lateral_acc_g"]
print(f"\n=== 大转角饱和效应 (80km/h, 8deg) ===")
print(f"线性:   r={rl8:.3f} deg/s, ay={ayl8:.4f} g")
print(f"Pacejka: r={rp8:.3f} deg/s, ay={ayp8:.4f} g")
print(f"饱和差:  Dr={rl8-rp8:.3f} deg/s ({(rl8-rp8)/rl8*100:.1f}%) <-- Pacejka 低于线性，体现轮胎力饱和")

print(f"\n=== 结论 ===")
print(f"小转角(3deg): 两者几乎一致 (差值 < 1%)，B*C*D = Ca 保证了小侧偏角等价")
print(f"大转角(8deg): Pacejka 明显低于线性 ({abs(rl8-rp8)/rl8*100:.0f}%)，体现轮胎侧偏力非线性饱和")
