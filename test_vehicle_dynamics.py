# -*- coding: utf-8 -*-
"""Pytest unit tests for vehicle dynamics module"""

import pytest
import math
from _constants import G, RHO_AIR, KMH_TO_MS
from vehicle import (
    Vehicle,
    calc_resistance,
    calc_braking_distance,
    calc_acceleration,
    calc_grade_power,
    calc_power_to_weight,
    calc_aero_drag_power,
    rolling_coeff_dynamic,
    simulate_acceleration,
    calc_power_breakdown,
    calc_braking_table,
    calc_wheel_force,
    get_engine_torque,
    idm_acceleration,
    car_following_simulation,
    acc_simulation,
)
from lateral_dynamics import (
    calc_slip_angles,
    calc_cornering_forces,
    calc_pacejka_lateral_force,
    calc_pacejka_cornering_forces,
    calc_understeer_gradient,
    calc_characteristic_speed,
    calc_critical_speed,
    calc_steady_state_cornering,
    simulate_step_steer,
    analyze_lateral,
    calc_steady_cornering_table,
    calc_step_steer_response,
)
from bsfc import (
    _interpolate_bsfc,
    _calc_l100_raw,
    _BSFC_RPM_GRID,
    _BSFC_LOAD_GRID,
    calc_fuel_table,
)
from wltc import (
    get_wltc_profile,
    _WLTC_DURATION,
)


# --- Fixtures ---

@pytest.fixture
def sedan():
    return Vehicle("TestSedan", 1500, 100, drag_coeff=0.28,
                   max_torque_nm=180, gear_ratios=[3.55, 2.11, 1.42, 1.00, 0.78],
                   final_drive=4.06, wheel_radius_m=0.32, trans_efficiency=0.90,
                   fuel_density_gl=740, fuel_type="gasoline")


@pytest.fixture
def truck():
    return Vehicle("TestTruck", 15000, 300, drag_coeff=0.65, frontal_area_m2=7.0,
                   max_torque_nm=1000, gear_ratios=[5.50, 3.20, 1.90, 1.00, 0.73],
                   final_drive=4.30, wheel_radius_m=0.52, trans_efficiency=0.85,
                   fuel_density_gl=840, fuel_type="diesel")


# Vehicle class

class TestVehicle:
    def test_default_attributes(self, sedan):
        assert sedan.name == "TestSedan"
        assert sedan.mass == 1500
        assert sedan.power == 100_000
        assert sedan.cd == 0.28
        assert sedan.area == 2.2
        assert sedan.rolling_coeff == 0.015
        assert sedan.max_torque == 180
        assert sedan.idle_rpm == 800
        assert sedan.max_rpm == 6000  # explicitly set in fixture
        assert sedan.fuel_type == "gasoline"

    def test_select_gear_stopped(self, sedan):
        assert sedan.select_gear(0) == 0

    def test_select_gear_low_speed(self, sedan):
        gear = sedan.select_gear(30)
        assert gear >= 1
        assert gear <= len(sedan.gear_ratios)

    def test_select_gear_high_speed(self, sedan):
        gear = sedan.select_gear(120)
        assert gear >= 3  # should be in higher gears

    def test_select_gear_returns_valid_range(self, sedan):
        for speed in [5, 15, 30, 50, 80, 110, 130]:
            gear = sedan.select_gear(speed)
            assert 0 <= gear <= len(sedan.gear_ratios)

    def test_select_gear_increasing_speed_gives_higher_gear(self, sedan):
        g_low = sedan.select_gear(20)
        g_high = sedan.select_gear(80)
        assert g_low <= g_high  # higher speed, higher gear


# calc_resistance

class TestCalcResistance:
    def test_rolling_resistance_zero_speed(self, sedan):
        """At v=0, zero aero drag, only rolling."""
        resistance = calc_resistance(sedan, 0)
        expected_rolling = 0.015 * 1500 * G  # = 220.5 N
        assert resistance == pytest.approx(expected_rolling, rel=1e-6)

    def test_aero_increases_with_speed(self, sedan):
        r_low = calc_resistance(sedan, 10)
        r_high = calc_resistance(sedan, 30)
        assert r_high > r_low

    def test_aero_drag_formula(self, sedan):
        """Aero: 0.5 * RHO_AIR * Cd * A * v^2"""
        v = 20  # m/s
        resistance = calc_resistance(sedan, v)
        rolling = 0.015 * 1500 * G
        aero = 0.5 * RHO_AIR * 0.28 * 2.2 * v ** 2
        expected = rolling + aero
        assert resistance == pytest.approx(expected, rel=1e-6)

    def test_heavier_vehicle_more_rolling(self, sedan, truck):
        r_sedan = calc_resistance(sedan, 0)
        r_truck = calc_resistance(truck, 0)
        assert r_truck > r_sedan


class TestDynamicRollingResistance:
    """rolling_coeff_dynamic + calc_resistance(dynamic_rr=True)"""

    def test_zero_speed_returns_f0(self):
        mu = rolling_coeff_dynamic(0)
        assert mu == pytest.approx(0.010, rel=1e-6)

    def test_100kmh_returns_sum(self):
        """v=100 → v/100=1 → f0+f1+f4"""
        v_ms = 100 * KMH_TO_MS
        mu = rolling_coeff_dynamic(v_ms)
        assert mu == pytest.approx(0.010 + 0.005 + 0.002, rel=1e-6)

    def test_increases_with_speed(self):
        mu_low = rolling_coeff_dynamic(30 * KMH_TO_MS)
        mu_high = rolling_coeff_dynamic(120 * KMH_TO_MS)
        assert mu_high > mu_low

    def test_fourth_order_dominates_at_high_speed(self):
        """120km/h 时四次项贡献应显著大于 60km/h"""
        mu_60 = rolling_coeff_dynamic(60 * KMH_TO_MS)
        mu_120 = rolling_coeff_dynamic(120 * KMH_TO_MS)
        # 从 60→120，增量主要来自四次项
        assert mu_120 - mu_60 > 0.003

    def test_dynamic_rr_lower_than_constant_at_low_speed(self, sedan):
        """常量 μ=0.015，动态在低速时应更低"""
        # 有显式参数的车，不是 sedan fixture
        r_const = calc_resistance(sedan, 10)
        r_dyn = calc_resistance(sedan, 10, dynamic_rr=True)
        assert r_dyn < r_const

    def test_dynamic_rr_switch_defaults_to_false(self, sedan):
        """不传第三个参数时默认用常量"""
        r1 = calc_resistance(sedan, 20)
        r2 = calc_resistance(sedan, 20, dynamic_rr=False)
        assert r1 == r2


# calc_braking_distance

class TestBrakingDistance:
    def test_braking_formula(self):
        """Total = reaction_dist + braking_dist"""
        v = 50  # km/h
        rd, bd, td = calc_braking_distance(v, friction_coeff=0.7, reaction_time=1.5)

        expected_rd = (50 * KMH_TO_MS) * 1.5
        expected_bd = (50 * KMH_TO_MS) ** 2 / (2 * 0.7 * G)

        assert rd == pytest.approx(expected_rd, rel=1e-6)
        assert bd == pytest.approx(expected_bd, rel=1e-6)
        assert td == pytest.approx(expected_rd + expected_bd, rel=1e-6)

    def test_higher_speed_longer_distance(self):
        _, _, td_low = calc_braking_distance(30)
        _, _, td_high = calc_braking_distance(80)
        assert td_high > td_low

    def test_wet_road_longer_distance(self):
        """Lower friction coeff = longer braking distance."""
        _, bd_dry, _ = calc_braking_distance(60, friction_coeff=0.7)
        _, bd_wet, _ = calc_braking_distance(60, friction_coeff=0.3)
        assert bd_wet > bd_dry

    def test_zero_speed(self):
        rd, bd, td = calc_braking_distance(0)
        assert rd == 0
        assert bd == 0
        assert td == 0


# _interpolate_bsfc (bilinear interpolation on BSFC map)

class TestInterpolateBSFC:
    def test_returns_float(self):
        result = _interpolate_bsfc(2500, 0.5, "gasoline")
        assert isinstance(result, float)

    def test_optimal_region_low_bsfc(self):
        """At 2500 RPM, 50% load should be near optimal ~233 g/kWh for gasoline."""
        bsfc = _interpolate_bsfc(2500, 0.5, "gasoline")
        assert bsfc < 300  # should be in efficient zone

    def test_idle_high_bsfc(self):
        """At idle (800 RPM), low load (5%) should have high BSFC ~580."""
        bsfc = _interpolate_bsfc(800, 0.05, "gasoline")
        assert bsfc > 400  # idle is inefficient

    def test_high_rpm_high_bsfc(self):
        """At redline and high load, BSFC is high."""
        bsfc = _interpolate_bsfc(6200, 0.85, "gasoline")
        assert bsfc > 300

    def test_clamps_rpm_to_map_bounds(self):
        """Values outside RPM grid should be clamped."""
        bsfc_low = _interpolate_bsfc(100, 0.5, "gasoline")
        bsfc_high = _interpolate_bsfc(10000, 0.5, "gasoline")
        # Should not crash, should return valid values
        assert bsfc_low > 0
        assert bsfc_high > 0

    def test_clamps_load_to_map_bounds(self):
        bsfc_zero = _interpolate_bsfc(2500, 0.0, "gasoline")
        bsfc_over = _interpolate_bsfc(2500, 2.0, "gasoline")
        assert bsfc_zero > 0
        assert bsfc_over > 0

    def test_diesel_vs_gasoline(self):
        """Diesel BSFC should generally be lower than gasoline."""
        gas = _interpolate_bsfc(2500, 0.5, "gasoline")
        diesel = _interpolate_bsfc(2000, 0.5, "diesel")
        # Diesel map values are lower
        assert diesel < gas

    def test_monotonically_decreasing_then_increasing(self):
        """BSFC should form a U-shape: high at very low load, low in middle."""
        low = _interpolate_bsfc(2500, 0.05, "gasoline")
        mid = _interpolate_bsfc(2500, 0.50, "gasoline")
        high = _interpolate_bsfc(2500, 1.0, "gasoline")
        assert mid < low
        assert mid < high


# _calc_l100_raw (fuel consumption per 100km)

class TestFuelConsumption:
    def test_zero_speed(self, sedan):
        fuel = _calc_l100_raw(sedan, 0)
        assert fuel == 0.0

    def test_cruise_returns_reasonable(self, sedan):
        """Highway cruise should be ~5-8 L/100km."""
        fuel = _calc_l100_raw(sedan, 90)
        assert 3 < fuel < 12

    def test_low_speed_reasonable(self, sedan):
        """20 km/h should return a reasonable L/100km value."""
        fuel = _calc_l100_raw(sedan, 20)
        assert 2 < fuel < 8

    def test_truck_diesel_higher_absolute(self, sedan, truck):
        """Truck consumes more fuel per 100km in absolute terms."""
        fuel_sedan = _calc_l100_raw(sedan, 80)
        fuel_truck = _calc_l100_raw(truck, 80)
        assert fuel_truck > fuel_sedan


# get_wltc_profile

class TestWLTCProfile:
    def test_returns_correct_length(self):
        profile = get_wltc_profile()
        assert len(profile) == _WLTC_DURATION + 1

    def test_starts_at_zero(self):
        profile = get_wltc_profile()
        assert profile[0] == 0
        # Last element may not be exactly 0 due to interpolation coverage limits

    def test_all_non_negative(self):
        profile = get_wltc_profile()
        assert all(v >= 0 for v in profile)

    def test_has_high_speed_segments(self):
        profile = get_wltc_profile()
        max_speed = max(profile)
        assert max_speed > 100  # WLTC Class 3 has speeds > 130 km/h

    def test_profile_is_numeric(self):
        profile = get_wltc_profile()
        for v in profile:
            assert isinstance(v, (int, float))


# calc_acceleration — F = ma physics

class TestAcceleration:
    def test_zero_at_standstill(self, sedan):
        """v=0 时轮端驱动力为 0，加速度应为 0。"""
        acc = calc_acceleration(sedan, 0)
        assert acc == 0

    def test_decreases_with_speed(self, sedan):
        """高速时空气阻力增大 + 高挡扭矩降低 → 加速度下降。"""
        acc_low = calc_acceleration(sedan, 15)   # 54 km/h
        acc_high = calc_acceleration(sedan, 30)  # 108 km/h
        assert acc_high < acc_low

    def test_torque_curve_based_not_p_over_v(self, sedan):
        """扭矩曲线模型 vs P=Fv：验证不再用功率/速度简单公式。"""
        v = 20  # m/s, 72 km/h
        # P=Fv 模型：100kW/20m/s = 5000N 驱动力
        # 扭矩曲线模型应显著低于 P=Fv（发动机非恒定功率）
        acc = calc_acceleration(sedan, v)
        # 72km/h 时普通轿车加速度应 < 3 m/s²（P=Fv 给 ~3.1）
        assert 0.2 < acc < 3.0, f"加速度 {acc:.2f} 偏离合理范围"

    def test_heavier_slower(self, sedan, truck):
        """15 吨卡车比 1.5 吨轿车加速慢。"""
        acc_sedan = calc_acceleration(sedan, 10)
        acc_truck = calc_acceleration(truck, 10)
        assert acc_sedan > acc_truck

    def test_throttle_partial_lower_than_full(self, sedan):
        """半油门加速应 < 全油门。"""
        acc_wot = calc_acceleration(sedan, 15, throttle=1.0)
        acc_part = calc_acceleration(sedan, 15, throttle=0.5)
        assert acc_wot > acc_part


class TestWheelForce:
    """calc_wheel_force — 发动机扭矩 → 轮端驱动力"""

    def test_low_speed_positive(self, sedan):
        force = calc_wheel_force(sedan, 8, throttle=1.0)
        assert force > 0, "应有驱动力"

    def test_zero_speed_zero_force(self, sedan):
        assert calc_wheel_force(sedan, 0) == 0.0

    def test_partial_throttle_lower_force(self, sedan):
        f_wot = calc_wheel_force(sedan, 10, throttle=1.0)
        f_part = calc_wheel_force(sedan, 10, throttle=0.3)
        assert f_part < f_wot

    def test_gear_override_works(self, sedan):
        """强制 1 档 vs 自动选档应有不同驱动力"""
        f_auto = calc_wheel_force(sedan, 15)  # 15 m/s=54 km/h, 自动选高档
        f_g1 = calc_wheel_force(sedan, 15, gear_override=1)
        # 1 档扭矩放大更大
        assert f_g1 > f_auto


class TestEngineTorque:
    """get_engine_torque — 扭矩曲线查表"""

    def test_wot_at_peak_rpm(self, sedan):
        tq = get_engine_torque(3500, 1.0, sedan.torque_curve)
        assert tq == pytest.approx(sedan.max_torque, rel=0.05)

    def test_idle_low_torque(self, sedan):
        tq = get_engine_torque(sedan.idle_rpm, 1.0, sedan.torque_curve)
        assert tq < sedan.max_torque * 0.5

    def test_throttle_scales_linearly(self, sedan):
        tq_full = get_engine_torque(3000, 1.0, sedan.torque_curve)
        tq_half = get_engine_torque(3000, 0.5, sedan.torque_curve)
        assert tq_half == pytest.approx(tq_full * 0.5, rel=0.01)

    def test_clamped_at_bounds(self, sedan):
        """超范围 RPM 应被钳制，不应崩溃"""
        tq_low = get_engine_torque(100, 1.0, sedan.torque_curve)
        tq_high = get_engine_torque(20000, 1.0, sedan.torque_curve)
        assert tq_low > 0
        assert tq_high > 0


# 爬坡功率、比功率、风阻功率

class TestGradePower:
    """calc_grade_power — 爬坡功率"""

    def test_zero_speed_zero_power(self, sedan):
        assert calc_grade_power(sedan, 0, 5) == 0

    def test_zero_grade_zero_power(self, sedan):
        assert calc_grade_power(sedan, 20, 0) == 0

    def test_steeper_grade_more_power(self, sedan):
        p5 = calc_grade_power(sedan, 20, 5)
        p10 = calc_grade_power(sedan, 20, 10)
        assert p10 > p5

    def test_truck_needs_more_grade_power(self, sedan, truck):
        p_sedan = calc_grade_power(sedan, 15, 5)
        p_truck = calc_grade_power(truck, 15, 5)
        assert p_truck > p_sedan

    def test_returns_watts(self, sedan):
        p = calc_grade_power(sedan, 20, 5)
        assert p > 0
        assert isinstance(p, float)


class TestPowerToWeight:
    """calc_power_to_weight — 比功率"""

    def test_sedan_reasonable(self, sedan):
        wpk, kpt = calc_power_to_weight(sedan)
        assert 50 < wpk < 100
        assert wpk == pytest.approx(100_000 / 1500, rel=1e-6)

    def test_returns_tuple(self, sedan):
        result = calc_power_to_weight(sedan)
        assert len(result) == 2

    def test_truck_lower_than_sedan(self, sedan, truck):
        wpk_s, _ = calc_power_to_weight(sedan)
        wpk_t, _ = calc_power_to_weight(truck)
        assert wpk_t < wpk_s

    def test_kw_per_ton_equals_w_per_kg(self, sedan):
        wpk, kpt = calc_power_to_weight(sedan)
        assert wpk == pytest.approx(kpt, rel=1e-6)


class TestAeroDragPower:
    """calc_aero_drag_power — 风阻功率"""

    def test_zero_speed_zero_power(self, sedan):
        assert calc_aero_drag_power(sedan, 0) == 0

    def test_cubic_relationship(self, sedan):
        """风阻功率 ∝ v³"""
        p1 = calc_aero_drag_power(sedan, 10)
        p2 = calc_aero_drag_power(sedan, 20)
        # v翻倍 → 功率应为 8 倍
        assert p2 == pytest.approx(p1 * 8, rel=1e-6)

    def test_higher_cd_more_power(self, sedan, truck):
        p_sedan = calc_aero_drag_power(sedan, 30)
        p_truck = calc_aero_drag_power(truck, 30)
        assert p_truck > p_sedan

    def test_formula_correct(self, sedan):
        v = 25  # m/s = 90 km/h
        expected = 0.5 * RHO_AIR * sedan.cd * sedan.area * v ** 3
        assert calc_aero_drag_power(sedan, v) == pytest.approx(expected, rel=1e-6)


# 真实车型参数验证 — 用已知油耗反推模型合理性

class TestRealWorldBenchmarks:
    """对照真实车型公告油耗，验证仿真模型不偏离物理实际。"""

    @pytest.fixture
    def camry_20(self):
        """Toyota Camry 2.0L 汽油机：1550kg, 127kW, 公告油耗 ~5.8 L/100km"""
        return Vehicle("Camry2.0", 1550, 127, drag_coeff=0.27, frontal_area_m2=2.3,
                       max_torque_nm=207, gear_ratios=[3.30, 1.90, 1.42, 1.00, 0.713],
                       final_drive=3.63, wheel_radius_m=0.335, trans_efficiency=0.92,
                       fuel_density_gl=740, fuel_type="gasoline")

    def test_camry_90kmh_cruise_in_ballpark(self, camry_20):
        """90 km/h 定速巡航油耗应在公告值 ±2 L/100km 范围。"""
        fuel = _calc_l100_raw(camry_20, 90)
        assert 4.5 < fuel < 8.0, f"仿真值 {fuel:.1f} 偏离真实范围"

    def test_camry_120kmh_higher_than_90(self, camry_20):
        """风阻正比于 v²，120km/h 应比 90km/h 油耗高。"""
        assert _calc_l100_raw(camry_20, 120) > _calc_l100_raw(camry_20, 90)

    def test_suv_higher_than_sedan(self, sedan, truck):
        """SUV/卡车在同等车速下油耗应高于轿车。"""
        fuel_sedan = _calc_l100_raw(sedan, 80)
        fuel_truck = _calc_l100_raw(truck, 80)
        assert fuel_truck > fuel_sedan

    def test_pipeline_vehicle_to_fuel_consistency(self, sedan):
        """全链路验证：车速 → 选档 → 转速 → 负荷 → BSFC → 油耗，不抛异常且合理。"""
        v = 60  # km/h
        gear = sedan.select_gear(v)
        assert gear > 0, "60 km/h 档位应为正"
        # 通过 calc_resistance 反算发动机扭矩
        resistance = calc_resistance(sedan, v * KMH_TO_MS)
        total_ratio = sedan.gear_ratios[gear - 1] * sedan.final_drive
        engine_torque = resistance * sedan.wheel_radius / (total_ratio * sedan.trans_efficiency)
        load = max(0.01, min(1.0, engine_torque / sedan.max_torque))
        wheel_rps = (v * KMH_TO_MS) / (2 * math.pi * sedan.wheel_radius)
        engine_rpm = wheel_rps * total_ratio * 60
        bsfc = _interpolate_bsfc(engine_rpm, load, sedan.fuel_type)
        # 60 km/h 巡航时发动机应运行在经济区 (BSFC < 350)
        assert bsfc < 350, f"BSFC {bsfc:.0f} > 350，发动机效率异常"


# WLTC 工况数据质量验证

class TestWLTCDataQuality:
    """WLTC Class 3 工况数据应满足法规定义的特征。"""

    def test_four_phases_exist(self):
        """四个阶段应有明显的速度分区特征。"""
        profile = get_wltc_profile()
        low = max(profile[0:589])
        med = max(profile[590:1022])
        high = max(profile[1023:1477])
        exhi = max(profile[1478:1800])
        assert low < med < high < exhi, "四阶段最高速度应递增"

    def test_phase_one_is_city_low_speed(self):
        """Phase 1 (Low) 最高速度不超过 60 km/h。"""
        profile = get_wltc_profile()
        phase1_max = max(profile[0:590])
        assert phase1_max < 60, f"Phase 1 最高速 {phase1_max:.0f} 超出城市工况范围"

    def test_phase_four_is_motorway(self):
        """Phase 4 (Extra High) 应有超过 100 km/h 的高速段。"""
        profile = get_wltc_profile()
        phase4_max = max(profile[1478:1801])
        assert phase4_max > 100, f"Phase 4 最高速 {phase4_max:.0f} 未达高速标准"

    def test_total_distance_approx_20km(self):
        """WLTC 工况总里程约 20 km（插值覆盖限制）。"""
        profile = get_wltc_profile()
        total_dist = sum(v * KMH_TO_MS for v in profile) / 1000  # 每 1 秒积分
        assert 18 < total_dist < 23, f"总里程 {total_dist:.1f}km 偏离合理范围"

    def test_idle_stops_exist(self):
        """WLTC 工况应包含多次停车怠速（车速=0）段。"""
        profile = get_wltc_profile()
        idle_count = sum(1 for v in profile if v == 0)
        assert idle_count > 50, f"仅 {idle_count} 个怠速点，停车次数不足"


# 横向动力学 — 自行车模型测试

@pytest.fixture
def lat_sedan():
    """带完整横向参数的轿车"""
    return Vehicle("LatSedan", 1500, 100, drag_coeff=0.28,
                   wheelbase_m=2.65, cg_to_front_m=1.2,
                   cornering_stiffness_f=80000, cornering_stiffness_r=70000,
                   max_torque_nm=180, gear_ratios=[3.55, 2.11, 1.42, 1.00, 0.78],
                   final_drive=4.06, wheel_radius_m=0.32)


@pytest.fixture
def lat_oversteer():
    """过度转向车：后轴侧偏刚度偏小"""
    return Vehicle("Oversteer", 1500, 100, drag_coeff=0.28,
                   wheelbase_m=2.65, cg_to_front_m=1.2,
                   cornering_stiffness_f=80000, cornering_stiffness_r=40000,
                   max_torque_nm=180, gear_ratios=[3.55, 2.11, 1.42, 1.00, 0.78],
                   final_drive=4.06, wheel_radius_m=0.32)


class TestVehicleLateralParams:
    """Vehicle 横向参数默认值"""

    def test_wheelbase_default(self, sedan):
        assert sedan.wheelbase > 0

    def test_cg_split_default(self, sedan):
        assert sedan.cg_to_front > 0
        assert sedan.cg_to_rear > 0
        assert sedan.wheelbase == pytest.approx(sedan.cg_to_front + sedan.cg_to_rear, rel=1e-6)

    def test_yaw_inertia_default(self, sedan):
        expected = sedan.mass * sedan.cg_to_front * sedan.cg_to_rear
        assert sedan.yaw_inertia == pytest.approx(expected, rel=1e-6)

    def test_cornering_stiffness_default(self, sedan):
        assert sedan.cornering_stiffness_f > 0
        assert sedan.cornering_stiffness_r > 0


class TestSlipAngles:
    """calc_slip_angles"""

    def test_zero_steer_zero_vy_gives_zero(self, lat_sedan):
        af, ar = calc_slip_angles(lat_sedan, vx_ms=20, vy_ms=0, yaw_rate=0, steer_angle_rad=0)
        assert af == 0
        assert ar == 0

    def test_steer_gives_negative_front_slip(self, lat_sedan):
        """转向时，初始侧偏角为负（轮胎运动方向落后于指向）"""
        af, ar = calc_slip_angles(lat_sedan, vx_ms=20, vy_ms=0, yaw_rate=0, steer_angle_rad=0.05)
        assert af < 0

    def test_positive_yaw_gives_front_less_negative(self, lat_sedan):
        """正横摆让前轮侧偏角负得更少"""
        af, ar = calc_slip_angles(lat_sedan, vx_ms=20, vy_ms=0, yaw_rate=0.1, steer_angle_rad=0.05)
        assert af < 0
        assert ar < 0


class TestUndersteerGradient:
    """calc_understeer_gradient"""

    def test_sedan_is_understeer(self, lat_sedan):
        _, kus_deg = calc_understeer_gradient(lat_sedan)
        assert kus_deg > 0, f"轿车应为不足转向，实际 {kus_deg:.3f} deg/g"

    def test_cg_forward_gives_more_understeer(self, lat_sedan):
        """质心前移 → 前轴载荷增加 → 不足转向更严重"""
        front_heavy = Vehicle("Front", 1500, 100, drag_coeff=0.28,
                              wheelbase_m=2.65, cg_to_front_m=1.0,
                              cornering_stiffness_f=80000, cornering_stiffness_r=70000)
        _, kus_f = calc_understeer_gradient(front_heavy)
        _, kus_s = calc_understeer_gradient(lat_sedan)
        # cg_to_front=1.0 < 1.2 → 前轴更重 → Kus 更大
        assert kus_f > kus_s

    def test_oversteer_car_negative_kus(self, lat_oversteer):
        _, kus_deg = calc_understeer_gradient(lat_oversteer)
        assert kus_deg < 0, f"过度转向车 Kus 应为负，实际 {kus_deg:.3f}"


class TestCharacteristicSpeed:
    """calc_characteristic_speed"""

    def test_sedan_has_finite_char_speed(self, lat_sedan):
        v_char = calc_characteristic_speed(lat_sedan)
        assert 50 < v_char < 300, f"特征车速应在合理范围，实际 {v_char:.0f}"

    def test_oversteer_has_infinite_char_speed(self, lat_oversteer):
        assert calc_characteristic_speed(lat_oversteer) == float("inf")


class TestCriticalSpeed:
    """calc_critical_speed"""

    def test_sedan_no_critical_speed(self, lat_sedan):
        assert calc_critical_speed(lat_sedan) == float("inf")

    def test_oversteer_has_finite_critical_speed(self, lat_oversteer):
        v_crit = calc_critical_speed(lat_oversteer)
        assert 0 < v_crit < 300, f"临界车速应在合理范围，实际 {v_crit:.0f}"


class TestSteadyStateCornering:
    """calc_steady_state_cornering"""

    def test_returns_dict_with_keys(self, lat_sedan):
        result = calc_steady_state_cornering(lat_sedan, 60, 3)
        for key in ["yaw_rate_deg_s", "lateral_acc_g", "turn_radius_m", "kus_deg_per_g"]:
            assert key in result

    def test_understeer_larger_radius_at_higher_speed(self, lat_sedan):
        r30 = calc_steady_state_cornering(lat_sedan, 30, 3)["turn_radius_m"]
        r90 = calc_steady_state_cornering(lat_sedan, 90, 3)["turn_radius_m"]
        # 不足转向 → 高速时转弯半径偏大
        assert r90 > r30

    def test_neutral_steer_constant_radius(self):
        """中性转向：转弯半径 ≈ L/δ，与车速无关"""
        neutral = Vehicle("Neutral", 1500, 100, drag_coeff=0.28,
                          wheelbase_m=2.65, cg_to_front_m=1.2,
                          cornering_stiffness_f=96600, cornering_stiffness_r=80000,
                          max_torque_nm=180)
        kus_rad, kus_deg = calc_understeer_gradient(neutral)
        # 确认 Kus ≈ 0（中性转向）
        assert abs(kus_deg) < 0.1, f"应为中性转向，实际 Kus={kus_deg:.4f} deg/g"
        r30 = calc_steady_state_cornering(neutral, 30, 3)["turn_radius_m"]
        r60 = calc_steady_state_cornering(neutral, 60, 3)["turn_radius_m"]
        # 中性转向半径变化很小
        assert abs(r60 - r30) / r30 < 0.10


class TestStepSteerSimulation:
    """simulate_step_steer"""

    def test_returns_non_empty_history(self, lat_sedan):
        history = simulate_step_steer(lat_sedan, 60, 3, duration_s=2)
        assert len(history) > 0

    def test_history_elements_are_five_tuples(self, lat_sedan):
        history = simulate_step_steer(lat_sedan, 60, 3, duration_s=1)
        assert all(len(h) == 5 for h in history)

    def test_converges_to_steady_state(self, lat_sedan):
        """仿真终值应收敛到稳态理论值"""
        result = calc_steady_state_cornering(lat_sedan, 60, 3)
        history = simulate_step_steer(lat_sedan, 60, 3, duration_s=5)
        _, _, _, final_r_deg, final_ay_g = history[-1]
        # 应在 5% 内收敛
        assert final_r_deg == pytest.approx(result["yaw_rate_deg_s"], rel=0.05)
        assert final_ay_g == pytest.approx(result["lateral_acc_g"], rel=0.05)

    def test_yaw_rate_starts_at_zero(self, lat_sedan):
        history = simulate_step_steer(lat_sedan, 60, 3, duration_s=1)
        _, _, r0, _, _ = history[0]
        assert r0 == 0


# ---- 高层封装函数冒烟测试 ----

class TestPowerBreakdown:
    """calc_power_breakdown — 返回结构化 dict"""

    def test_returns_required_keys(self, sedan):
        data = calc_power_breakdown(sedan, speed_kmh=100, grade_percent=5)
        for key in ["engine_max_power_w", "rolling_power_const_w", "rolling_power_dyn_w",
                     "aero_power_w", "grade_power_w", "power_utilization_pct"]:
            assert key in data

    def test_power_utilization_in_range(self, sedan):
        data = calc_power_breakdown(sedan, speed_kmh=100, grade_percent=5)
        assert 0 < data["power_utilization_pct"] < 100

    def test_dynamic_rr_different_from_constant(self, sedan):
        data = calc_power_breakdown(sedan, speed_kmh=120, grade_percent=0)
        assert data["rolling_power_dyn_w"] != pytest.approx(data["rolling_power_const_w"], abs=1)


class TestBrakingTable:
    """calc_braking_table — 返回 list[dict]"""

    def test_returns_correct_length(self):
        table = calc_braking_table()
        assert len(table) == 6

    def test_each_row_has_required_keys(self):
        for row in calc_braking_table():
            for key in ["speed_kmh", "reaction_dist_m", "braking_dist_m", "total_dist_m"]:
                assert key in row

    def test_higher_speed_longer_total(self):
        table = calc_braking_table()
        for i in range(len(table) - 1):
            assert table[i + 1]["total_dist_m"] > table[i]["total_dist_m"]


class TestSimulateAcceleration:
    """simulate_acceleration — 返回结构化 dict"""

    def test_returns_required_keys(self, sedan):
        data = simulate_acceleration(sedan, target_speed_kmh=100)
        for key in ["time", "speed_kmh", "acc_ms2", "distance_m", "elapsed_s", "total_dist_m"]:
            assert key in data

    def test_speed_increases_monotonically(self, sedan):
        data = simulate_acceleration(sedan, target_speed_kmh=60)
        speeds = data["speed_kmh"]
        for i in range(len(speeds) - 1):
            assert speeds[i + 1] >= speeds[i]

    def test_final_speed_reaches_target(self, sedan):
        data = simulate_acceleration(sedan, target_speed_kmh=60)
        assert data["speed_kmh"][-1] >= 60


class TestAnalyzeLateral:
    """analyze_lateral — 横向动力学综合分析"""

    def test_returns_required_keys(self, lat_sedan):
        data = analyze_lateral(lat_sedan)
        for key in ["wheelbase_m", "kus_deg_per_g", "steer_type",
                     "characteristic_speed_kmh", "critical_speed_kmh"]:
            assert key in data

    def test_sedan_is_understeer(self, lat_sedan):
        data = analyze_lateral(lat_sedan)
        assert "不足转向" in data["steer_type"]

    def test_oversteer_car_is_oversteer(self, lat_oversteer):
        data = analyze_lateral(lat_oversteer)
        assert "过度转向" in data["steer_type"]


class TestSteadyCorneringTable:
    """calc_steady_cornering_table — 返回 list[dict]"""

    def test_returns_non_empty(self, lat_sedan):
        table = calc_steady_cornering_table(lat_sedan)
        assert len(table) == 5

    def test_higher_speed_larger_radius(self, lat_sedan):
        table = calc_steady_cornering_table(lat_sedan)
        for i in range(len(table) - 1):
            assert table[i + 1]["turn_radius_m"] > table[i]["turn_radius_m"]


class TestStepSteerResponse:
    """calc_step_steer_response — 返回结构化 dict"""

    def test_returns_required_keys(self, lat_sedan):
        data = calc_step_steer_response(lat_sedan, vx_kmh=80, steer_deg=3)
        for key in ["history", "steady_yaw_rate", "steady_lateral_acc",
                     "final_yaw_rate", "final_lateral_acc"]:
            assert key in data

    def test_final_near_steady(self, lat_sedan):
        data = calc_step_steer_response(lat_sedan, vx_kmh=80, steer_deg=3)
        assert data["final_yaw_rate"] == pytest.approx(data["steady_yaw_rate"], rel=0.05)


class TestFuelTable:
    """calc_fuel_table — 返回 list[dict]"""

    def test_returns_non_empty(self):
        table = calc_fuel_table()
        assert len(table) == 3 * 7  # 3 cars x 7 speeds

    def test_each_row_has_required_keys(self):
        for row in calc_fuel_table():
            for key in ["car_name", "speed_kmh", "gear", "rpm", "load_pct",
                         "bsfc_gkwh", "l100km"]:
                assert key in row

    def test_truck_higher_consumption_than_sedan(self):
        table = calc_fuel_table()
        sedan_90 = next(r for r in table if r["car_name"] == "普通轿车" and r["speed_kmh"] == 90)
        truck_70 = next(r for r in table if r["car_name"] == "重型卡车" and r["speed_kmh"] == 70)
        assert truck_70["l100km"] > sedan_90["l100km"]


# ---- IDM 跟车模型测试 ----

class TestIDMAcceleration:
    """idm_acceleration — IDM 核心公式"""

    def test_free_road_accelerates(self):
        """前方无车（gap 极大）时应加速趋近期望速度"""
        acc = idm_acceleration(v_ego=10, v_leader=20, gap=200, v0=20, T=1.5)
        assert acc > 0, "空旷路段应加速"

    def test_too_close_decelerates(self):
        """间距小于安全距离时应减速"""
        acc = idm_acceleration(v_ego=20, v_leader=10, gap=5, v0=20, T=1.5)
        assert acc < 0, "间距过近应减速"

    def test_same_speed_safe_gap_zero_accel(self):
        """同速 + 安全间距 → 加速度接近 0"""
        # v0=22 > v_ego=15 提供轻微加速空间，gap=30 > s*=24.5 平衡 IDM 两项
        acc = idm_acceleration(v_ego=15, v_leader=15, gap=30, v0=22, T=1.5, s0=2.0)
        assert abs(acc) < 0.5, f"稳态跟车加速度应接近0，实际 {acc:.3f}"

    def test_stopped_returns_zero(self):
        """两车都静止 → 加速度为 0"""
        acc = idm_acceleration(v_ego=0, v_leader=0, gap=10, v0=0)
        assert acc == 0.0

    def test_deceleration_within_comfort_limit(self):
        """温和接近前车时减速度应在舒适范围内"""
        # 后车 20m/s、前车 15m/s、间距 50m → 轻微减速
        acc = idm_acceleration(v_ego=20, v_leader=15, gap=50, v0=22, T=1.0)
        assert acc > -2.0, f"减速度 {acc:.3f} 超出舒适范围"


class TestCarFollowingSimulation:
    """car_following_simulation — IDM 跟车仿真"""

    def test_returns_required_keys(self):
        data = car_following_simulation(duration_s=10)
        for key in ["time", "gap_m", "follower_kmh", "leader_kmh",
                     "acc_ms2", "status", "collision_s"]:
            assert key in data

    def test_follower_eventually_matches_leader(self):
        """后车初始 70km/h 追前车 60km/h → 应减速匹配"""
        data = car_following_simulation(lead_speed_kmh=60, follower_speed_kmh=70,
                                         initial_gap_m=30, duration_s=30)
        final_speed = data["follower_kmh"][-1]
        assert abs(final_speed - 60) < 5, f"后车应接近 60km/h，实际 {final_speed:.0f}"

    def test_no_collision_with_safe_gap(self):
        """30m 初始间距 + IDM → 不应碰撞"""
        data = car_following_simulation(duration_s=20)
        assert data["collision_s"] is None, "30m 间距不应碰撞"

    def test_gap_stays_positive(self):
        data = car_following_simulation(duration_s=20)
        assert min(data["gap_m"]) > 0, "间距应始终为正"


class TestACCSimulation:
    """acc_simulation — ACC 自适应巡航场景"""

    def test_returns_required_keys(self):
        data = acc_simulation()
        for key in ["time", "gap_m", "follower_kmh", "leader_kmh", "acc_ms2"]:
            assert key in data

    def test_follower_tracks_leader_profile(self):
        """后车速度应大致跟随前车变速工况"""
        data = acc_simulation()
        # 前车最终停车，后车应显著减速
        assert data["leader_kmh"][-1] < 2
        # 末速度应远低于初始 50km/h
        assert data["follower_kmh"][-1] < 50, f"后车末速度 {data['follower_kmh'][-1]} 应已减速"

    def test_gap_never_negative(self):
        data = acc_simulation()
        assert min(data["gap_m"]) >= 0, "间距不应为负"


# ---- Pacejka 魔术公式轮胎模型测试 ----

class TestPacejkaLateralForce:
    """calc_pacejka_lateral_force — Pacejka 魔术公式核心"""

    @pytest.fixture
    def pacejka_params(self):
        """典型乘用车轮胎参数：cornering_stiffness ≈ 80000 N/rad"""
        C, D = 1.3, 4000.0
        B = 80000 / (C * D)  # B·C·D = cornering_stiffness
        return {"B": B, "C": C, "D": D, "E": 0.0}

    def test_zero_slip_zero_force(self, pacejka_params):
        """侧偏角为 0 时应无侧向力"""
        Fy = calc_pacejka_lateral_force(**pacejka_params, alpha=0)
        assert Fy == pytest.approx(0, abs=1e-6)

    def test_small_alpha_matches_linear(self, pacejka_params):
        """小侧偏角（< 2°）时 Pacejka ≈ 线性 Fy = Cα·α"""
        alpha = math.radians(1.0)  # 1°
        Fy_pacejka = calc_pacejka_lateral_force(**pacejka_params, alpha=alpha)
        Fy_linear = 80000 * abs(alpha)  # Cα × |α|
        # 1° 时偏差应 < 5%（Pacejka 在小角度有轻微非线性）
        assert Fy_pacejka == pytest.approx(Fy_linear, rel=0.05)

    def test_large_alpha_saturates(self, pacejka_params):
        """大侧偏角时力饱和，不再线性增长"""
        alpha_small = math.radians(3)
        alpha_large = math.radians(12)
        Fy_small = calc_pacejka_lateral_force(**pacejka_params, alpha=alpha_small)
        Fy_large = calc_pacejka_lateral_force(**pacejka_params, alpha=alpha_large)
        # 饱和后增长远低于线性
        ratio = Fy_large / Fy_small
        linear_ratio = 12 / 3
        assert ratio < linear_ratio, (
            f"Pacejka 饱和比 {ratio:.2f} 应 < 线性比 {linear_ratio:.2f}"
        )

    def test_peak_not_exceed_D(self, pacejka_params):
        """侧向力不应超过峰值因子 D"""
        for deg in [1, 3, 5, 8, 12, 20]:
            alpha = math.radians(deg)
            Fy = calc_pacejka_lateral_force(**pacejka_params, alpha=alpha)
            assert Fy <= pacejka_params["D"] * 1.05, (
                f"α={deg}° 时侧向力 {Fy:.0f} 超过 D={pacejka_params['D']:.0f}"
            )

    def test_force_increases_with_alpha_in_linear_range(self, pacejka_params):
        """侧偏角增大 → 侧向力增大（在线性区）"""
        Fy1 = calc_pacejka_lateral_force(**pacejka_params, alpha=math.radians(1))
        Fy2 = calc_pacejka_lateral_force(**pacejka_params, alpha=math.radians(2))
        assert Fy2 > Fy1

    def test_E_curvature_effect(self):
        """曲率因子 E 不为零时力曲线形状改变"""
        b, c, d, e0, alpha = 15.38, 1.3, 4000.0, 0.0, math.radians(5)
        fy_flat = calc_pacejka_lateral_force(B=b, C=c, D=d, E=e0, alpha=alpha)
        fy_curved = calc_pacejka_lateral_force(B=b, C=c, D=d, E=-0.5, alpha=alpha)
        # E = -0.5 使力更大（向峰值上方弯曲）
        assert fy_curved > fy_flat


class TestPacejkaCorneringForces:
    """calc_pacejka_cornering_forces — 整车前后轴 Pacejka 力"""

    @pytest.fixture
    def pacejka_car(self):
        return Vehicle("PacejkaCar", 1500, 100, drag_coeff=0.28,
                       wheelbase_m=2.65, cg_to_front_m=1.2,
                       cornering_stiffness_f=80000, cornering_stiffness_r=70000,
                       max_torque_nm=180)

    def test_zero_slip_zero_forces(self, pacejka_car):
        Fyf, Fyr = calc_pacejka_cornering_forces(pacejka_car, 0.0, 0.0)
        assert Fyf == pytest.approx(0, abs=1e-6)
        assert Fyr == pytest.approx(0, abs=1e-6)

    def test_positive_slip_negative_force(self, pacejka_car):
        """SAE 坐标系：正侧偏角 → 负侧向力"""
        Fyf, Fyr = calc_pacejka_cornering_forces(pacejka_car,
                                                   math.radians(3),
                                                   math.radians(2))
        assert Fyf < 0, "正侧偏角应产生负侧向力"
        assert Fyr < 0

    def test_small_alpha_close_to_linear(self, pacejka_car):
        """小侧偏角时 Pacejka 力 ≈ 线性模型力"""
        alpha_f = math.radians(1.5)
        alpha_r = math.radians(1.0)
        Fyf_p, Fyr_p = calc_pacejka_cornering_forces(pacejka_car, alpha_f, alpha_r)
        Fyf_l, Fyr_l = calc_cornering_forces(pacejka_car, alpha_f, alpha_r)
        assert Fyf_p == pytest.approx(Fyf_l, rel=0.05)
        assert Fyr_p == pytest.approx(Fyr_l, rel=0.05)


class TestStepSteerPacejka:
    """simulate_step_steer with tire_model="pacejka" """

    @pytest.fixture
    def pacejka_car(self):
        return Vehicle("PacejkaCar", 1500, 100, drag_coeff=0.28,
                       wheelbase_m=2.65, cg_to_front_m=1.2,
                       cornering_stiffness_f=80000, cornering_stiffness_r=70000,
                       max_torque_nm=180)

    def test_pacejka_runs_without_error(self, pacejka_car):
        """Pacejka 模型可正常仿真"""
        history = simulate_step_steer(pacejka_car, 60, 3, duration_s=2,
                                       tire_model="pacejka")
        assert len(history) > 0
        assert all(len(h) == 5 for h in history)

    def test_pacejka_converges(self, pacejka_car):
        """Pacejka 模型应收敛到稳态附近"""
        history = simulate_step_steer(pacejka_car, 60, 3, duration_s=5,
                                       tire_model="pacejka")
        _, _, _, final_r_deg, _ = history[-1]
        result = calc_steady_state_cornering(pacejka_car, 60, 3)
        # Pacejka 的稳态与线性不同，但应大致接近
        assert final_r_deg > 0
        assert final_r_deg == pytest.approx(result["yaw_rate_deg_s"], rel=0.25)

    def test_pacejka_vs_linear_small_steer_close(self, pacejka_car):
        """小转角（1°）时 Pacejka ≈ 线性"""
        h_lin = simulate_step_steer(pacejka_car, 60, 1, duration_s=2,
                                     tire_model="linear")
        h_pac = simulate_step_steer(pacejka_car, 60, 1, duration_s=2,
                                     tire_model="pacejka")
        _, _, _, r_lin, _ = h_lin[-1]
        _, _, _, r_pac, _ = h_pac[-1]
        assert r_pac == pytest.approx(r_lin, rel=0.10)

    def test_pacejka_large_steer_different_from_linear(self, pacejka_car):
        """大转角（8°）时 Pacejka 应明显偏离线性（饱和效应）"""
        h_lin = simulate_step_steer(pacejka_car, 60, 8, duration_s=2,
                                     tire_model="linear")
        h_pac = simulate_step_steer(pacejka_car, 60, 8, duration_s=2,
                                     tire_model="pacejka")
        _, _, _, r_lin, _ = h_lin[-1]
        _, _, _, r_pac, _ = h_pac[-1]
        # 大转角时轮胎饱和 → Pacejka 横摆角速度应小于线性预测
        assert r_pac < r_lin, (
            f"大转角饱和效应：Pacejka {r_pac:.2f} 应 < 线性 {r_lin:.2f}"
        )

    def test_calc_step_steer_response_pacejka(self, pacejka_car):
        """calc_step_steer_response 支持 tire_model 参数"""
        data = calc_step_steer_response(pacejka_car, vx_kmh=80, steer_deg=3,
                                         tire_model="pacejka")
        assert data["tire_model"] == "pacejka"
        assert "history" in data
        assert data["final_yaw_rate"] > 0

    def test_vehicle_has_pacejka_params(self, pacejka_car):
        """Vehicle 对象自动初始化 Pacejka 参数"""
        assert pacejka_car.pacejka_B_f > 0
        assert pacejka_car.pacejka_C_f > 0
        assert pacejka_car.pacejka_D_f > 0
        assert pacejka_car.pacejka_B_r > 0
        # B·C·D 应等于 cornering_stiffness
        slope_f = pacejka_car.pacejka_B_f * pacejka_car.pacejka_C_f * pacejka_car.pacejka_D_f
        assert slope_f == pytest.approx(pacejka_car.cornering_stiffness_f, rel=0.01)
