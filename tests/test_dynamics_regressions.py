"""Physical invariants and failure cases missed by the original demo tests."""
import math
import pytest
from vehicle_dynamics_toolkit.vehicle import (
    Vehicle, G, calc_acceleration, calc_resistance, simulate_acceleration,
    acc_simulation, car_following_simulation, idm_acceleration, _advance_speed,
)
from vehicle_dynamics_toolkit.lateral_dynamics import (
    calc_steady_state_cornering, simulate_step_steer, calc_critical_speed,
)
from vehicle_dynamics_toolkit.simulation import DynamicsModel, reference_vehicle


def test_coasting_obeys_force_balance():
    car = reference_vehicle()
    assert calc_acceleration(car, 20, throttle=0) == pytest.approx(-calc_resistance(car, 20) / car.mass)


def test_acceleration_starts_at_rest_and_reports_timeout():
    result = simulate_acceleration(reference_vehicle(), 30)
    assert result["time"][0] == result["speed_kmh"][0] == result["distance_m"][0] == 0
    assert result["reached_target"] and result["status"] == "reached_target"
    assert result["speed_kmh"][-1] == pytest.approx(30)
    weak = Vehicle("weak", 1500, 1, max_torque_nm=1)
    result = simulate_acceleration(weak, 100, dt=.1, max_time_s=2)
    assert result["status"] == "timeout" and not result["reached_target"]
    assert result["elapsed_s"] == 2
    assert max(result["speed_kmh"]) == 0


@pytest.mark.parametrize("dt", [0, -1, float("nan"), float("inf")])
def test_invalid_steps_fail_instead_of_hanging(dt):
    with pytest.raises(ValueError):
        simulate_acceleration(reference_vehicle(), dt=dt)
    with pytest.raises(ValueError):
        acc_simulation(dt=dt)
    with pytest.raises(ValueError):
        simulate_step_steer(reference_vehicle(), 80, 3, dt=dt)


def test_stop_inside_step_preserves_stopping_distance():
    speed, distance = _advance_speed(1, -2, 1)
    assert speed == 0
    assert distance == pytest.approx(.25)  # v² / (2a), then at rest


def test_acc_has_true_initial_sample_and_physical_actuator_limits():
    result = acc_simulation()
    assert result["time"][0] == 0
    assert result["follower_kmh"][0] == pytest.approx(50)
    assert result["gap_m"][0] == pytest.approx(40)
    assert min(result["acc_ms2"]) >= -8
    assert max(result["acc_ms2"]) <= 1.4
    assert result["follower_kmh"][1] > 45
    assert result["collision_s"] is None
    assert all(math.isfinite(x) for key in ("follower_kmh", "acc_ms2", "gap_m") for x in result[key])


def test_stopped_leader_does_not_define_free_road_setpoint():
    assert idm_acceleration(5, 0, 1000) > 0
    assert idm_acceleration(5, 0, 5) < 0


def test_collision_is_reported_not_hidden_by_teleporting():
    result = acc_simulation([(0, 0), (2, 0)], follower_v0_kmh=100, initial_gap_m=1)
    assert result["collision_s"] is not None
    assert min(result["gap_m"]) < 0


@pytest.mark.parametrize("profile", [[], [(0, 0)], [(1, 0), (2, 10)], [(0, 0), (0, 10)], [(0, 0), (1, -10)]])
def test_invalid_lead_profiles(profile):
    with pytest.raises(ValueError):
        acc_simulation(profile)


def test_turns_are_symmetric_and_standstill_is_defined():
    car = reference_vehicle()
    left = calc_steady_state_cornering(car, 80, 3)
    right = calc_steady_state_cornering(car, 80, -3)
    assert left["turn_radius_m"] == pytest.approx(right["turn_radius_m"])
    assert left["yaw_rate_deg_s"] == pytest.approx(-right["yaw_rate_deg_s"])
    stopped = calc_steady_state_cornering(car, 0, 3)
    assert stopped["yaw_rate_deg_s"] == stopped["lateral_acc_g"] == 0
    assert math.isinf(stopped["turn_radius_m"])


def test_unstable_steady_state_is_rejected():
    car = Vehicle("oversteer", 1500, 100, cornering_stiffness_f=120000, cornering_stiffness_r=40000)
    critical = calc_critical_speed(car)
    assert math.isfinite(critical)
    with pytest.raises(ValueError):
        calc_steady_state_cornering(car, critical, 3)


def test_transient_acceleration_comes_from_tire_force():
    car = reference_vehicle()
    history = simulate_step_steer(car, 80, 3, duration_s=.02)
    expected_ay = car.cornering_stiffness_f * math.radians(3) / car.mass
    assert history[0]["yaw_rate_rad"] == 0
    assert history[0]["lateral_acc_g"] == pytest.approx(expected_ay / G, abs=1e-6)


@pytest.mark.parametrize("speed", [0, .1, 1, 5])
def test_low_speed_transient_is_finite(speed):
    history = simulate_step_steer(reference_vehicle(), speed, 3, duration_s=.1)
    assert all(math.isfinite(x["yaw_rate_rad"]) and abs(x["lateral_acc_g"]) < 1 for x in history)


def test_launch_coast_brake_and_park():
    model = DynamicsModel()
    for _ in range(1000):
        model.step(.01, .5, 0, 0)
    assert model.state.vx > 1
    speed = model.state.vx
    model.step(.01, 0, 0, 0)
    assert model.state.vx < speed and model.state.ax < 0
    for _ in range(1000):
        model.step(.01, 0, 1, .02)
    assert model.state.vx == model.state.vy == model.state.yaw_rate == 0
    x, y = model.state.position_x, model.state.position_y
    model.step(.01, 0, 1, .02)
    assert model.state.position_x == x and model.state.position_y == y


def test_runtime_mirror_symmetry():
    left, right = DynamicsModel(initial_vx=20), DynamicsModel(initial_vx=20)
    for _ in range(200):
        a, b = left.step(.01, .2, 0, .02), right.step(.01, .2, 0, -.02)
    for field in ("vy", "ay", "yaw_rate", "heading", "position_y"):
        assert a[field] == pytest.approx(-b[field])
    assert a["position_x"] == pytest.approx(b["position_x"])


def test_acc_time_step_convergence():
    coarse, fine = acc_simulation(dt=.1), acc_simulation(dt=.05)
    assert abs(coarse["gap_m"][-1] - fine["gap_m"][-1]) < 1
    assert abs(coarse["follower_kmh"][-1] - fine["follower_kmh"][-1]) < 1


def test_short_following_duration_is_not_extended():
    data = car_following_simulation(duration_s=.03, dt=.1)
    assert data["time"][-1] == .03


@pytest.mark.parametrize("duration,dt", [(.07, .01), (.03, .1), (0, .1)])
def test_sample_times_include_end_once(duration, dt):
    history = simulate_step_steer(reference_vehicle(), 80, 3, duration_s=duration, dt=dt)
    times = [row["time"] for row in history]
    assert times[0] == 0 and times[-1] == duration
    assert len(times) == len(set(times))
    following = car_following_simulation(duration_s=duration, dt=dt)
    assert following["time"][0] == 0 and following["time"][-1] == duration
    assert len(following["time"]) == len(set(following["time"]))
