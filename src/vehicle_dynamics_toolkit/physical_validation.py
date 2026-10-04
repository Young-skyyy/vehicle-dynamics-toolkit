"""Independent closed-form checks; these verify equations, not real-car accuracy.

Expected values deliberately do not call the production force/integration helpers.
Tolerances cover numerical discretization and are fixed before running the checks.
"""
from __future__ import annotations
from dataclasses import asdict, dataclass
import math
from .simulation import DynamicsModel, reference_vehicle


@dataclass(frozen=True)
class CheckResult:
    case: str
    field: str
    unit: str
    actual: float
    expected: float
    tolerance: float

    @property
    def passed(self) -> bool:
        return (math.isfinite(self.actual) and math.isfinite(self.expected)
                and abs(self.actual - self.expected) <= self.tolerance)


def analytical_checks() -> list[CheckResult]:
    """Zero resistance, rolling stop, aerodynamic coast, braking and steady turn."""
    results = []
    def record(case, field, unit, actual, expected, tolerance):
        results.append(CheckResult(case, field, unit, actual, expected, tolerance))

    # Constant speed and constant deceleration have exact kinematic solutions.
    for name, initial, duration, rolling, brake in (
        ("free_motion", 20., 3., 0., 0.),
        ("rolling_coast", 20., 5., .015, 0.),
        ("rolling_stop", .3, 3., .015, 0.),
        ("brake_stop", 20., 5., 0., 1.),
    ):
        car = reference_vehicle()
        car.cd, car.rolling_coeff = 0., rolling
        model = DynamicsModel(car, initial)
        dt = .01
        for _ in range(round(duration / dt)):
            model.step(dt, 0, brake, 0)
        deceleration = (rolling + .8 * brake) * 9.8
        moving_time = min(duration, initial / deceleration) if deceleration else duration
        expected_v = max(0., initial - deceleration * duration)
        expected_x = initial * moving_time - .5 * deceleration * moving_time**2
        record(name, "vx", "m/s", model.state.vx, expected_v, 1e-10)
        record(name, "position_x", "m", model.state.position_x, expected_x, 1e-9)

    # dv/dt=-k*v^2: v=v0/(1+k*v0*t), x=log(1+k*v0*t)/k.
    aero_errors = []
    for dt in (.02, .01, .005):
        car = reference_vehicle()
        car.rolling_coeff = 0.
        k = .5 * 1.225 * car.cd * car.area / car.mass
        model = DynamicsModel(car, 30.)
        duration = 10.
        for _ in range(round(duration / dt)):
            model.step(dt, 0, 0, 0)
        expected_v = 30. / (1 + k * 30. * duration)
        expected_x = math.log1p(k * 30. * duration) / k
        record(f"aero_coast_dt_{dt}", "vx", "m/s", model.state.vx, expected_v, .003)
        record(f"aero_coast_dt_{dt}", "position_x", "m", model.state.position_x, expected_x, .02)
        aero_errors.append(abs(model.state.vx - expected_v))
    # First-order integration: halving dt should halve the error (allow 5%).
    for i in (0, 1):
        record(f"aero_convergence_{i}", "error_ratio", "1",
               aero_errors[i + 1] / aero_errors[i], .5, .025)

    # Solve front/rear force and yaw-moment equilibrium independently.
    for speed in (5., 20., 30.):
        for steer in (-.01, .01):
            car = reference_vehicle()
            car.cd = car.rolling_coeff = 0.
            a, b, length = car.cg_to_front, car.cg_to_rear, car.wheelbase
            cf, cr, mass = car.cornering_stiffness_f, car.cornering_stiffness_r, car.mass
            yaw = speed * steer / (length + mass / length * (b / cf - a / cr) * speed**2)
            # Fyr=m*v*r*a/L and Fyr=-Cr*(vy-b*r)/v, hence v squared.
            vy = b * yaw - mass * speed**2 * yaw * a / (length * cr)
            model = DynamicsModel(car, speed)
            for _ in range(2000):
                model.step(.005, 0, 0, steer)
            name = f"steady_turn_{speed}_{steer}"
            record(name, "yaw_rate", "rad/s", model.state.yaw_rate, yaw, 1e-7)
            record(name, "vy", "m/s", model.state.vy, vy, 1e-6)
            record(name, "ay", "m/s2", model.state.ay, speed * yaw, 1e-6)
    return results


def physics_report() -> dict:
    checks = analytical_checks()
    return {"evidence": "analytical_verification", "real_vehicle_accuracy": "NOT_VALIDATED",
            "passed": all(item.passed for item in checks), "checks": len(checks),
            "results": [{**asdict(item), "passed": item.passed,
                         "abs_error": abs(item.actual - item.expected)} for item in checks]}
