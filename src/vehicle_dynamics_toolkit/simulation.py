"""Deterministic fixed-step reference for the C++ ROS2 dynamics core.

SI units throughout; steering is the front-wheel angle. The linear bicycle
model uses a kinematic approximation below 1 m/s. No combined slip, load
transfer or physical clutch transient is modeled.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

from .vehicle import Vehicle, MS_TO_KMH, calc_acceleration, get_engine_torque, _advance_speed
from .lateral_dynamics import integrate_lateral, calc_slip_angles, calc_cornering_forces


def reference_vehicle() -> Vehicle:
    """Defaults shared by the comparison cases and ROS2 node."""
    return Vehicle("RefSedan", 1500, 140, drag_coeff=0.30,
                   max_torque_nm=250, idle_rpm=800, max_rpm=6200,
                   gear_ratios=[3.55, 2.11, 1.42, 1.00, 0.78],
                   final_drive=4.06, wheel_radius_m=0.32, trans_efficiency=0.90,
                   wheelbase_m=2.65, cg_to_front_m=1.2,
                   cornering_stiffness_f=80000, cornering_stiffness_r=70000)


@dataclass
class DynamicsState:
    t: float = 0.0
    vx: float = 0.0
    ax: float = 0.0
    vy: float = 0.0
    ay: float = 0.0  # physical body-frame acceleration, dvy/dt + vx*yaw_rate
    yaw_rate: float = 0.0
    heading: float = 0.0
    position_x: float = 0.0  # inertial/world coordinates
    position_y: float = 0.0
    gear: int = 0
    engine_rpm: float = 0.0
    engine_torque: float = 0.0


class DynamicsModel:
    def __init__(self, vehicle: Vehicle | None = None, initial_vx: float = 0.0):
        if not math.isfinite(initial_vx) or initial_vx < 0:
            raise ValueError("initial_vx must be finite and non-negative")
        self.vehicle = vehicle or reference_vehicle()
        self.state = DynamicsState(vx=initial_vx)

    def step(self, dt: float, throttle: float, brake: float, steer: float) -> dict:
        if not all(math.isfinite(x) for x in (dt, throttle, brake, steer)) or not 0 < dt <= 0.1:
            raise ValueError("finite controls and 0 < dt <= 0.1 are required")
        throttle, brake = max(0.0, min(1.0, throttle)), max(0.0, min(1.0, brake))
        steer = max(-0.7, min(0.7, steer))
        s, car = self.state, self.vehicle
        old_vx, old_vy, old_heading = s.vx, s.vy, s.heading
        ax = calc_acceleration(car, s.vx, throttle=throttle, brake=brake)
        s.vx, distance = _advance_speed(s.vx, ax, dt)
        s.ax = (s.vx - old_vx) / dt
        if s.vx < 1.0:
            s.yaw_rate = s.vx * math.tan(steer) / car.wheelbase
            s.vy = car.cg_to_rear * s.yaw_rate
            s.ay = s.vx * s.yaw_rate
        else:
            s.vy, s.yaw_rate = integrate_lateral(car, s.vx, s.vy, s.yaw_rate, steer, dt)
            af, ar = calc_slip_angles(car, s.vx, s.vy, s.yaw_rate, steer)
            fyf, fyr = calc_cornering_forces(car, af, ar)
            s.ay = (fyf + fyr) / car.mass
        s.heading += s.yaw_rate * dt
        middle_heading = 0.5 * (old_heading + s.heading)
        lateral_distance = 0.5 * (old_vy + s.vy) * dt
        s.position_x += distance * math.cos(middle_heading) - lateral_distance * math.sin(middle_heading)
        s.position_y += distance * math.sin(middle_heading) + lateral_distance * math.cos(middle_heading)
        s.gear = car.select_gear(s.vx * MS_TO_KMH) if s.vx > 0 else (1 if throttle > 0 else 0)
        if s.gear:
            ratio = car.gear_ratios[s.gear - 1] * car.final_drive
            s.engine_rpm = max(car.idle_rpm, min(car.max_rpm, s.vx * ratio * 60 / (2 * math.pi * car.wheel_radius)))
        else:
            s.engine_rpm = 0.0
        s.engine_torque = get_engine_torque(s.engine_rpm, throttle, car.torque_curve)
        s.t += dt
        return asdict(s)
