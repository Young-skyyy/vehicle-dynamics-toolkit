# -*- coding: utf-8 -*-
"""
Vehicle Dynamics Engineering Toolkit
=====================================
Python library for vehicle dynamics simulation, analysis and visualization.

Modules:
    vehicle      — Vehicle physics, powertrain, longitudinal dynamics
    lateral      — Bicycle model, understeer gradient, Pacejka tire
    bsfc         — BSFC fuel map, bilinear interpolation
    wltc         — WLTC Class 3 transient cycle simulation
    can_demo     — CAN bus multi-ECU simulation, DBC export
    uds          — UDS (ISO 14229) diagnostic protocol stack
    can_bus_load — CAN bus load rate simulation
"""

# ── Vehicle ──
from .vehicle import (
    Vehicle,
    G, RHO_AIR, KMH_TO_MS, MS_TO_KMH,
    DEFAULT_ROLLING_COEFF, SECONDS_PER_HOUR, SECONDS_PER_MINUTE,
    DEFAULT_CG_FRONT_RATIO,
    car_sedan, car_suv, car_truck,
    get_default_vehicles,
    get_engine_torque,
    calc_resistance,
    calc_wheel_force,
    calc_acceleration,
    simulate_acceleration,
    calc_braking_distance,
    calc_braking_table,
    calc_grade_power,
    calc_power_to_weight,
    calc_aero_drag_power,
    calc_power_breakdown,
    rolling_coeff_dynamic,
    idm_acceleration,
    car_following_simulation,
    acc_simulation,
)

# ── Lateral Dynamics ──
from .lateral_dynamics import (
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

# ── BSFC ──
from .bsfc import (
    calc_fuel_consumption,
    calc_fuel_table,
    _calc_l100_raw,
    _interpolate_bsfc,
)

# ── WLTC ──
from .wltc import (
    get_wltc_profile,
    get_wltc_summary,
    simulate_transient_cycle,
    simulate_wltc,
)

# ── CAN ──
from .can_demo import (
    CAN_MESSAGES,
    encode_signal,
    decode_signal,
    build_can_frame,
    parse_can_frame,
    VehicleECU,
    generate_dbc,
    simulate_dtc_check,
    simulate_can_bus,
    simulate_can_bus_advanced,
)

from .can_bus_load_demo import (
    ECUS,
    frame_bits,
    simulate,
    print_simulation_result,
)

# ── UDS ──
from .uds import (
    UDSSID,
    NRC,
    DTCStatusMask,
    DTC_DATABASE,
    dtc_status_byte,
    decode_dtc_status,
    DiagnosticSession,
    ECUDiagnosticServer,
    run_diagnostic_session,
    print_diagnostic_session,
)

# ── Visualization ──
from .plotting import plot_bsfc_map
from .plot_dashboard import plot_dashboard

__version__ = "2.0.0"
