"""Controls shared by replay adapters; no model outputs or force calculations."""
def scenario_rows():
    """Six maneuvers at two step sizes: 12 cases, 18,600 samples."""
    for dt in (.01, .005):
        for name, duration, initial_vx in (
            ("launch", 30, 0), ("coast", 10, 20), ("brake", 7, 20),
            ("left", 5, 20), ("right", 5, 20), ("stop_turn", 5, 3),
        ):
            for step in range(1, round(duration / dt) + 1):
                t = (step - 1) * dt
                throttle = .5 if name == "launch" else .2 if name in ("left", "right") else 0.
                brake = .8 if name == "brake" else .3 if name == "stop_turn" else 0.
                steer = (.02 if name == "left" else -.02 if name == "right" else 0.) if t >= 1 else 0.
                if name == "stop_turn":
                    steer = .05
                yield dict(case=f"{name}_{dt}", step=step, dt=dt, initial_vx=initial_vx,
                           throttle=throttle, brake=brake, steer=steer)
