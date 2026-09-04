"""Drop a free body under gravity and report its final height."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).with_name("model.xml")


def run(duration: float | None, headless: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    body_id = model.body("falling_box").id
    mujoco.mj_forward(model, data)
    initial_z = float(data.xpos[body_id, 2])
    validation_time = min(0.1, duration) if duration is not None else 0.1
    measured_z: float | None = None
    contacts_at_validation: int | None = None

    def step_and_measure() -> None:
        nonlocal measured_z, contacts_at_validation
        mujoco.mj_step(model, data)
        if measured_z is None and data.time >= validation_time:
            measured_z = float(data.xpos[body_id, 2])
            contacts_at_validation = int(data.ncon)

    if headless:
        assert duration is not None
        while data.time < duration:
            step_and_measure()
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and (duration is None or data.time < duration):
                step_start = time.perf_counter()
                step_and_measure()
                viewer.sync()
                delay = model.opt.timestep - (time.perf_counter() - step_start)
                if delay > 0:
                    time.sleep(delay)

    print(
        f"initial_z={initial_z:.6f} m final_z={data.xpos[body_id, 2]:.6f} m "
        f"contacts={data.ncon} final_time={data.time:.6f} s"
    )
    if measured_z is None:
        print("free_fall_check=not_reached")
        return
    expected_z = initial_z - 0.5 * 9.81 * validation_time**2
    ballistic_error = measured_z - expected_z
    print(
        f"free_fall_check_time={validation_time:.6f} s measured_z={measured_z:.6f} m "
        f"expected_z={expected_z:.6f} m error={ballistic_error:.6e} m "
        f"contacts={contacts_at_validation}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, help="simulation seconds; viewer is unlimited by default")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    duration = args.duration if args.duration is not None else (5.0 if args.headless else None)
    run(duration, args.headless)


if __name__ == "__main__":
    main()
