"""Move a body with a sinusoidal control signal to inspect camera modes."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).with_name("model.xml")


def target_position(time_s: float) -> float:
    return 1.25 * math.sin(2.0 * math.pi * 0.1 * time_s)


def run(duration: float | None, headless: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    slider_id = model.joint("slider").id
    qpos_adr = model.jnt_qposadr[slider_id]
    actuator_id = model.actuator("slide_position").id
    squared_error_sum = 0.0
    sample_count = 0
    max_abs_position = 0.0
    print(f"ncam={model.ncam} nu={model.nu} nsensordata={model.nsensordata}")

    def control_and_step() -> None:
        nonlocal squared_error_sum, sample_count, max_abs_position
        target = target_position(float(data.time))
        position = float(data.qpos[qpos_adr])
        data.ctrl[actuator_id] = target
        squared_error_sum += (target - position) ** 2
        sample_count += 1
        max_abs_position = max(max_abs_position, abs(position))
        mujoco.mj_step(model, data)

    if headless:
        assert duration is not None
        while data.time < duration:
            control_and_step()
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and (duration is None or data.time < duration):
                step_start = time.perf_counter()
                control_and_step()
                viewer.sync()
                delay = model.opt.timestep - (time.perf_counter() - step_start)
                if delay > 0:
                    time.sleep(delay)
    tracking_rmse = math.sqrt(squared_error_sum / max(1, sample_count))
    print(
        f"final_time={data.time:.6f} s tracking_rmse={tracking_rmse:.6f} m "
        f"max_abs_position={max_abs_position:.6f} m sensordata={data.sensordata.copy()}"
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
