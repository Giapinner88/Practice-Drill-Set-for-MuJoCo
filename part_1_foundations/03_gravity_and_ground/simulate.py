"""Drop a free body under gravity and report its final height."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).with_name("model.xml")


def run(duration: float, headless: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    body_id = model.body("falling_box").id
    mujoco.mj_forward(model, data)
    initial_z = float(data.xpos[body_id, 2])

    if headless:
        while data.time < duration:
            mujoco.mj_step(model, data)
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time < duration:
                step_start = time.perf_counter()
                mujoco.mj_step(model, data)
                viewer.sync()
                delay = model.opt.timestep - (time.perf_counter() - step_start)
                if delay > 0:
                    time.sleep(delay)

    print(
        f"initial_z={initial_z:.6f} m final_z={data.xpos[body_id, 2]:.6f} m "
        f"contacts={data.ncon} final_time={data.time:.6f} s"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    run(args.duration, args.headless)


if __name__ == "__main__":
    main()
