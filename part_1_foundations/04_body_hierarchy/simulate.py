"""Inspect generalized and Cartesian state for a free body."""

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
    print(f"nq={model.nq} nv={model.nv} qpos={data.qpos.copy()}")
    print(f"initial_xpos={data.xpos[body_id].copy()}")

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
    print(f"final_xpos={data.xpos[body_id].copy()} final_time={data.time:.6f} s")


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
