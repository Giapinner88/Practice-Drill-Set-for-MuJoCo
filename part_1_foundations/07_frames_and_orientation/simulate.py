"""Inspect compiled world orientations for several MJCF representations."""

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
    mujoco.mj_forward(model, data)
    for body_name in ("reference_frame", "ref", "box1"):
        body_id = model.body(body_name).id
        print(f"{body_name}: xpos={data.xpos[body_id]} xquat={data.xquat[body_id]}")
    for geom_name in ("axisangle_box", "quaternion_box", "euler_box", "zaxis_box"):
        geom_id = model.geom(geom_name).id
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        print(f"{geom_name}: world_rotation=\n{rotation}")

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
    print(f"final_time={data.time:.6f} s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    run(args.duration, args.headless)


if __name__ == "__main__":
    main()
