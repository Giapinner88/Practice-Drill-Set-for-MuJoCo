"""Inspect local and world poses in a three-level body hierarchy."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

MODEL_PATH = Path(__file__).with_name("model.xml")
BODY_NAMES = ("base", "arm_link", "tool")


def print_tree(model: mujoco.MjModel) -> None:
    for body_name in BODY_NAMES:
        body_id = model.body(body_name).id
        parent_id = int(model.body_parentid[body_id])
        parent_name = "world" if parent_id == 0 else model.body(parent_id).name
        print(
            f"body={body_name} parent={parent_name} "
            f"local_pos={model.body_pos[body_id]}"
        )


def print_world_poses(model: mujoco.MjModel, data: mujoco.MjData, label: str) -> None:
    print(label)
    for body_name in BODY_NAMES:
        body_id = model.body(body_name).id
        print(f"  {body_name}: xpos={data.xpos[body_id]} xquat={data.xquat[body_id]}")


def run(duration: float | None, headless: bool, initial_angle_deg: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    shoulder_id = model.joint("shoulder").id
    qpos_adr = model.jnt_qposadr[shoulder_id]

    print_tree(model)
    mujoco.mj_forward(model, data)
    print_world_poses(model, data, "world poses at theta=0 deg")

    data.qpos[qpos_adr] = np.deg2rad(initial_angle_deg)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    print_world_poses(model, data, f"world poses at theta={initial_angle_deg:g} deg")

    if headless:
        assert duration is not None
        while data.time < duration:
            mujoco.mj_step(model, data)
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and (duration is None or data.time < duration):
                step_start = time.perf_counter()
                mujoco.mj_step(model, data)
                viewer.sync()
                delay = model.opt.timestep - (time.perf_counter() - step_start)
                if delay > 0:
                    time.sleep(delay)

    tool_id = model.body("tool").id
    print(f"final_time={data.time:.6f} s tool_xpos={data.xpos[tool_id]}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, help="simulation seconds; viewer is unlimited by default")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--initial-angle-deg", type=float, default=45.0)
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    duration = args.duration if args.duration is not None else (5.0 if args.headless else None)
    run(duration, args.headless, args.initial_angle_deg)


if __name__ == "__main__":
    main()
