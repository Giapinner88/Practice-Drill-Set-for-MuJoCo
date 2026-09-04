"""Open the quadrotor at hover and control its four rotor thrusts with sliders."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).with_name("model.xml")


def simulate(duration: float | None, headless: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("hover").id)

    if headless:
        assert duration is not None
        while data.time < duration:
            mujoco.mj_step(model, data)
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and (duration is None or data.time < duration):
                start = time.perf_counter()
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - start)))

    print(
        f"simulated_time={data.time:.3f} s "
        f"position_world_m={data.sensor('position').data.copy()} "
        f"linear_velocity_world_m_s={data.sensor('linear_velocity').data.copy()}"
    )
    print(
        f"orientation_quat={data.sensor('orientation').data.copy()} "
        f"angular_velocity_body_rad_s={data.sensor('angular_velocity').data.copy()} "
        f"specific_force_body_m_s2={data.sensor('specific_force').data.copy()}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    duration = args.duration if args.duration is not None else (2.0 if args.headless else None)
    simulate(duration, args.headless)


if __name__ == "__main__":
    main()
