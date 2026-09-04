"""Open the differential-drive robot and control both wheels with sliders."""

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
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)

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

    position = data.sensor("base_position").data.copy()
    orientation = data.sensor("base_orientation").data.copy()
    wheel_speed = [
        float(data.sensor("left_wheel_speed").data[0]),
        float(data.sensor("right_wheel_speed").data[0]),
    ]
    print(
        f"simulated_time={data.time:.3f} s position_world_m={position} "
        f"wheel_speed_rad_s={wheel_speed}"
    )
    print(
        f"orientation_quat={orientation} "
        f"gyro_rad_s={data.sensor('base_gyro').data.copy()}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    duration = args.duration if args.duration is not None else (1.0 if args.headless else None)
    simulate(duration, args.headless)


if __name__ == "__main__":
    main()
