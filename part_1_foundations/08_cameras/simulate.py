"""Move a body with a sinusoidal control signal to inspect camera modes."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).with_name("model.xml")


def set_control(data: mujoco.MjData) -> None:
    data.ctrl[0] = 0.5 * math.sin(2.0 * math.pi * 0.2 * data.time)


def run(duration: float, headless: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    print(f"ncam={model.ncam} nu={model.nu} nsensordata={model.nsensordata}")

    if headless:
        while data.time < duration:
            set_control(data)
            mujoco.mj_step(model, data)
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time < duration:
                step_start = time.perf_counter()
                set_control(data)
                mujoco.mj_step(model, data)
                viewer.sync()
                delay = model.opt.timestep - (time.perf_counter() - step_start)
                if delay > 0:
                    time.sleep(delay)
    print(f"final_time={data.time:.6f} s sensordata={data.sensordata.copy()}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    run(args.duration, args.headless)


if __name__ == "__main__":
    main()
