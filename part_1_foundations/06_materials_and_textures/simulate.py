"""Compare built-in and file-backed textures."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

LESSON_DIR = Path(__file__).resolve().parent
MODEL_CHOICES = ("model.xml", "file_textures.xml")


def run(model_name: str, duration: float, headless: bool) -> None:
    model_path = LESSON_DIR / model_name
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    print(f"model_file={model_name} ntex={model.ntex} nmat={model.nmat}")

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
    parser.add_argument("--model", choices=MODEL_CHOICES, default="model.xml")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    run(args.model, args.duration, args.headless)


if __name__ == "__main__":
    main()
