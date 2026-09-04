"""Open SO-101 at home and control its six actuators with viewer sliders."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).resolve().parent / "model.xml"


def simulate(duration: float | None, headless: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)

    def step() -> None:
        # In viewer mode, the Control sliders write directly to data.ctrl.
        mujoco.mj_step(model, data)

    if headless:
        assert duration is not None
        while data.time < duration:
            step()
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and (duration is None or data.time < duration):
                start = time.perf_counter()
                step()
                viewer.sync()
                delay = model.opt.timestep - (time.perf_counter() - start)
                if delay > 0:
                    time.sleep(delay)

    print(
        f"simulated_time={data.time:.3f} s timestep={model.opt.timestep:.4f} s "
        f"nq={model.nq} nv={model.nv} nu={model.nu}"
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
