"""Load separate visual and collision meshes."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).with_name("model.xml")


def run(duration: float | None, headless: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    print(f"nmesh={model.nmesh} ngeom={model.ngeom}")
    for geom_name in ("visual_mesh", "collision_mesh"):
        geom_id = model.geom(geom_name).id
        print(
            f"{geom_name}: group={model.geom_group[geom_id]} "
            f"contype={model.geom_contype[geom_id]} "
            f"conaffinity={model.geom_conaffinity[geom_id]}"
        )

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
    print(f"final_time={data.time:.6f} s contacts={data.ncon}")


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
