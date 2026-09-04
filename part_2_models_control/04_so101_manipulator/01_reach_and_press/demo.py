"""Run the complete SO-101 Cartesian reach-and-press demo."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np

from controller import (
    APPROACH_RADIUS_M,
    PRESS_DEPTH_M,
    ReachPressController,
)

LESSON_DIR = Path(__file__).resolve().parent
MODEL_PATH = LESSON_DIR / "model.xml"
ARTIFACT_DIR = LESSON_DIR / "artifacts"


def simulate(duration: float | None, headless: bool) -> dict[str, np.ndarray | float]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    controller = ReachPressController(model, data)
    log: dict[str, list[float]] = {}

    def step() -> None:
        for name, value in controller.apply(model, data).items():
            log.setdefault(name, []).append(value)
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

    result: dict[str, np.ndarray | float] = {
        name: np.asarray(values) for name, values in log.items()
    }
    result["success_time"] = controller.success_time
    result["timestep"] = float(model.opt.timestep)
    return result


def save_plot(result: dict[str, np.ndarray | float]) -> Path:
    ARTIFACT_DIR.mkdir(exist_ok=True)
    output_path = ARTIFACT_DIR / "reach_press_diagnostics.png"
    t = np.asarray(result["time"])
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)

    axes[0, 0].plot(t, np.asarray(result["distance"]))
    axes[0, 0].axhline(APPROACH_RADIUS_M, color="tab:red", linestyle="--")
    axes[0, 0].set(xlabel="time [s]", ylabel="distance [m]", title="Active target error")

    axes[0, 1].plot(t, 1000.0 * np.asarray(result["button_depth"]), label="depth [mm]")
    axes[0, 1].plot(t, np.asarray(result["button_force"]), label="force [N]")
    axes[0, 1].axhline(1000.0 * PRESS_DEPTH_M, color="tab:red", linestyle="--")
    axes[0, 1].set(xlabel="time [s]", title="Button measurements")
    axes[0, 1].legend()

    axes[1, 0].plot(t, np.asarray(result["minimum_singular_value"]), label="sigma_min")
    axes[1, 0].plot(t, np.asarray(result["condition_number"]) / 100.0, label="condition / 100")
    axes[1, 0].set(xlabel="time [s]", title="Jacobian conditioning")
    axes[1, 0].legend()

    axes[1, 1].plot(t, np.asarray(result["maximum_actuator_force"]), label="max force [N*m]")
    axes[1, 1].plot(t, np.asarray(result["phase"]), label="phase")
    axes[1, 1].plot(t, np.asarray(result["success"]), label="success")
    axes[1, 1].set(xlabel="time [s]", title="Control and task state")
    axes[1, 1].legend()

    for axis in axes.flat:
        axis.grid(True)
    figure.savefig(output_path, dpi=150)
    plt.close(figure)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    duration = args.duration if args.duration is not None else (4.0 if args.headless else None)
    result = simulate(duration, args.headless)
    plot_path = save_plot(result)

    success_time = float(result["success_time"])
    print(
        f"simulated_time={np.asarray(result['time'])[-1]:.3f} s "
        f"timestep={float(result['timestep']):.4f} s "
        f"success={np.isfinite(success_time)} success_time={success_time:.3f} s"
    )
    print(
        f"max_button_depth={np.max(result['button_depth']):.6f} m "
        f"max_button_force={np.max(result['button_force']):.6f} N "
        f"min_joint_margin={np.min(result['minimum_joint_margin']):.6f} rad "
        f"actuator_saturation_fraction={np.mean(result['saturated']):.6f}"
    )
    print(
        f"minimum_jacobian_singular_value={np.min(result['minimum_singular_value']):.6f} "
        f"maximum_condition_number={np.max(result['condition_number']):.6f}"
    )
    print(f"plot={plot_path}")


if __name__ == "__main__":
    main()
