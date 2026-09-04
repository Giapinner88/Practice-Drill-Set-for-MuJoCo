"""Simulate Acrobot swing-up and save the measured trajectory."""

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

from controller import AcrobotController, TORQUE_LIMIT_NM

LESSON_DIR = Path(__file__).resolve().parent
MODEL_PATH = LESSON_DIR / "model.xml"
ARTIFACT_DIR = LESSON_DIR / "artifacts"
GAIN_PATH = ARTIFACT_DIR / "lqr_gain.npy"


def simulate(
    duration: float | None,
    headless: bool,
    initial_angle: float,
) -> dict[str, np.ndarray | float]:
    if not GAIN_PATH.exists():
        raise FileNotFoundError(f"missing {GAIN_PATH}; run linearize_lqr.py first")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    controller = AcrobotController(model, np.load(GAIN_PATH))
    data.qpos[controller.q1_adr] = initial_angle
    mujoco.mj_forward(model, data)
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
                time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - start)))

    result: dict[str, np.ndarray | float] = {
        name: np.asarray(values) for name, values in log.items()
    }
    result["target_energy"] = controller.target_energy
    result["first_lqr_time"] = controller.first_balance_time
    result["saturation_fraction"] = controller.saturated_steps / len(log["time"])
    result["timestep"] = float(model.opt.timestep)
    return result


def save_plot(result: dict[str, np.ndarray | float]) -> Path:
    ARTIFACT_DIR.mkdir(exist_ok=True)
    output_path = ARTIFACT_DIR / "acrobot_diagnostics.png"
    t = np.asarray(result["time"])
    e1 = np.asarray(result["shoulder_error"])

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axes[0, 0].plot(e1, np.asarray(result["shoulder_velocity"]))
    axes[0, 0].set(
        xlabel="shoulder upright error [rad]",
        ylabel="shoulder velocity [rad/s]",
        title="Shoulder phase portrait",
    )
    axes[0, 1].plot(t, np.asarray(result["energy"]), label="E(t)")
    axes[0, 1].axhline(
        float(result["target_energy"]), color="tab:red", linestyle="--", label="E upright"
    )
    axes[0, 1].set(xlabel="time [s]", ylabel="energy [J]", title="Mechanical energy")
    axes[0, 1].legend()
    axes[1, 0].plot(t, e1, label="q1 - pi")
    axes[1, 0].plot(t, np.asarray(result["elbow_error"]), label="q2")
    axes[1, 0].plot(t, np.asarray(result["mode"]), label="LQR mode")
    axes[1, 0].set(xlabel="time [s]", ylabel="angle [rad]", title="Hybrid mode")
    axes[1, 0].legend()
    axes[1, 1].plot(t, np.asarray(result["torque"]))
    axes[1, 1].axhline(TORQUE_LIMIT_NM, color="tab:red", linestyle="--")
    axes[1, 1].axhline(-TORQUE_LIMIT_NM, color="tab:red", linestyle="--")
    axes[1, 1].set(xlabel="time [s]", ylabel="torque [N*m]", title="Elbow actuation")
    for axis in axes.flat:
        axis.grid(True)
    figure.savefig(output_path, dpi=150)
    plt.close(figure)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--initial-angle", type=float, default=0.1)
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")

    duration = args.duration if args.duration is not None else (20.0 if args.headless else None)
    result = simulate(duration, args.headless, args.initial_angle)
    plot_path = save_plot(result)
    velocity = np.column_stack(
        [result["shoulder_velocity"], result["elbow_velocity"]]
    )
    print(
        f"simulated_time={np.asarray(result['time'])[-1]:.3f} s "
        f"timestep={float(result['timestep']):.4f} s "
        f"initial_angle={args.initial_angle:.3f} rad "
        f"torque_limit={TORQUE_LIMIT_NM:.1f} N*m "
        f"first_lqr_time={float(result['first_lqr_time']):.3f} s "
        f"max_abs_velocity={np.max(np.abs(velocity)):.6f} rad/s"
    )
    print(
        f"final_errors=[{abs(np.asarray(result['shoulder_error'])[-1]):.3e}, "
        f"{abs(np.asarray(result['elbow_error'])[-1]):.3e}] rad "
        f"saturation_fraction={float(result['saturation_fraction']):.6f}"
    )
    print(f"plot={plot_path}")


if __name__ == "__main__":
    main()
