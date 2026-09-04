"""Run quadrotor hover recovery and save measured diagnostics."""

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
    GRAVITY_M_S2,
    MASS_KG,
    MAX_ROTOR_THRUST_N,
    ROTOR_NAMES,
    HoverController,
)

LESSON_DIR = Path(__file__).resolve().parent
MODEL_PATH = LESSON_DIR / "model.xml"
ARTIFACT_DIR = LESSON_DIR / "artifacts"


def set_initial_state(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    root_id = model.joint("root").id
    qpos_adr = model.jnt_qposadr[root_id]
    data.qpos[qpos_adr : qpos_adr + 3] = [0.25, -0.20, 0.75]
    mujoco.mju_euler2Quat(
        data.qpos[qpos_adr + 3 : qpos_adr + 7],
        np.deg2rad([10.0, -8.0, 15.0]),
        "xyz",
    )
    data.qvel[:] = 0.0
    data.ctrl[:] = MASS_KG * GRAVITY_M_S2 / 4.0
    mujoco.mj_forward(model, data)


def simulate(duration: float | None, headless: bool) -> dict[str, np.ndarray | float]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    set_initial_state(model, data)
    controller = HoverController(model)
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
    result["success_time"] = controller.success_time
    result["timestep"] = float(model.opt.timestep)
    result["target_position"] = data.site_xpos[controller.target_site_id].copy()
    return result


def save_plot(result: dict[str, np.ndarray | float]) -> Path:
    ARTIFACT_DIR.mkdir(exist_ok=True)
    output_path = ARTIFACT_DIR / "hover_diagnostics.png"
    t = np.asarray(result["time"])
    target_position = np.asarray(result["target_position"])
    hover_thrust = MASS_KG * GRAVITY_M_S2 / 4.0

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for coordinate, target, label in zip(("x", "y", "z"), target_position, ("x", "y", "z")):
        axes[0, 0].plot(t, result[coordinate], label=label)
        axes[0, 0].axhline(target, color="black", linewidth=0.7, linestyle="--")
    axes[0, 0].set(xlabel="time [s]", ylabel="world position [m]", title="Position recovery")
    axes[0, 0].legend()

    axes[0, 1].plot(t, result["position_error"], label="position [m]")
    axes[0, 1].plot(t, result["velocity_norm"], label="velocity [m/s]")
    attitude_axis = axes[0, 1].twinx()
    attitude_axis.plot(
        t, np.rad2deg(result["attitude_error"]), color="tab:green", label="attitude [deg]"
    )
    axes[0, 1].set(xlabel="time [s]", ylabel="translation error", title="Hover errors")
    attitude_axis.set_ylabel("attitude error [deg]")
    error_lines = axes[0, 1].get_lines() + attitude_axis.get_lines()
    axes[0, 1].legend(error_lines, [line.get_label() for line in error_lines])

    for rotor_name in ROTOR_NAMES:
        axes[1, 0].plot(t, result[rotor_name], label=rotor_name.removeprefix("thrust_"))
    axes[1, 0].axhline(hover_thrust, color="black", linestyle="--", label="mg/4")
    axes[1, 0].axhline(MAX_ROTOR_THRUST_N, color="tab:red", linestyle=":", label="limit")
    axes[1, 0].set(xlabel="time [s]", ylabel="rotor thrust [N]", title="Rotor allocation")
    axes[1, 0].legend()

    axes[1, 1].plot(t, result["total_thrust"], label="total thrust", color="tab:blue")
    moment_axis = axes[1, 1].twinx()
    moment_axis.plot(t, result["roll_moment"], label="roll moment", color="tab:orange")
    moment_axis.plot(t, result["pitch_moment"], label="pitch moment", color="tab:green")
    moment_axis.plot(t, result["yaw_moment"], label="yaw moment", color="tab:red")
    axes[1, 1].set(xlabel="time [s]", ylabel="total thrust [N]", title="Commanded wrench")
    moment_axis.set_ylabel("moment [N m]")
    wrench_lines = axes[1, 1].get_lines() + moment_axis.get_lines()
    axes[1, 1].legend(wrench_lines, [line.get_label() for line in wrench_lines])

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
    duration = args.duration if args.duration is not None else (8.0 if args.headless else None)

    result = simulate(duration, args.headless)
    plot_path = save_plot(result)
    t = np.asarray(result["time"])
    tail = t >= max(0.0, t[-1] - 2.0)
    success_time = float(result["success_time"])
    final_position = np.array([result["x"][-1], result["y"][-1], result["z"][-1]])
    target_position = np.asarray(result["target_position"])

    print(
        f"simulated_time={t[-1]:.3f} s timestep={float(result['timestep']):.4f} s "
        f"success={np.isfinite(success_time)} success_time={success_time:.3f} s"
    )
    print(
        f"final_position_world_m={final_position} "
        f"final_position_error={np.linalg.norm(final_position - target_position):.6f} m "
        f"tail_position_rmse={np.sqrt(np.mean(np.asarray(result['position_error'])[tail] ** 2)):.6f} m"
    )
    print(
        f"tail_attitude_rmse={np.sqrt(np.mean(np.asarray(result['attitude_error'])[tail] ** 2)):.6f} rad "
        f"max_quaternion_norm_error={np.max(np.abs(np.asarray(result['quaternion_norm']) - 1.0)):.3e} "
        f"rotor_saturation_fraction={np.mean(result['saturated']):.6f}"
    )
    print(f"plot={plot_path}")


if __name__ == "__main__":
    main()
