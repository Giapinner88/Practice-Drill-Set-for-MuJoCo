"""Run energy shaping on the point-mass pendulum and save diagnostics."""

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

LESSON_DIR = Path(__file__).resolve().parent
MODEL_PATH = LESSON_DIR / "model.xml"
ARTIFACT_DIR = LESSON_DIR / "artifacts"

MASS_KG = 1.0
LENGTH_M = 1.0
COM_INERTIA_KG_M2 = 1e-6
GRAVITY_M_S2 = 9.81
DAMPING_NM_S_RAD = 0.01


def simulate(
    duration: float | None,
    headless: bool,
    gain: float,
    torque_limit: float,
    initial_angle: float,
) -> dict[str, np.ndarray | float]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    joint_id = model.joint("hinge_y").id
    qpos_adr = model.jnt_qposadr[joint_id]
    dof_adr = model.jnt_dofadr[joint_id]
    actuator_id = model.actuator("torque_motor").id

    data.qpos[qpos_adr] = initial_angle
    data.qvel[dof_adr] = 0.0
    mujoco.mj_forward(model, data)

    generalized_inertia = MASS_KG * LENGTH_M**2 + COM_INERTIA_KG_M2
    desired_energy = MASS_KG * GRAVITY_M_S2 * LENGTH_M

    log_time: list[float] = []
    log_angle: list[float] = []
    log_velocity: list[float] = []
    log_energy: list[float] = []
    log_torque: list[float] = []
    saturated_steps = 0

    def control_and_step() -> None:
        nonlocal saturated_steps
        angle = float(data.qpos[qpos_adr])
        velocity = float(data.qvel[dof_adr])
        energy = (
            0.5 * generalized_inertia * velocity**2
            - MASS_KG * GRAVITY_M_S2 * LENGTH_M * np.cos(angle)
        )
        raw_torque = -gain * velocity * (energy - desired_energy)
        raw_torque += DAMPING_NM_S_RAD * velocity
        torque = float(np.clip(raw_torque, -torque_limit, torque_limit))
        saturated_steps += int(not np.isclose(raw_torque, torque))
        data.ctrl[actuator_id] = torque

        log_time.append(float(data.time))
        log_angle.append(angle)
        log_velocity.append(velocity)
        log_energy.append(float(energy))
        log_torque.append(torque)
        mujoco.mj_step(model, data)

    if headless:
        assert duration is not None
        while data.time < duration:
            control_and_step()
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and (duration is None or data.time < duration):
                step_start = time.perf_counter()
                control_and_step()
                viewer.sync()
                delay = model.opt.timestep - (time.perf_counter() - step_start)
                if delay > 0:
                    time.sleep(delay)

    sample_count = max(1, len(log_time))
    return {
        "time": np.asarray(log_time),
        "angle": np.asarray(log_angle),
        "velocity": np.asarray(log_velocity),
        "energy": np.asarray(log_energy),
        "torque": np.asarray(log_torque),
        "desired_energy": desired_energy,
        "saturation_fraction": saturated_steps / sample_count,
    }


def save_plot(result: dict[str, np.ndarray | float], gain: float) -> Path:
    ARTIFACT_DIR.mkdir(exist_ok=True)
    output_path = ARTIFACT_DIR / "swingup_analysis.png"
    time_log = np.asarray(result["time"])
    angle = np.asarray(result["angle"])
    velocity = np.asarray(result["velocity"])
    energy = np.asarray(result["energy"])

    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    axes[0].plot(angle, velocity, linewidth=1.0, label="trajectory")
    axes[0].scatter([angle[0]], [velocity[0]], label="start", zorder=3)
    axes[0].scatter([angle[-1]], [velocity[-1]], label="end", zorder=3)
    axes[0].scatter([-np.pi, np.pi], [0, 0], facecolors="none", edgecolors="tab:orange", label="upright")
    axes[0].set(xlabel="theta [rad]", ylabel="theta_dot [rad/s]", title=f"Phase portrait (k={gain:g})")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(time_log, energy, label="E(t)")
    axes[1].axhline(float(result["desired_energy"]), color="tab:red", linestyle="--", label="E_up")
    axes[1].set(xlabel="time [s]", ylabel="energy [J]", title="Mechanical energy")
    axes[1].grid(True)
    axes[1].legend()

    figure.savefig(output_path, dpi=150)
    plt.close(figure)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, help="simulation seconds; viewer is unlimited by default")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--gain", type=float, default=0.5)
    parser.add_argument("--torque-limit", type=float, default=3.0)
    parser.add_argument("--initial-angle", type=float, default=0.1)
    args = parser.parse_args()
    if (args.duration is not None and args.duration <= 0) or args.gain < 0 or args.torque_limit <= 0:
        parser.error("duration and torque limit must be positive; gain must be non-negative")

    duration = args.duration if args.duration is not None else (5.0 if args.headless else None)
    result = simulate(
        duration,
        args.headless,
        args.gain,
        args.torque_limit,
        args.initial_angle,
    )
    plot_path = save_plot(result, args.gain)
    final_error = float(np.asarray(result["energy"])[-1] - result["desired_energy"])
    simulated_time = float(np.asarray(result["time"])[-1])
    print(
        f"simulated_time={simulated_time:.3f} s gain={args.gain:g} "
        f"torque_limit={args.torque_limit:g} N*m "
        f"final_energy_error={final_error:.6f} J "
        f"saturation_fraction={result['saturation_fraction']:.6f}"
    )
    print(f"plot={plot_path}")


if __name__ == "__main__":
    main()
