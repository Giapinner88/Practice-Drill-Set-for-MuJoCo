"""Run a hybrid energy-shaping and discrete-LQR cart-pole controller."""

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
GAIN_PATH = ARTIFACT_DIR / "lqr_gain.npy"

POLE_MASS_KG = 1.0
POLE_LENGTH_M = 0.5
GRAVITY_M_S2 = 9.81
FORCE_LIMIT_N = 10.0


def wrap_to_pi(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def run(duration: float, headless: bool) -> dict[str, np.ndarray | float]:
    if not GAIN_PATH.exists():
        raise FileNotFoundError(
            f"missing {GAIN_PATH}; run mujoco_linearization.py before simulation"
        )
    gain = np.load(GAIN_PATH)
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    slider_id = model.joint("slider").id
    hinge_id = model.joint("hinge").id
    x_adr = model.jnt_qposadr[slider_id]
    theta_adr = model.jnt_qposadr[hinge_id]
    xdot_adr = model.jnt_dofadr[slider_id]
    thetadot_adr = model.jnt_dofadr[hinge_id]
    actuator_id = model.actuator("force_x").id

    data.qpos[x_adr] = 0.0
    data.qpos[theta_adr] = 0.1
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    target_energy = 2.0 * POLE_MASS_KG * GRAVITY_M_S2 * POLE_LENGTH_M
    balance_mode = False
    saturated_steps = 0
    log: dict[str, list[float]] = {
        "time": [],
        "x": [],
        "theta_error": [],
        "theta_dot": [],
        "force": [],
        "mode": [],
    }

    def control_and_step() -> None:
        nonlocal balance_mode, saturated_steps
        x = float(data.qpos[x_adr])
        theta = float(data.qpos[theta_adr])
        x_dot = float(data.qvel[xdot_adr])
        theta_dot = float(data.qvel[thetadot_adr])
        upright_error = wrap_to_pi(theta - np.pi)

        if balance_mode and (abs(upright_error) > 0.5 or abs(theta_dot) > 3.0):
            balance_mode = False
        elif not balance_mode and abs(upright_error) < 0.25 and abs(theta_dot) < 1.5:
            balance_mode = True

        if balance_mode:
            state_error = np.array([x, upright_error, x_dot, theta_dot])
            raw_force = float(-(gain @ state_error)[0])
        else:
            wrapped_theta = wrap_to_pi(theta)
            energy = (
                0.5 * POLE_MASS_KG * (POLE_LENGTH_M * theta_dot) ** 2
                + POLE_MASS_KG
                * GRAVITY_M_S2
                * POLE_LENGTH_M
                * (1.0 - np.cos(wrapped_theta))
            )
            energy_error = energy - target_energy
            raw_force = -20.0 * energy_error * theta_dot * np.cos(wrapped_theta)
            raw_force += -3.0 * x - 1.5 * x_dot

        force = float(np.clip(raw_force, -FORCE_LIMIT_N, FORCE_LIMIT_N))
        saturated_steps += int(not np.isclose(raw_force, force))
        data.ctrl[actuator_id] = force

        log["time"].append(float(data.time))
        log["x"].append(x)
        log["theta_error"].append(upright_error)
        log["theta_dot"].append(theta_dot)
        log["force"].append(force)
        log["mode"].append(float(balance_mode))
        mujoco.mj_step(model, data)

    if headless:
        while data.time < duration:
            control_and_step()
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time < duration:
                step_start = time.perf_counter()
                control_and_step()
                viewer.sync()
                delay = model.opt.timestep - (time.perf_counter() - step_start)
                if delay > 0:
                    time.sleep(delay)

    result: dict[str, np.ndarray | float] = {key: np.asarray(value) for key, value in log.items()}
    result["saturation_fraction"] = saturated_steps / max(1, len(log["time"]))
    return result


def save_plot(result: dict[str, np.ndarray | float]) -> Path:
    ARTIFACT_DIR.mkdir(exist_ok=True)
    output_path = ARTIFACT_DIR / "cartpole_diagnostics.png"
    time_log = np.asarray(result["time"])
    theta_error = np.asarray(result["theta_error"])
    theta_dot = np.asarray(result["theta_dot"])
    position = np.asarray(result["x"])
    force = np.asarray(result["force"])

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axes[0, 0].plot(theta_error, theta_dot)
    axes[0, 0].set(xlabel="upright angle error [rad]", ylabel="theta_dot [rad/s]", title="Phase portrait")
    axes[0, 1].plot(time_log, theta_error, label="angle error")
    axes[0, 1].plot(time_log, np.asarray(result["mode"]), label="LQR mode")
    axes[0, 1].set(xlabel="time [s]", title="Hybrid mode")
    axes[0, 1].legend()
    axes[1, 0].plot(time_log, position)
    axes[1, 0].set(xlabel="time [s]", ylabel="x [m]", title="Cart position")
    axes[1, 1].plot(time_log, force)
    axes[1, 1].axhline(FORCE_LIMIT_N, color="tab:red", linestyle="--")
    axes[1, 1].axhline(-FORCE_LIMIT_N, color="tab:red", linestyle="--")
    axes[1, 1].set(xlabel="time [s]", ylabel="force [N]", title="Control")
    for axis in axes.flat:
        axis.grid(True)
    figure.savefig(output_path, dpi=150)
    plt.close(figure)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    result = run(args.duration, args.headless)
    plot_path = save_plot(result)
    balance_fraction = float(np.mean(np.asarray(result["mode"])))
    print(
        f"duration={args.duration:.3f} s balance_mode_fraction={balance_fraction:.6f} "
        f"saturation_fraction={result['saturation_fraction']:.6f}"
    )
    print(f"plot={plot_path}")


if __name__ == "__main__":
    main()
