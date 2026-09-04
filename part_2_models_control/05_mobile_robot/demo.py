"""Run differential-drive waypoint tracking and save measured diagnostics."""

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

from controller import TRACK_WIDTH_M, WAYPOINTS_M, WHEEL_RADIUS_M, WaypointController

LESSON_DIR = Path(__file__).resolve().parent
MODEL_PATH = LESSON_DIR / "model.xml"
ARTIFACT_DIR = LESSON_DIR / "artifacts"


def simulate(duration: float | None, headless: bool) -> dict[str, np.ndarray | float]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    controller = WaypointController(model)
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
    return result


def estimate_wheel_odometry(result: dict[str, np.ndarray | float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    timestep = float(result["timestep"])
    left_speed = np.asarray(result["left_speed"])
    right_speed = np.asarray(result["right_speed"])
    x = np.zeros_like(left_speed)
    y = np.zeros_like(left_speed)
    yaw = np.zeros_like(left_speed)
    for index in range(1, len(x)):
        linear_speed = 0.5 * WHEEL_RADIUS_M * (left_speed[index - 1] + right_speed[index - 1])
        yaw_rate = WHEEL_RADIUS_M * (right_speed[index - 1] - left_speed[index - 1]) / TRACK_WIDTH_M
        x[index] = x[index - 1] + linear_speed * np.cos(yaw[index - 1]) * timestep
        y[index] = y[index - 1] + linear_speed * np.sin(yaw[index - 1]) * timestep
        yaw[index] = yaw[index - 1] + yaw_rate * timestep
    return x, y, yaw


def measured_forward_speed(result: dict[str, np.ndarray | float]) -> np.ndarray:
    timestep = float(result["timestep"])
    x = np.asarray(result["x"])
    y = np.asarray(result["y"])
    yaw = np.asarray(result["yaw"])
    vx = np.gradient(x, timestep)
    vy = np.gradient(y, timestep)
    return vx * np.cos(yaw) + vy * np.sin(yaw)


def save_plot(result: dict[str, np.ndarray | float]) -> Path:
    ARTIFACT_DIR.mkdir(exist_ok=True)
    output_path = ARTIFACT_DIR / "mobile_robot_diagnostics.png"
    t = np.asarray(result["time"])
    odom_x, odom_y, _ = estimate_wheel_odometry(result)
    forward_speed = measured_forward_speed(result)
    slip_speed = np.asarray(result["wheel_linear_speed"]) - forward_speed

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axes[0, 0].plot(result["x"], result["y"], label="MuJoCo pose")
    axes[0, 0].plot(odom_x, odom_y, linestyle="--", label="wheel odometry")
    axes[0, 0].scatter(WAYPOINTS_M[:, 0], WAYPOINTS_M[:, 1], marker="x", color="tab:red", label="waypoints")
    axes[0, 0].set(xlabel="world x [m]", ylabel="world y [m]", title="Planar trajectory")
    axes[0, 0].axis("equal")
    axes[0, 0].legend()

    axes[0, 1].plot(t, result["distance"], label="active waypoint distance")
    axes[0, 1].plot(t, result["waypoint_index"], label="waypoint index")
    axes[0, 1].set(xlabel="time [s]", title="Waypoint progress")
    axes[0, 1].legend()

    axes[1, 0].plot(t, result["left_target"], label="left target")
    axes[1, 0].plot(t, result["left_speed"], linestyle="--", label="left measured")
    axes[1, 0].plot(t, result["right_target"], label="right target")
    axes[1, 0].plot(t, result["right_speed"], linestyle="--", label="right measured")
    axes[1, 0].set(xlabel="time [s]", ylabel="wheel speed [rad/s]", title="Velocity actuators")
    axes[1, 0].legend()

    axes[1, 1].plot(t, result["wheel_linear_speed"], label="from wheel speed")
    axes[1, 1].plot(t, forward_speed, label="from body pose")
    axes[1, 1].plot(t, slip_speed, label="difference")
    axes[1, 1].set(xlabel="time [s]", ylabel="forward speed [m/s]", title="No-slip comparison")
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
    duration = args.duration if args.duration is not None else (15.0 if args.headless else None)

    result = simulate(duration, args.headless)
    plot_path = save_plot(result)
    odom_x, odom_y, _ = estimate_wheel_odometry(result)
    actual_position = np.array([result["x"][-1], result["y"][-1]])
    odom_position = np.array([odom_x[-1], odom_y[-1]])
    forward_speed = measured_forward_speed(result)
    slip_speed = np.asarray(result["wheel_linear_speed"]) - forward_speed
    success_time = float(result["success_time"])

    print(
        f"simulated_time={np.asarray(result['time'])[-1]:.3f} s "
        f"timestep={float(result['timestep']):.4f} s "
        f"success={np.isfinite(success_time)} success_time={success_time:.3f} s"
    )
    print(
        f"final_position_world_m={actual_position} "
        f"final_goal_error={np.linalg.norm(actual_position - WAYPOINTS_M[-1]):.6f} m "
        f"odometry_position_error={np.linalg.norm(odom_position - actual_position):.6f} m"
    )
    print(
        f"slip_speed_rmse={np.sqrt(np.mean(slip_speed**2)):.6f} m/s "
        f"control_saturation_fraction={np.mean(result['control_saturated']):.6f} "
        f"force_saturation_fraction={np.mean(result['force_saturated']):.6f}"
    )
    print(f"plot={plot_path}")


if __name__ == "__main__":
    main()
