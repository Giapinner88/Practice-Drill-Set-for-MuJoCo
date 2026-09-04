"""Waypoint controller for a differential-drive mobile robot."""

from __future__ import annotations

import mujoco
import numpy as np

WHEEL_RADIUS_M = 0.07
TRACK_WIDTH_M = 0.33
MAX_WHEEL_SPEED_RAD_S = 12.0
MAX_LINEAR_SPEED_M_S = 0.40
MAX_YAW_RATE_RAD_S = 1.8
POSITION_GAIN = 1.2
HEADING_GAIN = 3.0
WAYPOINT_RADIUS_M = 0.08
WAYPOINT_DWELL_S = 0.15

WAYPOINTS_M = np.array(
    [
        [0.8, 0.0],
        [0.8, 0.6],
        [0.0, 0.6],
    ]
)


def wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


class WaypointController:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.base_id = model.body("base").id
        self.actuator_ids = np.array(
            [
                model.actuator("left_wheel_velocity").id,
                model.actuator("right_wheel_velocity").id,
            ]
        )
        self.wheel_dof_adrs = np.array(
            [
                model.jnt_dofadr[model.joint("left_wheel_hinge").id],
                model.jnt_dofadr[model.joint("right_wheel_hinge").id],
            ]
        )
        self.waypoint_index = 0
        self.dwell_time = 0.0
        self.success_time = np.nan

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        position = data.xpos[self.base_id, :2].copy()
        rotation = data.xmat[self.base_id].reshape(3, 3)
        yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))

        target = WAYPOINTS_M[self.waypoint_index]
        displacement = target - position
        distance = float(np.linalg.norm(displacement))
        desired_heading = float(np.arctan2(displacement[1], displacement[0]))
        heading_error = wrap_to_pi(desired_heading - yaw)

        if distance < WAYPOINT_RADIUS_M:
            self.dwell_time += model.opt.timestep
            if self.dwell_time >= WAYPOINT_DWELL_S:
                if self.waypoint_index == len(WAYPOINTS_M) - 1:
                    if np.isnan(self.success_time):
                        self.success_time = float(data.time)
                else:
                    self.waypoint_index += 1
                    self.dwell_time = 0.0
        else:
            self.dwell_time = 0.0

        if np.isfinite(self.success_time):
            linear_speed = 0.0
            yaw_rate = 0.0
        else:
            linear_speed = min(POSITION_GAIN * distance, MAX_LINEAR_SPEED_M_S)
            linear_speed *= max(0.0, float(np.cos(heading_error)))
            yaw_rate = float(
                np.clip(HEADING_GAIN * heading_error, -MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S)
            )

        raw_wheel_targets = np.array(
            [
                (linear_speed - 0.5 * TRACK_WIDTH_M * yaw_rate) / WHEEL_RADIUS_M,
                (linear_speed + 0.5 * TRACK_WIDTH_M * yaw_rate) / WHEEL_RADIUS_M,
            ]
        )
        scale = min(
            1.0,
            MAX_WHEEL_SPEED_RAD_S / max(float(np.max(np.abs(raw_wheel_targets))), 1e-12),
        )
        wheel_targets = scale * raw_wheel_targets
        data.ctrl[self.actuator_ids] = wheel_targets

        wheel_speeds = data.qvel[self.wheel_dof_adrs]
        wheel_linear_speed = 0.5 * WHEEL_RADIUS_M * float(np.sum(wheel_speeds))
        wheel_yaw_rate = WHEEL_RADIUS_M * float(wheel_speeds[1] - wheel_speeds[0]) / TRACK_WIDTH_M
        force_limit = model.actuator_forcerange[self.actuator_ids, 1]
        force_saturated = np.any(np.abs(data.actuator_force[self.actuator_ids]) >= force_limit - 1e-6)

        return {
            "time": float(data.time),
            "x": float(position[0]),
            "y": float(position[1]),
            "yaw": yaw,
            "distance": distance,
            "heading_error": heading_error,
            "waypoint_index": float(self.waypoint_index),
            "left_target": float(wheel_targets[0]),
            "right_target": float(wheel_targets[1]),
            "left_speed": float(wheel_speeds[0]),
            "right_speed": float(wheel_speeds[1]),
            "wheel_linear_speed": wheel_linear_speed,
            "wheel_yaw_rate": wheel_yaw_rate,
            "control_saturated": float(scale < 1.0),
            "force_saturated": float(force_saturated),
            "success": float(np.isfinite(self.success_time)),
        }
