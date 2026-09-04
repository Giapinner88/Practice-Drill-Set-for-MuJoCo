"""Cascaded position-attitude controller for the quadrotor hover task."""

from __future__ import annotations

import mujoco
import numpy as np

MASS_KG = 1.2
GRAVITY_M_S2 = 9.81
ARM_XY_M = 0.18
YAW_MOMENT_PER_THRUST_M = 0.02
MAX_ROTOR_THRUST_N = 6.0

HOVER_TARGET_SITE = "hover_target"
POSITION_GAIN = np.array([1.5, 1.5, 4.0])
VELOCITY_GAIN = np.array([2.5, 2.5, 3.0])
ATTITUDE_GAIN_NM = np.array([0.50, 0.50, 0.25])
ANGULAR_RATE_GAIN_NMS = np.array([0.20, 0.20, 0.14])

POSITION_TOLERANCE_M = 0.05
VELOCITY_TOLERANCE_M_S = 0.05
ATTITUDE_TOLERANCE_RAD = np.deg2rad(3.0)
HOVER_DWELL_S = 0.5

ROTOR_NAMES = ("thrust_fl", "thrust_fr", "thrust_rr", "thrust_rl")

MIXER = np.array(
    [
        [1.0, 1.0, 1.0, 1.0],
        [ARM_XY_M, -ARM_XY_M, -ARM_XY_M, ARM_XY_M],
        [-ARM_XY_M, -ARM_XY_M, ARM_XY_M, ARM_XY_M],
        [
            YAW_MOMENT_PER_THRUST_M,
            -YAW_MOMENT_PER_THRUST_M,
            YAW_MOMENT_PER_THRUST_M,
            -YAW_MOMENT_PER_THRUST_M,
        ],
    ]
)
MIXER_INVERSE = np.linalg.inv(MIXER)


def rotation_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, quaternion)
    return rotation.reshape(3, 3)


def desired_rotation(force_world: np.ndarray) -> np.ndarray:
    body_z = force_world / max(float(np.linalg.norm(force_world)), 1e-12)
    heading_x = np.array([1.0, 0.0, 0.0])
    body_y = np.cross(body_z, heading_x)
    body_y /= max(float(np.linalg.norm(body_y)), 1e-12)
    body_x = np.cross(body_y, body_z)
    return np.column_stack((body_x, body_y, body_z))


def attitude_error_vector(current: np.ndarray, desired: np.ndarray) -> np.ndarray:
    error_matrix = 0.5 * (desired.T @ current - current.T @ desired)
    return np.array([error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]])


class HoverController:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.actuator_ids = np.array([model.actuator(name).id for name in ROTOR_NAMES])
        self.target_site_id = model.site(HOVER_TARGET_SITE).id
        self.dwell_time = 0.0
        self.success_time = np.nan

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        position = data.sensor("position").data.copy()
        velocity = data.sensor("linear_velocity").data.copy()
        quaternion = data.sensor("orientation").data.copy()
        angular_velocity = data.sensor("angular_velocity").data.copy()
        rotation = rotation_from_quaternion(quaternion)

        target_position = data.site_xpos[self.target_site_id]
        position_error = target_position - position
        desired_acceleration = POSITION_GAIN * position_error - VELOCITY_GAIN * velocity
        desired_acceleration[2] += GRAVITY_M_S2
        desired_force = MASS_KG * desired_acceleration
        target_rotation = desired_rotation(desired_force)

        error_vector = attitude_error_vector(rotation, target_rotation)
        attitude_angle = float(
            np.arccos(np.clip(0.5 * (np.trace(target_rotation.T @ rotation) - 1.0), -1.0, 1.0))
        )
        moment = -ATTITUDE_GAIN_NM * error_vector - ANGULAR_RATE_GAIN_NMS * angular_velocity
        total_thrust = float(np.clip(desired_force @ rotation[:, 2], 0.0, 4.0 * MAX_ROTOR_THRUST_N))

        requested_rotor_thrust = MIXER_INVERSE @ np.concatenate(([total_thrust], moment))
        rotor_thrust = np.clip(requested_rotor_thrust, 0.0, MAX_ROTOR_THRUST_N)
        data.ctrl[self.actuator_ids] = rotor_thrust

        position_error_norm = float(np.linalg.norm(position_error))
        velocity_norm = float(np.linalg.norm(velocity))
        if (
            position_error_norm < POSITION_TOLERANCE_M
            and velocity_norm < VELOCITY_TOLERANCE_M_S
            and attitude_angle < ATTITUDE_TOLERANCE_RAD
        ):
            self.dwell_time += model.opt.timestep
            if self.dwell_time >= HOVER_DWELL_S and np.isnan(self.success_time):
                self.success_time = float(data.time)
        else:
            self.dwell_time = 0.0

        return {
            "time": float(data.time),
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
            "position_error": position_error_norm,
            "velocity_norm": velocity_norm,
            "attitude_error": attitude_angle,
            "quaternion_norm": float(np.linalg.norm(quaternion)),
            "total_thrust": total_thrust,
            "roll_moment": float(moment[0]),
            "pitch_moment": float(moment[1]),
            "yaw_moment": float(moment[2]),
            "thrust_fl": float(rotor_thrust[0]),
            "thrust_fr": float(rotor_thrust[1]),
            "thrust_rr": float(rotor_thrust[2]),
            "thrust_rl": float(rotor_thrust[3]),
            "saturated": float(np.any(np.abs(requested_rotor_thrust - rotor_thrust) > 1e-12)),
            "success": float(np.isfinite(self.success_time)),
        }
