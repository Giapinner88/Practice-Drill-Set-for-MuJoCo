"""Jacobian-based controller for the SO-101 reach-and-press task."""

from __future__ import annotations

import mujoco
import numpy as np

ARM_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

TASK_GAIN = 4.0
DAMPING = 0.03
MAX_TASK_SPEED_M_S = 0.08
MAX_JOINT_SPEED_RAD_S = 1.2
APPROACH_RADIUS_M = 0.006
APPROACH_DWELL_S = 0.15
PRESS_DEPTH_M = 0.006
PRESS_FORCE_N = 0.25
PRESS_DWELL_S = 0.10


class ReachPressController:
    """Reach an approach site, then press until depth and force agree."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        joint_ids = np.array([model.joint(name).id for name in ARM_JOINTS])
        self.qpos_adrs = model.jnt_qposadr[joint_ids]
        self.dof_adrs = model.jnt_dofadr[joint_ids]
        self.actuator_ids = np.array([model.actuator(name).id for name in ARM_JOINTS])
        self.joint_ids = joint_ids
        self.ee_site = model.site("gripperframe").id
        self.target_sites = (
            model.site("approach_target").id,
            model.site("press_target").id,
        )
        self.button_qpos_adr = model.jnt_qposadr[model.joint("button_slide").id]
        self.force_sensor_adr = model.sensor_adr[model.sensor("button_force").id]
        self.q_reference = data.ctrl.copy()
        self.jacobian_position = np.zeros((3, model.nv))
        self.jacobian_rotation = np.zeros((3, model.nv))
        self.phase = 0
        self.dwell_time = 0.0
        self.success_time = np.nan

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        mujoco.mj_forward(model, data)
        target = data.site_xpos[self.target_sites[self.phase]]
        error = target - data.site_xpos[self.ee_site]
        distance = float(np.linalg.norm(error))

        if self.phase == 0:
            self.dwell_time = (
                self.dwell_time + model.opt.timestep
                if distance < APPROACH_RADIUS_M
                else 0.0
            )
            if self.dwell_time >= APPROACH_DWELL_S:
                self.phase = 1
                self.dwell_time = 0.0

        button_depth = float(data.qpos[self.button_qpos_adr])
        button_force = float(data.sensordata[self.force_sensor_adr])
        if self.phase == 1 and button_depth > PRESS_DEPTH_M and button_force > PRESS_FORCE_N:
            self.dwell_time += model.opt.timestep
            if self.dwell_time >= PRESS_DWELL_S and np.isnan(self.success_time):
                self.success_time = float(data.time)
        elif self.phase == 1:
            self.dwell_time = 0.0

        mujoco.mj_jacSite(
            model,
            data,
            self.jacobian_position,
            self.jacobian_rotation,
            self.ee_site,
        )
        jacobian = self.jacobian_position[:, self.dof_adrs]
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        task_velocity = TASK_GAIN * error
        task_velocity *= min(
            1.0,
            MAX_TASK_SPEED_M_S / max(float(np.linalg.norm(task_velocity)), 1e-12),
        )
        joint_velocity = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + DAMPING**2 * np.eye(3),
            task_velocity,
        )
        joint_velocity *= min(
            1.0,
            MAX_JOINT_SPEED_RAD_S
            / max(float(np.max(np.abs(joint_velocity))), 1e-12),
        )

        self.q_reference[self.actuator_ids] += joint_velocity * model.opt.timestep
        self.q_reference[self.actuator_ids] = np.clip(
            self.q_reference[self.actuator_ids],
            model.actuator_ctrlrange[self.actuator_ids, 0],
            model.actuator_ctrlrange[self.actuator_ids, 1],
        )
        data.ctrl[:] = self.q_reference

        actuator_force = np.abs(data.actuator_force[self.actuator_ids])
        force_limit = model.actuator_forcerange[self.actuator_ids, 1]
        joint_margin = np.minimum(
            data.qpos[self.qpos_adrs] - model.jnt_range[self.joint_ids, 0],
            model.jnt_range[self.joint_ids, 1] - data.qpos[self.qpos_adrs],
        )
        return {
            "time": float(data.time),
            "distance": distance,
            "button_depth": button_depth,
            "button_force": button_force,
            "minimum_singular_value": float(singular_values[-1]),
            "condition_number": float(singular_values[0] / singular_values[-1]),
            "minimum_joint_margin": float(np.min(joint_margin)),
            "maximum_actuator_force": float(np.max(actuator_force)),
            "saturated": float(np.any(actuator_force >= force_limit - 1e-4)),
            "phase": float(self.phase),
            "success": float(not np.isnan(self.success_time)),
        }
