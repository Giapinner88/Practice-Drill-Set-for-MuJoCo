"""PFL energy-shaping and LQR control for the Acrobot lesson."""

from __future__ import annotations

import mujoco
import numpy as np

MASS_1_KG = 1.0
MASS_2_KG = 1.0
LENGTH_1_M = 0.7
LENGTH_2_M = 0.7
GRAVITY_M_S2 = 9.81
TORQUE_LIMIT_NM = 8.0

ELBOW_POSITION_GAIN = 8.0
ELBOW_VELOCITY_GAIN = 4.0
ENERGY_GAIN = 1.0


def wrap_to_pi(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def mechanical_energy(
    q1: float,
    q2: float,
    velocity: np.ndarray,
    mass_matrix: np.ndarray,
) -> float:
    kinetic = 0.5 * float(velocity @ mass_matrix @ velocity)
    potential = (
        (MASS_1_KG + MASS_2_KG)
        * GRAVITY_M_S2
        * LENGTH_1_M
        * (1.0 - np.cos(q1))
        + MASS_2_KG
        * GRAVITY_M_S2
        * LENGTH_2_M
        * (1.0 - np.cos(q1 + q2))
    )
    return kinetic + potential


class AcrobotController:
    """Hybrid controller that owns mode switching and torque saturation."""

    def __init__(self, model: mujoco.MjModel, lqr_gain: np.ndarray) -> None:
        shoulder = model.joint("shoulder_hinge").id
        elbow = model.joint("elbow_hinge").id
        self.q1_adr = model.jnt_qposadr[shoulder]
        self.q2_adr = model.jnt_qposadr[elbow]
        self.v1_adr = model.jnt_dofadr[shoulder]
        self.v2_adr = model.jnt_dofadr[elbow]
        self.actuator_id = model.actuator("elbow_torque").id
        self.gain = lqr_gain
        self.mass_matrix = np.empty((model.nv, model.nv))
        self.target_energy = 2.0 * GRAVITY_M_S2 * (
            (MASS_1_KG + MASS_2_KG) * LENGTH_1_M + MASS_2_KG * LENGTH_2_M
        )
        self.balance_mode = False
        self.first_balance_time = np.nan
        self.saturated_steps = 0

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        mujoco.mj_forward(model, data)
        mujoco.mj_fullM(model, self.mass_matrix, data.qM)

        q1 = float(data.qpos[self.q1_adr])
        q2 = float(data.qpos[self.q2_adr])
        v1 = float(data.qvel[self.v1_adr])
        v2 = float(data.qvel[self.v2_adr])
        e1 = wrap_to_pi(q1 - np.pi)
        e2 = wrap_to_pi(q2)
        speed = max(abs(v1), abs(v2))

        if self.balance_mode and (
            max(abs(e1), abs(e2)) > 0.50 or speed > 5.0
        ):
            self.balance_mode = False
        elif not self.balance_mode and (
            abs(e1) < 0.25 and abs(e2) < 0.30 and speed < 2.0
        ):
            self.balance_mode = True
            if np.isnan(self.first_balance_time):
                self.first_balance_time = float(data.time)

        energy = mechanical_energy(q1, q2, data.qvel, self.mass_matrix)
        if self.balance_mode:
            state_error = np.array([e1, e2, v1, v2])
            raw_torque = float(-(self.gain @ state_error)[0])
        else:
            raw_torque = self._pfl_torque(data, e2, v1, v2, energy)

        torque = float(np.clip(raw_torque, -TORQUE_LIMIT_NM, TORQUE_LIMIT_NM))
        self.saturated_steps += int(not np.isclose(raw_torque, torque))
        data.ctrl[self.actuator_id] = torque
        return {
            "time": float(data.time),
            "shoulder_error": e1,
            "elbow_error": e2,
            "shoulder_velocity": v1,
            "elbow_velocity": v2,
            "energy": energy,
            "torque": torque,
            "mode": float(self.balance_mode),
        }

    def _pfl_torque(
        self,
        data: mujoco.MjData,
        elbow_error: float,
        shoulder_velocity: float,
        elbow_velocity: float,
        energy: float,
    ) -> float:
        desired_elbow_acceleration = (
            -ELBOW_POSITION_GAIN * elbow_error
            - ELBOW_VELOCITY_GAIN * elbow_velocity
            + ENERGY_GAIN * shoulder_velocity * (energy - self.target_energy)
        )
        effective_bias = data.qfrc_bias - data.qfrc_passive
        shoulder_acceleration = -(
            effective_bias[0]
            + self.mass_matrix[0, 1] * desired_elbow_acceleration
        ) / self.mass_matrix[0, 0]
        return float(
            self.mass_matrix[1, 0] * shoulder_acceleration
            + self.mass_matrix[1, 1] * desired_elbow_acceleration
            + effective_bias[1]
        )
