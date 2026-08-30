"""Compute discrete MuJoCo transition Jacobians and a discrete LQR gain."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
import scipy.linalg as la

LESSON_DIR = Path(__file__).resolve().parent
MODEL_PATH = LESSON_DIR / "model.xml"
ARTIFACT_DIR = LESSON_DIR / "artifacts"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    slider_id = model.joint("slider").id
    hinge_id = model.joint("hinge").id
    data.qpos[model.jnt_qposadr[slider_id]] = 0.0
    data.qpos[model.jnt_qposadr[hinge_id]] = np.pi
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    state_size = 2 * model.nv
    a_discrete = np.zeros((state_size, state_size))
    b_discrete = np.zeros((state_size, model.nu))
    mujoco.mjd_transitionFD(
        model,
        data,
        1e-6,
        True,
        a_discrete,
        b_discrete,
        None,
        None,
    )

    controllability = np.hstack(
        [np.linalg.matrix_power(a_discrete, power) @ b_discrete for power in range(state_size)]
    )
    controllability_rank = int(np.linalg.matrix_rank(controllability))
    if controllability_rank != state_size:
        raise RuntimeError(
            f"linearized system is not controllable: {controllability_rank}/{state_size}"
        )

    q_cost = np.diag([2.0, 30.0, 1.0, 4.0])
    r_cost = np.diag([0.5])
    riccati = la.solve_discrete_are(a_discrete, b_discrete, q_cost, r_cost)
    gain = np.linalg.solve(
        b_discrete.T @ riccati @ b_discrete + r_cost,
        b_discrete.T @ riccati @ a_discrete,
    )
    residual = (
        a_discrete.T @ riccati @ a_discrete
        - riccati
        - a_discrete.T
        @ riccati
        @ b_discrete
        @ np.linalg.solve(b_discrete.T @ riccati @ b_discrete + r_cost, b_discrete.T @ riccati @ a_discrete)
        + q_cost
    )

    ARTIFACT_DIR.mkdir(exist_ok=True)
    gain_path = ARTIFACT_DIR / "lqr_gain.npy"
    metadata_path = ARTIFACT_DIR / "lqr_metadata.json"
    np.save(gain_path, gain)
    metadata_path.write_text(
        json.dumps(
            {
                "model": MODEL_PATH.name,
                "timestep_s": float(model.opt.timestep),
                "equilibrium": [0.0, float(np.pi), 0.0, 0.0],
                "state_order": ["x", "theta", "x_dot", "theta_dot"],
                "controllability_rank": controllability_rank,
                "riccati_residual_inf_norm": float(np.linalg.norm(residual, ord=np.inf)),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    print("A_d =\n", np.array2string(a_discrete, precision=6))
    print("B_d =\n", np.array2string(b_discrete, precision=6))
    print("K =\n", np.array2string(gain, precision=6))
    print(f"controllability_rank={controllability_rank}/{state_size}")
    print(f"riccati_residual_inf_norm={np.linalg.norm(residual, ord=np.inf):.3e}")
    print(f"gain={gain_path}")
    print(f"metadata={metadata_path}")


if __name__ == "__main__":
    main()
