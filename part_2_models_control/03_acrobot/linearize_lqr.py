"""Linearize the Acrobot at upright and save a discrete-time LQR gain."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
import scipy.linalg as la

LESSON_DIR = Path(__file__).resolve().parent
MODEL_PATH = LESSON_DIR / "model.xml"
ARTIFACT_DIR = LESSON_DIR / "artifacts"
GAIN_PATH = ARTIFACT_DIR / "lqr_gain.npy"
METADATA_PATH = ARTIFACT_DIR / "lqr_metadata.json"

STATE_ORDER = ["shoulder_error", "elbow_error", "shoulder_velocity", "elbow_velocity"]
Q_COST = np.diag([60.0, 25.0, 8.0, 4.0])
R_COST = np.diag([1.0])


def linearize_and_design() -> dict[str, np.ndarray | float | int]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    shoulder_id = model.joint("shoulder_hinge").id
    elbow_id = model.joint("elbow_hinge").id

    data.qpos[model.jnt_qposadr[shoulder_id]] = np.pi
    data.qpos[model.jnt_qposadr[elbow_id]] = 0.0
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
        [
            np.linalg.matrix_power(a_discrete, power) @ b_discrete
            for power in range(state_size)
        ]
    )
    controllability_rank = int(np.linalg.matrix_rank(controllability))
    if controllability_rank != state_size:
        raise RuntimeError(
            "upright linearization is not controllable: "
            f"{controllability_rank}/{state_size}"
        )

    riccati = la.solve_discrete_are(a_discrete, b_discrete, Q_COST, R_COST)
    gain = np.linalg.solve(
        b_discrete.T @ riccati @ b_discrete + R_COST,
        b_discrete.T @ riccati @ a_discrete,
    )
    residual = (
        a_discrete.T @ riccati @ a_discrete
        - riccati
        - a_discrete.T
        @ riccati
        @ b_discrete
        @ np.linalg.solve(
            b_discrete.T @ riccati @ b_discrete + R_COST,
            b_discrete.T @ riccati @ a_discrete,
        )
        + Q_COST
    )

    return {
        "a_discrete": a_discrete,
        "b_discrete": b_discrete,
        "gain": gain,
        "controllability_rank": controllability_rank,
        "riccati_residual_inf_norm": float(np.linalg.norm(residual, ord=np.inf)),
        "timestep_s": float(model.opt.timestep),
    }


def main() -> None:
    result = linearize_and_design()
    ARTIFACT_DIR.mkdir(exist_ok=True)
    np.save(GAIN_PATH, np.asarray(result["gain"]))
    METADATA_PATH.write_text(
        json.dumps(
            {
                "model": MODEL_PATH.name,
                "timestep_s": result["timestep_s"],
                "equilibrium": [float(np.pi), 0.0, 0.0, 0.0],
                "state_order": STATE_ORDER,
                "controllability_rank": result["controllability_rank"],
                "riccati_residual_inf_norm": result["riccati_residual_inf_norm"],
                "q_cost_diagonal": np.diag(Q_COST).tolist(),
                "r_cost_diagonal": np.diag(R_COST).tolist(),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    print("A_d =\n", np.array2string(np.asarray(result["a_discrete"]), precision=6))
    print("B_d =\n", np.array2string(np.asarray(result["b_discrete"]), precision=6))
    print("K =\n", np.array2string(np.asarray(result["gain"]), precision=6))
    print(f"controllability_rank={result['controllability_rank']}/4")
    print(
        "riccati_residual_inf_norm="
        f"{float(result['riccati_residual_inf_norm']):.3e}"
    )
    print(f"gain={GAIN_PATH}")
    print(f"metadata={METADATA_PATH}")


if __name__ == "__main__":
    main()
