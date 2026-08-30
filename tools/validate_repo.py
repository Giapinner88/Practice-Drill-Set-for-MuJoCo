"""Validate lesson structure, Python syntax and MuJoCo model smoke steps."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PART_DIRS = (ROOT / "part_1_foundations", ROOT / "part_2_models_control")


def lesson_directories() -> list[Path]:
    return sorted(
        directory
        for part_dir in PART_DIRS
        for directory in part_dir.iterdir()
        if directory.is_dir() and (directory / "model.xml").exists()
    )


def validate_structure() -> list[str]:
    errors: list[str] = []
    for lesson_dir in lesson_directories():
        for required_name in ("README.md", "model.xml"):
            if not (lesson_dir / required_name).is_file():
                errors.append(f"missing {lesson_dir.relative_to(ROOT) / required_name}")
        scripts = sorted(lesson_dir.glob("*.py"))
        if not scripts:
            errors.append(f"no Python script in {lesson_dir.relative_to(ROOT)}")
        for script in scripts:
            try:
                ast.parse(script.read_text(encoding="utf-8"), filename=str(script))
            except SyntaxError as exc:
                errors.append(f"Python syntax error in {script.relative_to(ROOT)}: {exc}")
    return errors


def model_paths() -> list[Path]:
    return sorted(path for part_dir in PART_DIRS for path in part_dir.glob("*/*.xml"))


def validate_models(step_count: int = 10) -> list[str]:
    errors: list[str] = []
    for model_path in model_paths():
        relative_path = model_path.relative_to(ROOT)
        try:
            model = mujoco.MjModel.from_xml_path(str(model_path))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            for _ in range(step_count):
                mujoco.mj_step(model, data)
            arrays = (data.qpos, data.qvel, data.qacc, data.ctrl)
            if not all(np.all(np.isfinite(array)) for array in arrays):
                errors.append(f"non-finite state after smoke steps: {relative_path}")
            else:
                print(
                    f"PASS {relative_path}: nq={model.nq} nv={model.nv} "
                    f"nu={model.nu} steps={step_count}"
                )
        except Exception as exc:  # report all models in one validation run
            errors.append(f"failed to compile/step {relative_path}: {type(exc).__name__}: {exc}")
    return errors


def validate_cartpole_linearization() -> list[str]:
    errors: list[str] = []
    model_path = ROOT / "part_2_models_control/02_cartpole/model.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[model.jnt_qposadr[model.joint("slider").id]] = 0.0
    data.qpos[model.jnt_qposadr[model.joint("hinge").id]] = np.pi
    mujoco.mj_forward(model, data)

    state_size = 2 * model.nv
    a_discrete = np.zeros((state_size, state_size))
    b_discrete = np.zeros((state_size, model.nu))
    mujoco.mjd_transitionFD(
        model, data, 1e-6, True, a_discrete, b_discrete, None, None
    )
    dt = float(model.opt.timestep)
    expected_acceleration_from_position = np.array([[0.0, -9.81], [0.0, 39.24]])
    expected_acceleration_from_control = np.array([[1.0], [-2.0]])
    measured_position_block = a_discrete[model.nv :, : model.nv] / dt
    measured_control_block = b_discrete[model.nv :, :] / dt
    if not np.allclose(
        measured_position_block,
        expected_acceleration_from_position,
        rtol=2e-4,
        atol=2e-4,
    ):
        errors.append(
            "Cart-pole analytical A_c convention disagrees with MuJoCo transition Jacobian"
        )
    if not np.allclose(
        measured_control_block,
        expected_acceleration_from_control,
        rtol=2e-4,
        atol=2e-4,
    ):
        errors.append(
            "Cart-pole analytical B_c convention disagrees with MuJoCo transition Jacobian"
        )
    if not errors:
        print("PASS cart-pole analytical/discrete sign and state-order convention")
    return errors


def main() -> int:
    errors = validate_structure() + validate_models() + validate_cartpole_linearization()
    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        print(f"validation failed with {len(errors)} error(s)", file=sys.stderr)
        return 1
    print(f"validation passed for {len(lesson_directories())} lessons and {len(model_paths())} models")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
