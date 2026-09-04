"""Repository-level contracts for the current lesson set."""

from __future__ import annotations

import ast
from pathlib import Path

import mujoco
import numpy as np
import pytest

from tools.validate_repo import (
    validate_acrobot,
    validate_lesson_invariants,
    validate_mobile_robot,
    validate_quadrotor,
    validate_so101,
)

ROOT = Path(__file__).resolve().parents[1]
PART_DIRS = (ROOT / "part_1_foundations", ROOT / "part_2_models_control")
LESSON_DIRS = sorted(
    {
        model_path.parent
        for part_dir in PART_DIRS
        for model_path in part_dir.rglob("model.xml")
    }
)
MODEL_PATHS = sorted(path for part_dir in PART_DIRS for path in part_dir.rglob("*.xml"))


@pytest.mark.parametrize("lesson_dir", LESSON_DIRS, ids=lambda path: path.name)
def test_lesson_contract(lesson_dir: Path) -> None:
    assert (lesson_dir / "README.md").is_file()
    assert (lesson_dir / "model.xml").is_file()
    assert list(lesson_dir.glob("*.py"))


@pytest.mark.parametrize("script", sorted(ROOT.glob("part_*/**/*.py")), ids=lambda path: path.name)
def test_python_syntax(script: Path) -> None:
    ast.parse(script.read_text(encoding="utf-8"), filename=str(script))


@pytest.mark.parametrize("model_path", MODEL_PATHS, ids=lambda path: f"{path.parent.name}/{path.name}")
def test_model_compiles_and_steps(model_path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(10):
        mujoco.mj_step(model, data)
    for array in (data.qpos, data.qvel, data.qacc, data.ctrl):
        assert np.all(np.isfinite(array))


def test_cartpole_linearization_matches_analytical_convention() -> None:
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
    expected_a_lower_left = np.array([[0.0, -9.81], [0.0, 39.24]])
    expected_b_lower = np.array([[1.0], [-2.0]])
    assert np.allclose(a_discrete[model.nv :, : model.nv] / dt, expected_a_lower_left, rtol=2e-4, atol=2e-4)
    assert np.allclose(b_discrete[model.nv :, :] / dt, expected_b_lower, rtol=2e-4, atol=2e-4)


def test_lesson_specific_invariants() -> None:
    assert validate_lesson_invariants() == []


def test_acrobot_contracts() -> None:
    assert validate_acrobot() == []


def test_so101_contracts() -> None:
    assert validate_so101() == []


def test_mobile_robot_contracts() -> None:
    assert validate_mobile_robot() == []


def test_quadrotor_contracts() -> None:
    assert validate_quadrotor() == []
