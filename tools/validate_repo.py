"""Validate lesson structure, Python syntax and MuJoCo model smoke steps."""

from __future__ import annotations

import ast
import importlib.util
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PART_DIRS = (ROOT / "part_1_foundations", ROOT / "part_2_models_control")


def lesson_directories() -> list[Path]:
    return sorted(
        {
            model_path.parent
            for part_dir in PART_DIRS
            for model_path in part_dir.rglob("model.xml")
        }
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
    return sorted(path for part_dir in PART_DIRS for path in part_dir.rglob("*.xml"))


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


def validate_lesson_invariants() -> list[str]:
    errors: list[str] = []

    def load(relative_path: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
        model = mujoco.MjModel.from_xml_path(str(ROOT / relative_path))
        return model, mujoco.MjData(model)

    def check(condition: bool, message: str) -> None:
        if not condition:
            errors.append(message)

    model, _ = load("part_1_foundations/01_empty_world/model.xml")
    check((model.nq, model.nv, model.nu) == (0, 0, 0), "P1-01 must remain an empty state model")
    check(np.isclose(model.opt.timestep, 0.002), "P1-01 timestep must be 0.002 s")

    model, _ = load("part_1_foundations/02_static_geoms/model.xml")
    check(model.nbody == 1 and model.nq == 0, "P1-02 geoms must remain fixed to world")
    check(model.ngeom == 6, "P1-02 must demonstrate six primitive geoms")

    model, data = load("part_1_foundations/03_gravity_and_ground/model.xml")
    body_id = model.body("falling_box").id
    mujoco.mj_forward(model, data)
    initial_z = float(data.xpos[body_id, 2])
    while data.time < 0.1:
        mujoco.mj_step(model, data)
    expected_z = initial_z - 0.5 * 9.81 * data.time**2
    check(data.ncon == 0, "P1-03 ballistic check must occur before contact")
    check(abs(float(data.xpos[body_id, 2]) - expected_z) < 2e-3, "P1-03 free-fall error exceeds 2 mm")

    model, data = load("part_1_foundations/04_body_hierarchy/model.xml")
    base_id = model.body("base").id
    arm_id = model.body("arm_link").id
    tool_id = model.body("tool").id
    check(int(model.body_parentid[arm_id]) == base_id, "P1-04 arm_link parent must be base")
    check(int(model.body_parentid[tool_id]) == arm_id, "P1-04 tool parent must be arm_link")
    check((model.nq, model.nv) == (1, 1), "P1-04 hierarchy must have exactly one hinge DOF")
    data.qpos[model.jnt_qposadr[model.joint("shoulder").id]] = np.pi / 2
    mujoco.mj_forward(model, data)
    check(np.isclose(np.linalg.norm(data.xpos[tool_id] - data.xpos[arm_id]), 0.8), "P1-04 tool offset must remain 0.8 m")

    model, _ = load("part_1_foundations/05_mesh_loading/model.xml")
    visual_id = model.geom("visual_mesh").id
    collision_id = model.geom("collision_mesh").id
    check(model.geom_contype[visual_id] == 0 and model.geom_conaffinity[visual_id] == 0, "P1-05 visual mesh must not collide")
    check(model.geom_contype[collision_id] != 0, "P1-05 collision mesh must participate in contact")

    builtin_model, _ = load("part_1_foundations/06_materials_and_textures/model.xml")
    file_model, _ = load("part_1_foundations/06_materials_and_textures/file_textures.xml")
    check((builtin_model.ntex, builtin_model.nmat) == (1, 2), "P1-06 built-in texture model contract changed")
    check((file_model.ntex, file_model.nmat) == (2, 3), "P1-06 file texture model contract changed")

    model, data = load("part_1_foundations/07_frames_and_orientation/model.xml")
    mujoco.mj_forward(model, data)
    rotations = [
        data.geom_xmat[model.geom(name).id].reshape(3, 3)
        for name in ("axisangle_box", "quaternion_box", "euler_box")
    ]
    check(np.allclose(rotations[0], rotations[1], atol=1e-5), "P1-07 axis-angle and quaternion disagree")
    check(np.allclose(rotations[0], rotations[2], atol=1e-8), "P1-07 axis-angle and Euler disagree")
    zaxis_rotation = data.geom_xmat[model.geom("zaxis_box").id].reshape(3, 3)
    check(np.allclose(zaxis_rotation[:, 2], np.ones(3) / np.sqrt(3), atol=1e-8), "P1-07 zaxis compilation changed")

    model, data = load("part_1_foundations/09_cameras/model.xml")
    check((model.ncam, model.nu, model.nsensordata) == (4, 1, 2), "P1-09 camera/actuator/sensor counts changed")
    slider_id = model.joint("slider").id
    actuator_id = model.actuator("slide_position").id
    qpos_adr = model.jnt_qposadr[slider_id]
    squared_error_sum = 0.0
    sample_count = 0
    max_abs_position = 0.0
    while data.time < 5.0:
        target = 1.25 * np.sin(2.0 * np.pi * 0.1 * data.time)
        position = float(data.qpos[qpos_adr])
        data.ctrl[actuator_id] = target
        squared_error_sum += (target - position) ** 2
        sample_count += 1
        max_abs_position = max(max_abs_position, abs(position))
        mujoco.mj_step(model, data)
    tracking_rmse = np.sqrt(squared_error_sum / sample_count)
    check(tracking_rmse < 0.05, "P1-09 position tracking RMSE exceeds 5 cm")
    check(max_abs_position <= 2.0 + 1e-6, "P1-09 moving camera body exceeded joint range")

    model, _ = load("part_1_foundations/08_lights/model.xml")
    check(
        model.light_type[model.light("sun").id] == mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        "P1-08 sun must remain directional",
    )
    check(
        model.light_type[model.light("blue_spot").id] == mujoco.mjtLightType.mjLIGHT_SPOT,
        "P1-08 blue_spot must remain a spot light",
    )
    check(np.isclose(model.light_cutoff[model.light("blue_spot").id], 30.0), "P1-08 spot cutoff must remain 30 deg")

    model, _ = load("part_1_foundations/10_inertial_properties/model.xml")
    inferred_id = model.body("uniform_cylinder").id
    explicit_id = model.body("offset_cylinder").id
    expected_mass = 1000.0 * np.pi * 0.1**2 * (2.0 * 0.4)
    check(np.isclose(model.body_mass[inferred_id], expected_mass), "P1-10 inferred cylinder mass disagrees with density*volume")
    check(np.isclose(model.body_mass[explicit_id], 2.0), "P1-10 explicit mass must remain 2 kg")
    check(np.allclose(model.body_ipos[explicit_id], [0.0, 0.2, 0.3]), "P1-10 explicit CoM offset changed")

    model, data = load("part_2_models_control/01_simple_pendulum/model.xml")
    joint_id = model.joint("hinge_y").id
    qpos_adr = model.jnt_qposadr[joint_id]
    dof_adr = model.jnt_dofadr[joint_id]
    data.qpos[qpos_adr] = 0.5
    mujoco.mj_forward(model, data)
    initial_energy = 0.5 * (1.0 + 1e-6) * data.qvel[dof_adr] ** 2 - 9.81 * np.cos(data.qpos[qpos_adr])
    while data.time < 1.0:
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
    final_energy = 0.5 * (1.0 + 1e-6) * data.qvel[dof_adr] ** 2 - 9.81 * np.cos(data.qpos[qpos_adr])
    check(final_energy < initial_energy, "P2-01 passive energy must decrease with positive damping")
    check(np.allclose(model.actuator_ctrlrange[0], [-3.0, 3.0]), "P2-01 torque limit contract changed")

    if not errors:
        print("PASS lesson-specific invariants for P1-01..10 and P2-01")
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


def validate_acrobot() -> list[str]:
    errors: list[str] = []
    model_path = ROOT / "part_2_models_control/03_acrobot/model.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    if (model.nq, model.nv, model.nu) != (2, 2, 1):
        errors.append("Acrobot must have two hinge DOFs and one actuator")
    elbow_id = model.joint("elbow_hinge").id
    actuator_joint_id = int(model.actuator_trnid[model.actuator("elbow_torque").id, 0])
    if actuator_joint_id != elbow_id:
        errors.append("Acrobot actuator must remain attached to the elbow hinge")
    if not np.allclose(model.actuator_ctrlrange[0], [-8.0, 8.0]):
        errors.append("Acrobot elbow torque limit must remain +/-8 N*m")

    def mujoco_energy(qpos: list[float], qvel: list[float]) -> float:
        data.qpos[:] = qpos
        data.qvel[:] = qvel
        mujoco.mj_forward(model, data)
        mujoco.mj_energyPos(model, data)
        mujoco.mj_energyVel(model, data)
        return float(np.sum(data.energy))

    downward_energy = mujoco_energy([0.0, 0.0], [0.0, 0.0])
    q1, q2 = 0.7, -0.4
    qvel = np.array([1.2, -0.8])
    measured_energy = mujoco_energy([q1, q2], qvel.tolist()) - downward_energy
    mass_matrix = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, mass_matrix, data.qM)
    expected_energy = (
        0.5 * qvel @ mass_matrix @ qvel
        + 2.0 * 9.81 * 0.7 * (1.0 - np.cos(q1))
        + 9.81 * 0.7 * (1.0 - np.cos(q1 + q2))
    )
    if not np.isclose(measured_energy, expected_energy, atol=1e-10):
        errors.append("Acrobot analytical energy disagrees with MuJoCo energy")

    data.qpos[:] = [np.pi, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    a_discrete = np.zeros((4, 4))
    b_discrete = np.zeros((4, 1))
    mujoco.mjd_transitionFD(
        model, data, 1e-6, True, a_discrete, b_discrete, None, None
    )
    controllability = np.hstack(
        [np.linalg.matrix_power(a_discrete, power) @ b_discrete for power in range(4)]
    )
    if np.linalg.matrix_rank(controllability) != 4:
        errors.append("Acrobot upright linearization must have controllability rank 4/4")

    if not errors:
        print("PASS acrobot actuation, energy and upright controllability contracts")
    return errors


def validate_so101() -> list[str]:
    errors: list[str] = []
    lesson_dir = ROOT / "part_2_models_control/04_so101_manipulator/01_reach_and_press"
    model = mujoco.MjModel.from_xml_path(str(lesson_dir / "model.xml"))
    expected_names = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )

    if (model.nq, model.nv, model.nu) != (7, 7, 6):
        errors.append("SO-101 task must have six robot hinges, one button slide and six actuators")
    for name in expected_names:
        joint_id = model.joint(name).id
        actuator_id = model.actuator(name).id
        if int(model.actuator_trnid[actuator_id, 0]) != joint_id:
            errors.append(f"SO-101 actuator {name} is not attached to its named joint")
    if model.nmesh != 13 or len(list((lesson_dir / "assets").glob("*.stl"))) != 13:
        errors.append("SO-101 must retain its 13 source meshes")
    if model.nkey != 1 or model.key("home").id != 0:
        errors.append("SO-101 must retain the named home keyframe")
    for site_name in ("gripperframe", "approach_target", "press_target", "button_touch"):
        if model.site(site_name).id < 0:
            errors.append(f"SO-101 task is missing site {site_name}")
    button_joint_id = model.joint("button_slide").id
    if model.jnt_type[button_joint_id] != mujoco.mjtJoint.mjJNT_SLIDE:
        errors.append("SO-101 button must use a slide joint")
    if model.sensor_type[model.sensor("button_force").id] != mujoco.mjtSensor.mjSENS_TOUCH:
        errors.append("SO-101 button_force must remain a touch sensor")

    controller_path = lesson_dir / "controller.py"
    spec = importlib.util.spec_from_file_location("so101_controller", controller_path)
    if spec is None or spec.loader is None:
        errors.append("SO-101 controller module could not be loaded")
    else:
        controller_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(controller_module)
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        controller = controller_module.ReachPressController(model, data)
        while data.time < 4.0:
            controller.apply(model, data)
            mujoco.mj_step(model, data)
        button_depth = float(data.qpos[model.jnt_qposadr[button_joint_id]])
        button_force_adr = model.sensor_adr[model.sensor("button_force").id]
        button_force = float(data.sensordata[button_force_adr])
        if not np.isfinite(controller.success_time):
            errors.append("SO-101 reference reach-and-press trajectory did not succeed within 4 s")
        if button_depth <= controller_module.PRESS_DEPTH_M:
            errors.append("SO-101 reference trajectory did not depress the button far enough")
        if button_force <= controller_module.PRESS_FORCE_N:
            errors.append("SO-101 reference trajectory did not produce the required touch force")

    if not errors:
        print("PASS SO-101 model, Jacobian controller and reach-and-press task contracts")
    return errors


def validate_mobile_robot() -> list[str]:
    errors: list[str] = []
    lesson_dir = ROOT / "part_2_models_control/05_mobile_robot"
    model = mujoco.MjModel.from_xml_path(str(lesson_dir / "model.xml"))

    if (model.nq, model.nv, model.nu) != (9, 8, 2):
        errors.append("Mobile robot must have one freejoint, two wheel hinges and two actuators")
    if model.jnt_type[model.joint("base_free").id] != mujoco.mjtJoint.mjJNT_FREE:
        errors.append("Mobile robot base_free must remain a freejoint")

    wheel_names = ("left", "right")
    for side in wheel_names:
        joint_id = model.joint(f"{side}_wheel_hinge").id
        actuator_id = model.actuator(f"{side}_wheel_velocity").id
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            errors.append(f"Mobile robot {side} wheel must remain a hinge")
        if int(model.actuator_trnid[actuator_id, 0]) != joint_id:
            errors.append(f"Mobile robot {side} actuator is not attached to its wheel")

    track_width = abs(model.body("left_wheel").pos[1] - model.body("right_wheel").pos[1])
    wheel_radius = model.geom("left_wheel_geom").size[0]
    if not np.isclose(track_width, 0.33) or not np.isclose(wheel_radius, 0.07):
        errors.append("Mobile robot wheel radius or track width changed")
    if not np.allclose(model.actuator_ctrlrange, [[-12.0, 12.0], [-12.0, 12.0]]):
        errors.append("Mobile robot wheel speed limits must remain +/-12 rad/s")
    if not np.allclose(model.actuator_forcerange, [[-1.5, 1.5], [-1.5, 1.5]]):
        errors.append("Mobile robot wheel torque limits must remain +/-1.5 N*m")
    for sensor_name in (
        "left_wheel_speed",
        "right_wheel_speed",
        "base_position",
        "base_orientation",
        "base_gyro",
    ):
        if model.sensor(sensor_name).id < 0:
            errors.append(f"Mobile robot is missing sensor {sensor_name}")
    for sensor_name in ("base_position", "base_orientation"):
        sensor_id = model.sensor(sensor_name).id
        if model.sensor_objtype[sensor_id] != mujoco.mjtObj.mjOBJ_XBODY:
            errors.append(f"Mobile robot {sensor_name} must measure the regular base frame")

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    initial_xy = data.body("base").xpos[:2].copy()
    while data.time < 2.0:
        mujoco.mj_step(model, data)
    if np.linalg.norm(data.body("base").xpos[:2] - initial_xy) >= 1e-3:
        errors.append("Mobile robot zero-control drift exceeds 1 mm in 2 s")

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    data.ctrl[:] = 6.0
    while data.time < 2.0:
        mujoco.mj_step(model, data)
    rotation = data.body("base").xmat.reshape(3, 3)
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    if data.body("base").xpos[0] <= 0.7 or abs(data.body("base").xpos[1]) >= 0.01 or abs(yaw) >= 0.01:
        errors.append("Equal positive wheel commands must drive the mobile robot straight along +x")

    controller_path = lesson_dir / "controller.py"
    spec = importlib.util.spec_from_file_location("mobile_robot_controller", controller_path)
    if spec is None or spec.loader is None:
        errors.append("Mobile robot controller module could not be loaded")
    else:
        controller_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(controller_module)
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        mujoco.mj_forward(model, data)
        controller = controller_module.WaypointController(model)
        while data.time < 10.0:
            controller.apply(model, data)
            mujoco.mj_step(model, data)
        goal_error = np.linalg.norm(
            data.body("base").xpos[:2] - controller_module.WAYPOINTS_M[-1]
        )
        if not np.isfinite(controller.success_time):
            errors.append("Mobile robot reference trajectory did not complete all waypoints within 10 s")
        if goal_error >= controller_module.WAYPOINT_RADIUS_M:
            errors.append("Mobile robot reference trajectory finished outside the goal radius")

    if not errors:
        print("PASS mobile robot geometry, contact baseline and waypoint task contracts")
    return errors


def validate_quadrotor() -> list[str]:
    errors: list[str] = []
    lesson_dir = ROOT / "part_2_models_control/06_quadrotor"
    model = mujoco.MjModel.from_xml_path(str(lesson_dir / "model.xml"))
    rotor_names = ("thrust_fl", "thrust_fr", "thrust_rr", "thrust_rl")

    if (model.nq, model.nv, model.nu) != (7, 6, 4):
        errors.append("Quadrotor must have one freejoint and four rotor controls")
    if model.jnt_type[model.joint("root").id] != mujoco.mjtJoint.mjJNT_FREE:
        errors.append("Quadrotor root must remain a freejoint")
    if not np.isclose(model.body("quadrotor").mass[0], 1.2):
        errors.append("Quadrotor mass must remain 1.2 kg")
    if not np.allclose(model.body("quadrotor").inertia, [0.025, 0.025, 0.045]):
        errors.append("Quadrotor principal inertia changed")
    if not np.allclose(model.actuator_ctrlrange, np.tile([0.0, 6.0], (4, 1))):
        errors.append("Quadrotor rotor thrust range must remain [0, 6] N")

    expected_wrenches = np.array(
        [
            [0.0, 0.0, 1.0, 0.18, -0.18, 0.02],
            [0.0, 0.0, 1.0, -0.18, -0.18, -0.02],
            [0.0, 0.0, 1.0, -0.18, 0.18, 0.02],
            [0.0, 0.0, 1.0, 0.18, 0.18, -0.02],
        ]
    )
    for index, actuator_name in enumerate(rotor_names):
        actuator_id = model.actuator(actuator_name).id
        if model.actuator_trntype[actuator_id] != mujoco.mjtTrn.mjTRN_SITE:
            errors.append(f"Quadrotor actuator {actuator_name} must use site transmission")
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("hover").id)
        data.ctrl[:] = 0.0
        data.ctrl[actuator_id] = 1.0
        mujoco.mj_forward(model, data)
        if not np.allclose(data.qfrc_actuator[:6], expected_wrenches[index], atol=1e-12):
            errors.append(f"Quadrotor actuator {actuator_name} wrench mapping changed")

    hover_thrust = 1.2 * 9.81 / 4.0
    if not np.allclose(model.key_ctrl[model.key("hover").id], hover_thrust):
        errors.append("Quadrotor hover keyframe must balance gravity")
    for sensor_name in ("position", "orientation", "linear_velocity"):
        sensor_id = model.sensor(sensor_name).id
        if model.sensor_objtype[sensor_id] != mujoco.mjtObj.mjOBJ_XBODY:
            errors.append(f"Quadrotor {sensor_name} must measure the regular body frame")

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("hover").id)
    mujoco.mj_forward(model, data)
    initial_position = data.sensor("position").data.copy()
    while data.time < 2.0:
        mujoco.mj_step(model, data)
    if np.linalg.norm(data.sensor("position").data - initial_position) >= 1e-9:
        errors.append("Quadrotor hover keyframe drift exceeds 1 nm in 2 s")
    if np.linalg.norm(data.sensor("linear_velocity").data) >= 1e-9:
        errors.append("Quadrotor hover keyframe develops nonzero velocity")

    controller_path = lesson_dir / "controller.py"
    spec = importlib.util.spec_from_file_location("quadrotor_controller", controller_path)
    if spec is None or spec.loader is None:
        errors.append("Quadrotor controller module could not be loaded")
    else:
        controller_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(controller_module)
        data = mujoco.MjData(model)
        data.qpos[:3] = [0.25, -0.20, 0.75]
        mujoco.mju_euler2Quat(data.qpos[3:7], np.deg2rad([10.0, -8.0, 15.0]), "xyz")
        data.ctrl[:] = hover_thrust
        mujoco.mj_forward(model, data)
        controller = controller_module.HoverController(model)
        saturated_steps = 0
        step_count = 0
        while data.time < 6.0:
            metrics = controller.apply(model, data)
            saturated_steps += int(metrics["saturated"])
            step_count += 1
            mujoco.mj_step(model, data)
        target = data.site_xpos[model.site("hover_target").id]
        position_error = np.linalg.norm(data.sensor("position").data - target)
        quaternion_norm_error = abs(np.linalg.norm(data.sensor("orientation").data) - 1.0)
        if not np.isfinite(controller.success_time):
            errors.append("Quadrotor reference recovery did not satisfy hover dwell within 6 s")
        if position_error >= 0.03:
            errors.append("Quadrotor reference recovery final position error exceeds 3 cm")
        if quaternion_norm_error >= 1e-12:
            errors.append("Quadrotor quaternion norm drift exceeds tolerance")
        if saturated_steps != 0:
            errors.append("Quadrotor reference recovery unexpectedly saturates a rotor")

    if not errors:
        print("PASS quadrotor site wrenches, hover equilibrium and recovery contracts")
    return errors


def main() -> int:
    errors = (
        validate_structure()
        + validate_models()
        + validate_lesson_invariants()
        + validate_cartpole_linearization()
        + validate_acrobot()
        + validate_so101()
        + validate_mobile_robot()
        + validate_quadrotor()
    )
    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        print(f"validation failed with {len(errors)} error(s)", file=sys.stderr)
        return 1
    print(f"validation passed for {len(lesson_directories())} lessons and {len(model_paths())} models")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
