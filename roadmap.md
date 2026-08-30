# Lộ trình phát triển giáo trình

Roadmap này phản ánh nội dung thực sự có trong repo. Ký hiệu:

- ✅: đã có bài và đang được chuẩn hóa;
- 🟡: đã lên thiết kế nhưng chưa có bài hoàn chỉnh;
- ⬜: dự kiến.

## Phần 1 — MuJoCo Foundations

| Trạng thái | Bài | Chủ đề | Trọng tâm MJCF/API |
| --- | --- | --- | --- |
| ✅ | 01 | Empty world và simulation loop | `<mujoco>`, `<option>`, `<worldbody>`, `MjModel`, `MjData`, `mj_step` |
| ✅ | 02 | Static geoms | `<geom>`, primitive geometry, `size`, `pos`, `rgba` |
| ✅ | 03 | Gravity và ground contact | `<option gravity>`, plane, dynamic body, `freejoint` |
| ✅ | 04 | Body hierarchy | `<body>`, local frame, `freejoint`, `nq`, `nv` |
| ✅ | 05 | Mesh loading | `<asset>`, `<mesh>`, visual/collision separation |
| ✅ | 06 | Materials và textures | `<texture>`, `<material>`, built-in/file texture |
| ✅ | 07 | Frames và orientation | `quat`, `euler`, `axisangle`, `zaxis` |
| ✅ | 08 | Cameras | `<camera>`, camera modes, actuator/sensor introduction |
| ✅ | 09 | Lights | `<light>`, diffuse/specular, directional/positional light |
| ✅ | 10 | Inertial properties | `<inertial>`, mass, CoM, inertia tensor |
| 🟡 | 11 | Joints và state indexing | hinge/slide/ball/free, `jnt_qposadr`, `jnt_dofadr` |
| 🟡 | 12 | Actuators | motor/position/velocity, gain/bias, limits |
| 🟡 | 13 | Contacts và solver | contact pairs, friction, `solref`, `solimp` |
| 🟡 | 14 | Sensors | joint, IMU, force/torque, touch, `sensordata` |
| 🟡 | 15 | Simulation pipeline | `mj_forward`, `mj_step1`, `mj_step2`, reset/keyframe |
| ⬜ | 16 | Constraints và tendons | `<equality>`, fixed/spatial tendon |
| ⬜ | 17 | Rendering và pixels | `Renderer`, RGB, depth, segmentation |
| ⬜ | 18 | Model composition | `<default>`, `<include>`, assets và reusable MJCF |

## Phần 2 — Modeling and Control

| Trạng thái | Bài | Case study | Điều kiện kiểm chứng |
| --- | --- | --- | --- |
| ✅ | 01 | Simple Pendulum | năng lượng, phase portrait, torque saturation |
| ✅ | 02 | Cart-pole | linearization, controllability, swing-up và LQR |
| 🟡 | 03 | Acrobot/Pendubot | underactuation và energy shaping |
| 🟡 | 04 | Planar Manipulator | forward kinematics, Jacobian, IK |
| ⬜ | 05 | Manipulator control | gravity compensation, joint/task-space control |
| ⬜ | 06 | Contact manipulation | gripper, friction, contact force, pick-and-place |

## Phần 3 — Reinforcement Learning với `mjlab`

Trước khi viết code, repo và phiên bản `mjlab` sẽ được pin rõ để tránh nhầm với MJX hoặc MuJoCo Playground.

| Trạng thái | Bài | Chủ đề | Bằng chứng đầu ra |
| --- | --- | --- | --- |
| 🟡 | 01 | Anatomy of an RL task | observation/action/reward/termination contract |
| 🟡 | 02 | Vectorized training | throughput và reproducible seed |
| ⬜ | 03 | PPO baseline | learning curve và checkpoint |
| ⬜ | 04 | Reward engineering | reward ablation |
| ⬜ | 05 | Domain randomization | held-out evaluation |
| ⬜ | 06 | Policy deployment | controller baseline và policy comparison |

## Thứ tự phát triển

1. Làm cho mọi bài hiện có chạy độc lập và kiểm tra được.
2. Hoàn thiện coverage còn thiếu của Phần 1.
3. Hoàn thiện bốn model/control case study cốt lõi.
4. Chốt dependency và API contract cho `mjlab`.
5. Chỉ công bố kết quả RL khi có cấu hình, seed và artifact tái lập được.
