# MuJoCo coverage matrix

Bảng này là contract phạm vi của giáo trình, không phải danh sách tuyên bố rằng mọi tính năng đã hoàn thiện.

| Nhóm | Tag/API tiêu biểu | Bài giới thiệu | Trạng thái |
| --- | --- | --- | --- |
| Root/configuration | `<mujoco>`, `<compiler>`, `<option>`, `<size>` | P1-01, P1-03 | một phần |
| Kinematic tree | `<worldbody>`, `<body>`, `<frame>` | P1-01, P1-04, P1-07 | một phần |
| Geometry | `<geom>`, primitives, mesh | P1-02, P1-05 | có bài |
| Inertia | `<inertial>`, density, mass, inertia | P1-04, P1-10 | có bài |
| Assets | `<asset>`, `<mesh>`, `<texture>`, `<material>` | P1-05, P1-06 | có bài |
| Orientation | `quat`, `euler`, `axisangle`, `xyaxes`, `zaxis` | P1-07 | có bài |
| Visual scene | `<camera>`, `<light>`, `<visual>` | P1-08, P1-09 | có bài |
| Joints | hinge, slide, ball, free | P1-11; P2-01/02 | đang phát triển |
| Actuation | motor, position, velocity, gain/bias | P1-12; P2-01/02 | đang phát triển |
| Contact | friction, pair/exclude, solver parameters | P1-13 | dự kiến |
| Sensors | joint, IMU, force/torque, touch | P1-14 | dự kiến |
| Constraints | equality, tendon | P1-16 | dự kiến |
| Model/data | `MjModel`, `MjData`, named access | P1-01 trở đi | một phần |
| Forward simulation | `mj_forward`, `mj_step`, step1/step2 | P1-01, P1-15 | một phần |
| State/reset | `qpos`, `qvel`, `ctrl`, reset/keyframe | P1-15; P2 | đang phát triển |
| Dynamics | mass matrix, inverse dynamics, Jacobian | P2-02/04 | đang phát triển |
| Rendering | viewer, `Renderer`, depth/segmentation | P1-08/17 | một phần |
| RL task API | env config, observation, reward, reset | P3-01 | dự kiến |

Khi thêm bài mới, phải cập nhật cả bảng này và roadmap.
