# MuJoCo coverage matrix

Bảng này giúp người biên soạn biết nhóm tag/API nào đã có bài và nhóm nào còn thiếu.

| Nhóm | Tag/API tiêu biểu | Bài giới thiệu | Trạng thái |
| --- | --- | --- | --- |
| Root/configuration | `<mujoco>`, `<compiler>`, `<option>`, `<size>` | P1-01, P1-03 | một phần |
| Kinematic tree | `<worldbody>`, nested `<body>`, welded frame, `<frame>` | P1-01, P1-04, P1-07 | một phần |
| Geometry | `<geom>`, primitives, mesh | P1-02, P1-05 | có bài |
| Inertia | `<inertial>`, density, mass, inertia | P1-04, P1-10 | có bài |
| Assets | `<asset>`, `<mesh>`, `<texture>`, `<material>` | P1-05, P1-06; P2-04 | có bài |
| Orientation | `quat`, `euler`, `axisangle`, `xyaxes`, `zaxis` | P1-07 | có bài |
| Visual scene | `<camera>`, `<light>`, `<visual>` | P1-08, P1-09 | có bài |
| Joints | hinge, slide, ball, free | P1-11; P2-01/02/03/04/05/06 | đang phát triển |
| Actuation | motor, position, velocity, site transmission, gain/bias | P1-12; P2-01/02/03/04/05/06 | đang phát triển |
| Contact | friction, pair/exclude, solver parameters | P1-13; P2-04/05 | đang phát triển |
| Sensors | joint, actuator force, IMU, force/torque, touch | P1-14; P2-03/04/05/06 | đang phát triển |
| Constraints | equality, tendon | P1-16 | dự kiến |
| Model/data | `MjModel`, `MjData`, named access | P1-01 trở đi | một phần |
| Forward simulation | `mj_forward`, `mj_step`, step1/step2 | P1-01, P1-15 | một phần |
| State/reset | `qpos`, `qvel`, `ctrl`, reset/keyframe | P1-15; P2-04/05/06 | đang phát triển |
| Dynamics | `mj_fullM`, `qfrc_bias`, `qfrc_passive`, `mj_jacSite`, wrench mixing, inverse dynamics | P2-02/03/04/06 | đang phát triển |
| Rendering | viewer, `Renderer`, RGB, depth, video; segmentation | P1-09; P1-17 | RGB/depth/video có bài, segmentation dự kiến |
| RL task API | env config, observation, reward, reset | P3-01 | dự kiến |

Khi thêm bài mới, phải cập nhật cả bảng này và roadmap.
