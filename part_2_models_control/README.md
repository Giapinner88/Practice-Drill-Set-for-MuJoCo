# Phần 2 — Models and Control

Phần này nối phương trình động lực học với model MJCF và controller. Bạn sẽ dùng cùng một quy ước state, tham số và timestep trong phần tính toán lẫn simulation.

| Bài | Nội dung |
| --- | --- |
| [01 — Simple Pendulum](01_simple_pendulum) | Phase portrait, cơ năng và energy-shaping control |
| [02 — Cart-pole](02_cartpole) | Linearization, controllability, swing-up và LQR |
| [03 — Acrobot](03_acrobot) | Underactuation, partial feedback linearization, energy shaping và LQR |
| [04 — SO-101 Manipulator](04_so101_manipulator) | Reach-and-press; pick-and-place và push-to-target đang phát triển |
| [05 — Mobile Robot](05_mobile_robot) | Differential-drive kinematics, wheel contact, odometry và waypoint control |
| [06 — Quadrotor](06_quadrotor) | Site thrust, wrench mixing, cascaded control và hover recovery |

Khi so sánh hai controller hoặc hai bộ tham số, hãy giữ nguyên initial condition, timestep và thời gian chạy. Dùng `--duration N` để các lần chạy có cùng simulation horizon.

Viewer không tự đóng nếu bạn không truyền `--duration`.
