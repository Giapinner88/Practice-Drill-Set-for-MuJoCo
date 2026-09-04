# MuJoCo Practice Drill Set

Đây là tập bài học thực hành MuJoCo bằng tiếng Việt. Bạn sẽ bắt đầu với một world trống, học cách đọc model và state qua Python, sau đó chuyển sang các hệ động lực học như Pendulum, Cart-pole và Acrobot. Phần cuối sử dụng `mjlab` để xây dựng môi trường reinforcement learning.

## Bạn sẽ học gì

- Viết model MJCF với body, geom, joint, asset, camera, light, actuator và sensor.
- Đọc cấu trúc đã biên dịch qua `mujoco.MjModel`.
- Đọc và thay đổi trạng thái simulation qua `mujoco.MjData`.
- Sử dụng `mj_forward`, `mj_step`, `mj_resetData`, `qpos`, `qvel`, `qacc` và `ctrl`.
- So sánh kết quả MuJoCo với phương trình giải tích.
- Cài đặt controller cho Pendulum, Cart-pole, Acrobot và Manipulator.
- Xây dựng observation, action, reward và reset cho một bài toán reinforcement learning.

## Các phần của giáo trình

### [Phần 1 — MuJoCo Foundations](part_1_foundations)

Bắt đầu từ MJCF và Python API: world, geometry, body tree, mesh, material, frame, camera, light và inertia.

### [Phần 2 — Models and Control](part_2_models_control)

Áp dụng MuJoCo vào các hệ động lực học cụ thể. Hiện có Simple Pendulum, Cart-pole, Acrobot, SO-101 Manipulator, Differential-drive Mobile Robot và Quadrotor.

### [Phần 3 — Reinforcement Learning với `mjlab`](part_3_reinforcement_learning)

Phần này sẽ sử dụng các model đã học để xây dựng và huấn luyện RL task.

## Cài đặt

Yêu cầu Python 3.10 trở lên:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
```

Kiểm tra MuJoCo đã được cài:

```bash
python -c "import mujoco; print(mujoco.__version__)"
```

## Học một bài

Mỗi thư mục bài học có:

- `README.md`: phần giải thích và bài tập;
- `model.xml`: model MJCF;
- `simulate.py`: chương trình chạy model.

Ví dụ:

```bash
python part_1_foundations/01_empty_world/simulate.py
```

Viewer chạy đến khi bạn đóng cửa sổ hoặc nhấn ESC. Nếu muốn giới hạn thời gian:

```bash
python part_1_foundations/01_empty_world/simulate.py --duration 20
```

Chạy không cần viewer:

```bash
python part_1_foundations/01_empty_world/simulate.py --headless --duration 1
```

Bạn nên đọc `README.md`, mở `model.xml`, dự đoán kết quả rồi mới chạy script. Sau đó thay đổi một tham số và giải thích sự khác biệt bằng đại lượng được in ra terminal hoặc biểu đồ.

## Bài Cart-pole

Cart-pole cần tạo gain LQR trước khi chạy controller:

```bash
python part_2_models_control/02_cartpole/analytical_linearization.py
python part_2_models_control/02_cartpole/mujoco_linearization.py
python part_2_models_control/02_cartpole/simulate_swingup_lqr.py
```

## Bài Acrobot

Acrobot cũng cần tạo gain LQR trước khi chạy controller lai PFL–LQR:

```bash
python part_2_models_control/03_acrobot/linearize_lqr.py
python part_2_models_control/03_acrobot/simulate.py
```

## Bài SO-101

P2-04 gồm ba task: reach-and-press đã hoàn thiện; pick-and-place và push-to-target đang phát triển. Xem [danh sách task và trạng thái hiện tại](part_2_models_control/04_so101_manipulator).

Mở model ở keyframe `home` và tự điều khiển sáu actuator bằng slider trong bảng **Control** của viewer:

```bash
python part_2_models_control/04_so101_manipulator/01_reach_and_press/simulate.py
```

Sau khi đã làm quen với chuyển động của từng khớp, chạy demo Cartesian reach-and-press bằng Jacobian IK và touch sensor:

```bash
python part_2_models_control/04_so101_manipulator/01_reach_and_press/demo.py
python part_2_models_control/04_so101_manipulator/01_reach_and_press/demo.py --headless --duration 4
```

## Bài Mobile Robot

Tự điều khiển hai bánh bằng slider hoặc chạy demo qua ba waypoint:

```bash
python part_2_models_control/05_mobile_robot/simulate.py
python part_2_models_control/05_mobile_robot/demo.py --headless --duration 15
```

## Bài Quadrotor

Tự thay đổi bốn rotor thrust bằng slider hoặc chạy demo hover recovery:

```bash
python part_2_models_control/06_quadrotor/simulate.py
python part_2_models_control/06_quadrotor/demo.py --headless --duration 8
```

## Tài liệu MuJoCo

- [MuJoCo documentation](https://mujoco.readthedocs.io/)
- [MJCF XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)
- Todorov, Erez, Tassa, “MuJoCo: A physics engine for model-based control,” IROS 2012.
