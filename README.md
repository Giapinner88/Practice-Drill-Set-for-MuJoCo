# MuJoCo Practice Drill Set

Giáo trình thực hành MuJoCo theo hướng **học bằng mô hình, đo lường và kiểm chứng**. Repo đưa người học đi từ MJCF và Python API, qua các bài toán modeling/control cổ điển, đến reinforcement learning với `mjlab`.

## Mục tiêu học tập

Sau khi hoàn thành lộ trình, người học có thể:

- đọc, viết và kiểm tra một mô hình MJCF có cấu trúc;
- phân biệt dữ liệu tĩnh trong `mujoco.MjModel` với trạng thái runtime trong `mujoco.MjData`;
- sử dụng simulation pipeline như `mj_forward`, `mj_step`, `mj_resetData` và các mảng `qpos`, `qvel`, `qacc`, `ctrl`;
- xây dựng và kiểm chứng các model động lực học cơ bản;
- cài đặt controller cổ điển và đánh giá bằng đại lượng định lượng;
- chuyển một model đã kiểm chứng thành môi trường RL có quy trình đánh giá tái lập được.

Mục tiêu “phủ MuJoCo” được hiểu là làm chủ các nhóm tag/API cốt lõi. Các tính năng chuyên biệt được theo dõi trong [bảng coverage](docs/coverage.md), không nhồi tất cả vào một bài.

## Cấu trúc giáo trình

### Phần 1 — Foundations

[part_1_foundations](part_1_foundations) giới thiệu MJCF, cấu trúc vật rắn, assets, rendering và simulation loop. Python API được sử dụng ngay từ bài đầu thay vì để đến cuối lộ trình.

### Phần 2 — Models and Control

[part_2_models_control](part_2_models_control) dùng Pendulum, Cart-pole, Acrobot và Manipulator làm case study để nối phương trình giải tích, model MuJoCo và controller.

### Phần 3 — Reinforcement Learning

[part_3_reinforcement_learning](part_3_reinforcement_learning) sẽ trình bày cách xây dựng task RL với `mjlab`, từ observation/action/reward đến training, evaluation và domain randomization.

Xem [roadmap.md](roadmap.md) để biết nội dung đã có và phần đang dự kiến.

## Cài đặt

Yêu cầu Python 3.10 trở lên. Tạo môi trường ảo riêng trước khi cài:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[test]'
```

Kiểm tra cài đặt:

```bash
python -c "import mujoco; print(mujoco.__version__)"
python tools/validate_repo.py
```

## Chạy một bài

Mỗi bài có `README.md`, `model.xml` và script chạy cục bộ. Script dùng đường dẫn dựa trên `__file__`, vì vậy có thể gọi từ thư mục bất kỳ:

```bash
python part_1_foundations/01_empty_world/simulate.py
python part_1_foundations/01_empty_world/simulate.py --headless --duration 1
```

Với bài Cart-pole:

```bash
python part_2_models_control/02_cartpole/analytical_linearization.py
python part_2_models_control/02_cartpole/mujoco_linearization.py
python part_2_models_control/02_cartpole/simulate_swingup_lqr.py
```

## Nguyên tắc học thuật

- Mọi đại lượng phải ghi đơn vị và quy ước hệ tọa độ.
- Phân biệt giả thiết mô hình, kết quả giải tích và kết quả đo từ simulation.
- Một lệnh chạy thành công không tự chứng minh mô hình đúng.
- Mỗi claim định lượng cần phép đo, baseline hoặc điều kiện kiểm chứng.
- Controller phải công bố giới hạn actuator, timestep và trạng thái cân bằng.
- Kết quả RL phải ghi seed, số episode, checkpoint và tiêu chí đánh giá.

Chuẩn chi tiết cho một bài nằm trong [docs/lesson_standard.md](docs/lesson_standard.md).

## Tài liệu chính

- [MuJoCo documentation](https://mujoco.readthedocs.io/)
- [MJCF XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)
- Todorov, Erez, Tassa, “MuJoCo: A physics engine for model-based control,” IROS 2012.
