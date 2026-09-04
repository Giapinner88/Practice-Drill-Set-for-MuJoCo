# P2-01 — Simple Pendulum

## Câu hỏi trung tâm

Làm thế nào nối một phương trình phi tuyến, một model MJCF và một controller energy-shaping sao cho các giả thiết giữa ba phần nhất quán?

## Sau bài này, bạn có thể

- xác định state $(\theta,\dot\theta)$ từ `qpos`/`qvel`;
- kiểm tra quy ước $\theta=0$ hướng xuống, $\theta=\pi$ hướng lên;
- tính cơ năng theo inertial model;
- áp torque có saturation;
- phân biệt việc chạm vị trí upright với việc giữ ổn định quanh upright.

## Mô hình và giả thiết

Model xấp xỉ point mass $m=1$ kg tại $l=1$ m; rod chỉ để hiển thị và không tham gia collision/inertia. Body có inertia rất nhỏ tại CoM để xấp xỉ point mass. Với $J=ml^2+I_C$:

$$
J\ddot\theta+b\dot\theta+mgl\sin\theta=u.
$$

Cơ năng theo mốc của model:

$$
E=\frac12J\dot\theta^2-mgl\cos\theta,
\qquad E_{up}=mgl.
$$

Luật energy-shaping trong script:

$$
u=\operatorname{clip}\left[-k\dot\theta(E-E_{up})+b\dot\theta,
-u_{max},u_{max}\right].
$$

Hạng $b\dot\theta$ bù damping đã khai báo trong model. Khi torque bị giới hạn, giá trị thực sự đưa vào `data.ctrl` là giá trị sau phép `clip`; hãy theo dõi `saturation_fraction` để biết giới hạn này xuất hiện bao nhiêu lần.

## Chạy

```bash
python part_2_models_control/01_simple_pendulum/simulate.py
python part_2_models_control/01_simple_pendulum/simulate.py --headless --duration 10
```

Biểu đồ được ghi vào `artifacts/swingup_analysis.png`. Terminal hiển thị gain, torque limit, energy error cuối và tỉ lệ bước bị saturation.

## Thí nghiệm

1. Passive: đặt `k=0`, damping bằng zero; đo độ trôi năng lượng.
2. Gain sweep: $k\in\{0.1,0.5,2.0\}$; báo thời gian đạt vùng gần upright và tỉ lệ bước bị saturation.
3. Actuator limit: $u_{max}\in\{0.5,1,3\}$ N·m; xác định trường hợp không đạt mục tiêu trong horizon cố định.
4. Model mismatch: controller dùng $l$ sai ±10%; so energy error và capture rate.

## Tiêu chí đánh giá

- Không gọi “stabilized” nếu chỉ chạm upright rồi rời đi.
- Capture cần điều kiện đồng thời trên angle error và angular velocity trong một khoảng dwell time đã nêu.
- Giữ nguyên initial condition, horizon và timestep khi so sánh hai bộ tham số.

## Tài liệu

- Russ Tedrake, *Underactuated Robotics*, chương Simple Pendulum.
- [MuJoCo actuators](https://mujoco.readthedocs.io/en/stable/modeling.html#actuation-model)
