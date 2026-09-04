# P2-02 — Cart-pole: linearization, swing-up và LQR

## Câu hỏi trung tâm

Làm thế nào kiểm tra rằng linear model dùng để thiết kế LQR nhất quán với model MuJoCo và đúng loại thời gian liên tục/rời rạc?

## Sau bài này, bạn có thể

- xây dựng phương trình cart-pole point-mass;
- linearize giải tích quanh upright;
- lấy transition Jacobian rời rạc bằng `mjd_transitionFD`;
- kiểm tra controllability;
- thiết kế discrete-time LQR;
- kết hợp swing-up và balance bằng điều kiện chuyển mode rõ ràng.

## State và quy ước

```text
state = [x, theta, x_dot, theta_dot]
theta = 0   : pole hướng xuống
theta = pi  : pole hướng lên
u           : lực tác dụng lên cart, giới hạn ±20 N
```

Model dùng cart mass 1 kg, pole point mass 1 kg cách pivot 0.5 m, timestep 0.002 s và Euler integrator. Euler được chọn tường minh vì `mjd_transitionFD` không hỗ trợ RK4; như vậy Jacobian rời rạc và simulation dùng cùng transition rule. Rail trong model chỉ là visual; hiện chưa có joint range, vì vậy controller phải quản lý vị trí cart bằng cost/feedback.

## Ba script

### `analytical_linearization.py`

Xây dựng $M(q)\ddot q+C(q,\dot q)+G(q)=Bu$ bằng SymPy và tính Jacobian liên tục tại upright. Kết quả là $\dot x=A_cx+B_cu$.

### `mujoco_linearization.py`

Gọi `mjd_transitionFD`, trả về transition Jacobian rời rạc:

$$
x_{k+1}=A_dx_k+B_du_k.
$$

Do đó gain được thiết kế bằng DARE, không dùng CARE. Script kiểm tra rank của controllability matrix và lưu gain vào `artifacts/lqr_gain.npy` cùng metadata cơ bản.

### `simulate_swingup_lqr.py`

Script ghi mode, state và control rồi tạo biểu đồ. Khi pole đi vào vùng gần upright với vận tốc đủ nhỏ, controller chuyển từ swing-up sang LQR. Hai ngưỡng vào/ra khác nhau giúp tránh chuyển mode liên tục.

Trong pha swing-up, lực điều khiển gồm hai phần:

$$
u = -k_E(E-E_d)\dot{\theta}\cos\theta-k_xx-k_v\dot{x}.
$$

Các giá trị mặc định là $k_E=15$, $k_x=16$ và $k_v=2.5$. Thành phần đầu bơm hoặc rút năng lượng; hai thành phần sau hạn chế cart đi quá xa tâm rail. Tăng $k_x$ thường làm hành trình ngắn hơn nhưng cũng có thể ngăn pole tích đủ năng lượng để swing-up.

## Chạy

```bash
python part_2_models_control/02_cartpole/analytical_linearization.py
python part_2_models_control/02_cartpole/mujoco_linearization.py
python part_2_models_control/02_cartpole/simulate_swingup_lqr.py
```

## Kiểm tra kết quả

1. So $A_d,B_d$ với xấp xỉ $I+A_c\Delta t, B_c\Delta t$ khi timestep nhỏ.
2. Báo controllability rank 4/4 trước khi giải DARE.
3. Kiểm tra residual của discrete Riccati equation.
4. Đánh giá balance bằng angle error, cart position, saturation rate và dwell time.
5. Lặp nhiều initial conditions; không suy rộng từ một trajectory.

## Sai lầm thường gặp

- Dùng continuous-time LQR trực tiếp với Jacobian rời rạc.
- Nhầm thứ tự state giữa code giải tích và MuJoCo.
- Quên wrap angle quanh $\pi$.
- Nói LQR “swing-up” trong khi nó chỉ ổn định cục bộ.
- Load gain không cùng model/timestep đã dùng khi tạo gain.

## Tài liệu

- Russ Tedrake, *Underactuated Robotics*, chương Acrobot and Cart-pole.
- [MuJoCo derivative API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)
