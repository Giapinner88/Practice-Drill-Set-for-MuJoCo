# P2-03 — Acrobot: underactuation, PFL và energy shaping

## Câu hỏi trung tâm

Một actuator ở khớp khuỷu có thể đưa cả hai link lên tư thế thẳng đứng bằng cách nào, dù khớp vai hoàn toàn thụ động?

## Sau bài này, bạn có thể

- nhận biết một hệ underactuated từ số bậc tự do và số actuator;
- đọc ma trận khối lượng đầy đủ bằng `mj_fullM`;
- phân biệt `qfrc_bias`, `qfrc_passive` và actuator torque;
- xây dựng collocated partial feedback linearization (PFL);
- dùng energy shaping để tạo chuyển động swing-up;
- kiểm tra controllability và thiết kế discrete-time LQR quanh upright;
- đánh giá controller lai bằng sai số state, thời điểm bắt LQR và torque saturation.

Bạn nên hoàn thành P2-01 và P2-02 trước bài này.

## Model và quy ước

```text
q1 = 0       : link 1 hướng xuống
q2 = 0       : link 2 thẳng hàng với link 1
upright      : q1 = pi, q2 = 0
state        : [q1 - pi, q2, q1_dot, q2_dot]
actuator     : torque tại q2; q1 không có actuator
```

Mỗi link được mô hình hóa bằng một point mass `1 kg` ở cuối link dài `0.7 m`; inertia tại point mass là `1e-6 kg*m^2` để tensor quán tính dương. Hai hinge quay quanh trục $+y$, nên chuyển động nằm trong mặt phẳng $x$–$z$. Timestep là `0.002 s`, Euler integrator và giới hạn torque là $\pm8\ \mathrm{N\,m}$.

Model có hai velocity DOF nhưng chỉ một control input:

$$
M(q)\ddot q+h(q,\dot q)=
\begin{bmatrix}0\\u\end{bmatrix}.
$$

Vì vậy không thể đặt độc lập $\ddot q_1$ và $\ddot q_2$ tại cùng một thời điểm. Underactuation không đồng nghĩa với uncontrollability: linearization quanh upright của model này có controllability rank `4/4`.

## Cơ năng

Lấy cấu hình thẳng xuống làm mốc thế năng bằng không:

$$
V(q)=2mgl(1-\cos q_1)+mgl\left[1-\cos(q_1+q_2)\right].
$$

Động năng được tính trực tiếp từ ma trận khối lượng do MuJoCo biên dịch:

$$
T(q,\dot q)=\frac{1}{2}\dot q^TM(q)\dot q.
$$

Năng lượng mục tiêu tại upright là

$$
E_d=2g(2l+l)=41.202\ \mathrm{J},
$$

với $m=1\ \mathrm{kg}$ và $l=0.7\ \mathrm{m}$ cho cả hai link.

## Từ gia tốc mong muốn đến torque bằng PFL

Chia phương trình động lực học thành hai hàng:

$$
M_{11}\ddot q_1+M_{12}\ddot q_2+h_1=0,
$$

$$
M_{21}\ddot q_1+M_{22}\ddot q_2+h_2=u.
$$

Chọn $\ddot q_2^d$ rồi giải hàng thụ động trước:

$$
\ddot q_1=-\frac{h_1+M_{12}\ddot q_2^d}{M_{11}}.
$$

Torque tại khuỷu là

$$
u=M_{21}\ddot q_1+M_{22}\ddot q_2^d+h_2.
$$

Trong code, $h=\mathtt{qfrc\_bias}-\mathtt{qfrc\_passive}$ vì damping của joint nằm trong `qfrc_passive`. Luật tạo gia tốc khuỷu là

$$
\ddot q_2^d=-k_1q_2-k_2\dot q_2+k_3\dot q_1(E-E_d),
$$

với $k_1=8$, $k_2=4$ và $k_3=1$. Hai số hạng đầu giữ khuỷu gần cấu hình thẳng; số hạng cuối trao đổi năng lượng với chuyển động của vai. Các số hạng PD hữu ích trong simulation nhưng không tự tạo thành một chứng minh hội tụ toàn cục.

## LQR chỉ dùng gần upright

`linearize_lqr.py` gọi `mjd_transitionFD` tại $(q_1,q_2,\dot q_1,\dot q_2)=(\pi,0,0,0)$. Jacobian nhận được là Jacobian của một bước `mj_step`, nên controller giải discrete algebraic Riccati equation (DARE).

PFL chuyển sang LQR khi đồng thời thỏa:

```text
|q1 - pi| < 0.25 rad
|q2|      < 0.30 rad
max(|q1_dot|, |q2_dot|) < 2.0 rad/s
```

Ngưỡng thoát lớn hơn ngưỡng vào. Controller có thể thử bắt upright, trở lại swing-up nếu vận tốc còn quá lớn, rồi bắt lại ở lần sau.

## Chạy bài

```bash
python part_2_models_control/03_acrobot/linearize_lqr.py
python part_2_models_control/03_acrobot/simulate.py
```

Viewer chạy đến khi bạn đóng cửa sổ. Để tạo một lần chạy hữu hạn và biểu đồ:

```bash
python part_2_models_control/03_acrobot/simulate.py --headless --duration 20
```

Với initial condition mặc định $q_1=0.1\ \mathrm{rad}$, $q_2=0$ và vận tốc bằng không, trajectory tham chiếu vào LQR khoảng `8.5 s` rồi giữ upright. Vận tốc lớn nhất khoảng `18.5 rad/s` và torque bão hòa khoảng `13.8%` số bước trong lần chạy `20 s`. Đây là baseline cho đúng model, timestep và gain trong bài; thay đổi một trong các điều kiện này cần đo lại.

## Quan sát và kiểm tra

1. Xác nhận `controllability_rank=4/4` và Riccati residual nhỏ trước khi chạy controller.
2. Trên biểu đồ năng lượng, kiểm tra $E(t)$ tiến đến vùng lân cận $E_d$ trong pha swing-up.
3. Phân biệt thời điểm chạm gần upright với thời điểm LQR giữ được state.
4. Kiểm tra torque có nằm trong $\pm8\ \mathrm{N\,m}$ và đọc `saturation_fraction`.
5. Đổi `--initial-angle`; không giả định gain mặc định làm việc với mọi initial condition.

## Bài tập

1. Đặt `ELBOW_POSITION_GAIN = 0`, so sánh chuyển động của $q_2$ và khả năng bắt upright.
2. Giảm torque limit xuống $6\ \mathrm{N\,m}$ trong cả XML và Python, sau đó đo lại thời điểm vào LQR.
3. Thay $Q$ hoặc $R$, tạo lại gain và giải thích thay đổi của torque saturation.
4. Dùng `mj_energyPos` và `mj_energyVel` để kiểm tra độc lập công thức cơ năng trong script.
5. So sánh Acrobot với Pendubot, nơi actuator được đặt ở khớp vai thay vì khuỷu.

## Tài liệu

- [Underactuated Robotics — Acrobots, Cart-Poles, and Quadrotors](https://underactuated.mit.edu/acrobot.html)
- [MuJoCo API — `mjd_transitionFD` và `mj_fullM`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)
