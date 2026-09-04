# P2-06 — Quadrotor: thrust mixing và hover recovery

## Câu hỏi trung tâm

Bốn lực đẩy cùng phương tạo ra total thrust và ba moment điều khiển như thế nào, và làm sao phối hợp position loop với attitude loop để khôi phục trạng thái hover?

## Sau bài này, bạn có thể

- mô hình hóa rigid body bay tự do bằng `freejoint`;
- dùng site transmission để đặt lực và moment trong body frame;
- phân biệt quaternion trong `qpos` với angular velocity trong `qvel`;
- xây dựng mixer từ desired wrench sang bốn rotor thrust;
- tạo desired attitude từ desired force trong world frame;
- đánh giá hover bằng position, velocity, attitude, dwell time và actuator saturation.

Bạn nên hoàn thành P1-07, P1-09 và các bài P2-01 đến P2-03 trước bài này.

## Model và giả thiết

World frame dùng $+z$ hướng lên. Body frame của quadrotor dùng $+x$ hướng về phía trước, $+y$ sang trái và $+z$ hướng lên qua các rotor. Rotation matrix $R$ ánh xạ vector từ body frame sang world frame.

```text
mass m                    : 1.2 kg
principal inertia        : [0.025, 0.025, 0.045] kg*m^2
nq, nv, nu               : 7, 6, 4
timestep                  : 0.002 s
rotor thrust range        : [0, 6] N
rotor arm x/y coordinate : +/-0.18 m
yaw moment / thrust      : +/-0.02 m
```

`freejoint` có bảy giá trị `qpos`: position $(x,y,z)$ và quaternion $[w,x,y,z]$. Nó chỉ có sáu giá trị `qvel`: ba linear velocity và ba angular velocity. Quaternion không phải một vector gồm ba góc và không được lấy đạo hàm từng phần tử để thay cho angular velocity.

Mỗi rotor là một site gắn vào body. Actuator

```xml
<motor site="rotor_fl" gear="0 0 1 0 0 0.02" ctrlrange="0 6"/>
```

đặt force theo site $+z$ và reaction moment quanh site $+z$. Ba phần tử đầu của `gear` là force axis; ba phần tử cuối là torque axis, đều biểu diễn trong site frame.

Model cố ý bỏ qua rotor-speed state, motor electrical dynamics, propeller gyroscopic moment, aerodynamic drag và ground effect. `data.ctrl[i]` được hiểu trực tiếp là rotor thrust tính bằng newton, không phải RPM hay duty cycle. Đây là rigid-body thrust model cho bài control, không phải mô hình khí động học đầy đủ.

## Rotor order và mixer

| Rotor | Position $(x,y)$ [m] | Yaw reaction sign |
| --- | --- | --- |
| `fl` — front left | $(+0.18,+0.18)$ | $+$ |
| `fr` — front right | $(+0.18,-0.18)$ | $-$ |
| `rr` — rear right | $(-0.18,-0.18)$ | $+$ |
| `rl` — rear left | $(-0.18,+0.18)$ | $-$ |

Với $f=[f_{fl},f_{fr},f_{rr},f_{rl}]^T$, desired wrench là

$$
\begin{bmatrix}
T\\ \tau_x\\ \tau_y\\ \tau_z
\end{bmatrix}
=
\begin{bmatrix}
1&1&1&1\\
l&-l&-l&l\\
-l&-l&l&l\\
c&-c&c&-c
\end{bmatrix}f,
$$

trong đó $l=0.18\ \mathrm{m}$ và $c=0.02\ \mathrm{m}$. Controller dùng nghịch đảo ma trận này rồi chặn từng rotor trong $[0,6]\ \mathrm{N}$.

Tại hover cân bằng và attitude nằm ngang:

$$
f_i=\frac{mg}{4}=2.943\ \mathrm{N}.
$$

Keyframe `hover` lưu cả pose cân bằng và bốn control bằng `2.943 N`.

## Sensor

| Sensor | Dữ liệu |
| --- | --- |
| `position` | Position của regular body frame trong world frame, $\mathrm{m}$ |
| `orientation` | Quaternion $[w,x,y,z]$ của body frame trong world frame |
| `linear_velocity` | Linear velocity trong world frame, $\mathrm{m/s}$ |
| `angular_velocity` | Angular velocity trong IMU site frame, $\mathrm{rad/s}$ |
| `specific_force` | Accelerometer specific force trong IMU site frame, $\mathrm{m/s^2}$ |

Ba frame sensor đầu dùng `objtype="xbody"`; `body` trong nhóm frame sensor chỉ inertial frame.

## Tự điều khiển bằng slider

```bash
python part_2_models_control/06_quadrotor/simulate.py
```

Script nạp keyframe hover và không ghi đè `data.ctrl`. Nhấn `Tab`, mở **Control** rồi thay đổi từng cặp rotor:

1. tăng hoặc giảm cả bốn rotor cùng lượng để đổi altitude;
2. tăng `fl`, `rl` và giảm `fr`, `rr` để tạo roll moment dương;
3. tăng hai rotor phía sau để tạo pitch moment dương;
4. tăng `fl`, `rr` và giảm `fr`, `rl` để tạo yaw moment dương mà gần như giữ nguyên total thrust.

Một slider thay đổi riêng lẻ đồng thời làm thay đổi total thrust và nhiều moment. Mixer là cách tách bốn desired wrench component thành bốn rotor command.

## Cascaded hover controller

Controller đọc target từ `data.site_xpos[model.site("hover_target").id]`; dời site trong MJCF không cần sửa lại một vector target trong Python. Position loop tạo desired acceleration trong world frame:

$$
a_d=K_p(p_d-p)-K_vv+g e_z,
\qquad F_d=ma_d.
$$

Desired body $+z$ axis trùng hướng $F_d$. Desired yaw được giữ bằng không; hai trục còn lại được dựng thành rotation matrix $R_d$. Total thrust chiếu desired force lên body $+z$ hiện tại:

$$
T=F_d^TRe_z.
$$

Attitude error dùng rotation matrix, không trừ trực tiếp quaternion:

$$
e_R=\frac{1}{2}\operatorname{vee}\left(R_d^TR-R^TR_d\right),
$$

$$
\tau=-K_Re_R-K_\omega\omega.
$$

Attitude loop được đặt nhanh hơn position loop để quadrotor gần đạt hướng lực mong muốn trước khi outer loop thay đổi mạnh position command.

## Demo hover recovery

```bash
python part_2_models_control/06_quadrotor/demo.py
python part_2_models_control/06_quadrotor/demo.py --headless --duration 8
```

Initial condition của demo là

```text
position       : [0.25, -0.20, 0.75] m
roll/pitch/yaw : [10, -8, 15] deg
velocity       : 0
target         : [0, 0, 1] m, yaw = 0
```

Hover được xác nhận khi position error nhỏ hơn $0.05\ \mathrm{m}$, speed nhỏ hơn $0.05\ \mathrm{m/s}$ và attitude error nhỏ hơn $3^\circ$ liên tục $0.5\ \mathrm{s}$.

Với model và gain mặc định, baseline $8\ \mathrm{s}$ đạt success tại khoảng $5.228\ \mathrm{s}$. Final position error là $0.00112\ \mathrm{m}$; position-error RMSE trong $2\ \mathrm{s}$ cuối là $0.00723\ \mathrm{m}$; attitude-error RMSE cùng khoảng là $0.00267\ \mathrm{rad}$. Không rotor nào bão hòa; sai lệch lớn nhất của quaternion norm so với một là $1.2\times10^{-15}$.

Các số trên chỉ mô tả initial condition, model và gain đã công bố. Chúng không chứng minh controller ổn định toàn cục và không đại diện cho quadrotor có motor/propeller dynamics thực.

## Bài tập

1. Đặt cả bốn rotor bằng $mg/4\pm0.2\ \mathrm{N}$ trong viewer và so dấu của vertical acceleration.
2. Viết lại mixer từ phép tính $r_i\times f_i$ và kiểm tra từng cột bằng `data.qfrc_actuator`.
3. Giảm `MAX_ROTOR_THRUST_N` xuống $3.2\ \mathrm{N}$; đo saturation fraction và vùng initial condition còn recover được.
4. Đặt yaw reaction coefficient bằng không và giải thích vì sao yaw không còn controllable trong model này.
5. Thêm first-order rotor state $\dot f=(f_c-f)/\tau_m$ rồi tune lại attitude loop.
6. Thêm linear drag trong body frame và so position recovery khi initial velocity khác không.
7. Đổi hover target thành một trajectory chậm, nhưng giữ riêng metric tracking và metric hover settling.

## Tài liệu

- [MuJoCo actuator site transmission](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-general)
- [MuJoCo computation — actuation model](https://mujoco.readthedocs.io/en/latest/computation/index.html#actuation-model)
- [Underactuated Robotics — Quadrotors](https://underactuated.mit.edu/acrobot.html#quadrotors)
