# P2-05 — Differential-drive Mobile Robot

## Câu hỏi trung tâm

Tốc độ hai bánh tạo ra vận tốc phẳng của robot như thế nào, và kết quả đo từ contact simulation khác mô hình động học không trượt bao nhiêu?

## Sau bài này, bạn có thể

- xây dựng mobile robot bằng `freejoint`, hai wheel hinge và một caster;
- dùng velocity actuator với giới hạn tốc độ và torque;
- đổi giữa wheel speed với linear velocity và yaw rate;
- đọc pose, quaternion, wheel speed và gyroscope từ sensor;
- điều khiển robot lần lượt đi qua các waypoint;
- tích phân wheel odometry và so sánh với pose do MuJoCo tính từ contact dynamics.

Bạn nên hoàn thành P1-03, P1-04 và các bài P2-01 đến P2-03 trước bài này.

## Model và hệ tọa độ

World frame dùng $+z$ hướng lên. Trong body frame của robot, $+x$ hướng về phía trước và $+y$ hướng sang trái. Hai wheel hinge cùng quay quanh trục body $+y$; vì vậy wheel speed dương ở cả hai bánh làm robot tiến theo $+x$.

```text
base state        : 1 freejoint, nq = 7 và nv = 6
wheel state       : 2 hinge joints, nq = nv = 2
toàn model        : nq = 9, nv = 8, nu = 2
wheel radius r    : 0.07 m
track width b     : 0.33 m
timestep          : 0.002 s
wheel ctrlrange   : [-12, 12] rad/s
wheel forcerange  : [-1.5, 1.5] N*m
```

`freejoint` lưu position và quaternion trong `qpos`, nhưng linear velocity và angular velocity chỉ cần sáu phần tử trong `qvel`. Hai bánh dùng primitive cylinder để cả visual geometry và contact geometry dễ kiểm tra. Front caster có sliding friction thấp và priority cao hơn ground; nếu bỏ priority, quy tắc trộn contact parameter sẽ lấy friction lớn của ground và caster cản chuyển động quay.

Model dùng elliptic friction cone, `impratio=10` và contact dimension `3` cho bánh. Đây là regular friction contact: có normal force và tangential friction force, nhưng không thêm torsional friction tại điểm tiếp xúc.

## Differential-drive kinematics

Giả sử hai bánh lăn không trượt, với wheel speed trái và phải lần lượt là $\dot\phi_L$ và $\dot\phi_R$:

$$
v=\frac{r}{2}\left(\dot\phi_L+\dot\phi_R\right),
$$

$$
\dot\psi=\frac{r}{b}\left(\dot\phi_R-\dot\phi_L\right).
$$

Trong đó $v$ có đơn vị $\mathrm{m/s}$ và $\dot\psi$ có đơn vị $\mathrm{rad/s}$. Phép đổi ngược dùng trong controller là

$$
\dot\phi_L=\frac{v-\frac{b}{2}\dot\psi}{r},
\qquad
\dot\phi_R=\frac{v+\frac{b}{2}\dot\psi}{r}.
$$

Đây là mô hình động học, không phải phương trình contact dynamics. `demo.py` tích phân các phương trình này thành wheel odometry rồi so với `data.xpos` và `data.xmat` của body `base`.

## Velocity actuator không đặt trực tiếp `qvel`

Mỗi `<velocity>` actuator tạo feedback force tỉ lệ với sai số giữa `data.ctrl` và joint velocity. Với `kv=0.4`, target $10\ \mathrm{rad/s}$ không đảm bảo bánh lập tức đạt đúng tốc độ đó. Torque vẫn bị chặn trong $\pm1.5\ \mathrm{N\,m}$, nên cần đọc cả wheel speed và `data.actuator_force` khi đánh giá.

Các sensor trong model gồm:

| Sensor | Dữ liệu |
| --- | --- |
| `left_wheel_speed`, `right_wheel_speed` | Tốc độ wheel hinge, $\mathrm{rad/s}$ |
| `base_position` | Position của geometric body frame trong world frame, $\mathrm{m}$ |
| `base_orientation` | Quaternion $[w,x,y,z]$ của geometric body frame trong world frame |
| `base_gyro` | Angular velocity biểu diễn trong site frame `imu`, $\mathrm{rad/s}$ |

Named sensor access trả về đúng lát cắt tương ứng trong `data.sensordata`:

```python
position = data.sensor("base_position").data
wheel_speed = data.sensor("left_wheel_speed").data[0]
```

Hai frame sensor dùng `objtype="xbody"`. Trong API MuJoCo, `xbody` chỉ regular body frame; `body` trong nhóm frame sensor chỉ inertial frame và có thể lệch khỏi frame dùng để đặt geom.

## Tự điều khiển bằng slider

```bash
python part_2_models_control/05_mobile_robot/simulate.py
```

Nhấn `Tab`, mở mục **Control** và thử:

1. đặt hai slider cùng dấu và cùng độ lớn để đi thẳng;
2. đặt hai slider trái dấu để quay gần tại chỗ;
3. đặt hai slider cùng dấu nhưng khác độ lớn để đi theo cung tròn.

`simulate.py` không ghi đè `data.ctrl`, nên giá trị slider được giữ nguyên cho đến khi bạn thay đổi.

## Demo waypoint tracking

```bash
python part_2_models_control/05_mobile_robot/demo.py
python part_2_models_control/05_mobile_robot/demo.py --headless --duration 15
```

Controller đi lần lượt qua ba waypoint $(0.8,0)$, $(0.8,0.6)$ và $(0,0.6)\ \mathrm{m}$ trong world frame. Với target hiện tại, controller tính distance $\rho$ và heading error $\alpha$, sau đó dùng

$$
v=\min(k_\rho\rho,v_{max})\max(0,\cos\alpha),
$$

$$
\dot\psi=\operatorname{clip}(k_\alpha\alpha,-\dot\psi_{max},\dot\psi_{max}).
$$

Robot phải ở trong bán kính $0.08\ \mathrm{m}$ liên tục $0.15\ \mathrm{s}$ trước khi chuyển waypoint. Giới hạn dùng trong controller là $v_{max}=0.40\ \mathrm{m/s}$ và $|\dot\psi|\leq1.8\ \mathrm{rad/s}$.

Với model và gain mặc định, lần chạy tham chiếu dài $15\ \mathrm{s}$ hoàn thành waypoint cuối tại khoảng $8.404\ \mathrm{s}$. Final goal error là $0.0664\ \mathrm{m}$, wheel-odometry position error là $0.2146\ \mathrm{m}$ và longitudinal speed discrepancy RMSE là $0.0242\ \mathrm{m/s}$. Wheel command không bão hòa; actuator force chạm giới hạn trong khoảng $0.093\%$ số bước.

Sai khác odometry không phải một phép đo độc lập của riêng wheel slip. Nó còn chứa wheel-speed tracking transient, contact softness, caster motion và sai số tích phân. Biểu đồ chỉ cho phép kết luận rằng giả thiết no-slip không tái tạo đúng toàn bộ trajectory của model này.

## Kiểm tra model trước khi tin controller

- Với control bằng không, base chỉ lệch khoảng $0.13\ \mathrm{mm}$ sau $2\ \mathrm{s}$ trong baseline hiện tại.
- Hai wheel command bằng nhau phải tạo chuyển động chủ yếu theo body $+x$.
- Đổi dấu hiệu giữa bánh trái và phải phải đổi chiều yaw.
- `data.ctrl` phải nằm trong $\pm12\ \mathrm{rad/s}$ và actuator force trong $\pm1.5\ \mathrm{N\,m}$.
- Một trajectory chạm đủ waypoint không chứng minh mô hình no-slip; phải xem thêm odometry discrepancy và contact configuration.

## Bài tập

1. Đặt hai wheel target lần lượt thành $(6,6)$, $(-6,6)$ và $(3,6)\ \mathrm{rad/s}$; dự đoán dấu của $v$ và $\dot\psi$ trước khi chạy.
2. Bỏ `priority="1"` khỏi caster rồi đo lại success time và odometry error.
3. Đổi wheel contact từ `condim=3` sang `condim=4`; giải thích ảnh hưởng của torsional friction lên chuyển động quay.
4. Giảm torque limit còn $0.5\ \mathrm{N\,m}$ và đo wheel-speed tracking error cùng thời gian hoàn thành.
5. Thay controller dùng pose tích phân từ wheel odometry thay vì ground-truth body pose; kiểm tra waypoint nào bị miss.
6. Thêm noise vào wheel speed trước khi tích phân odometry, chạy nhiều seed và báo phân bố final position error.

## Tài liệu

- [Differential-drive motion model — Introduction to Robotics and Perception](https://www.roboticsbook.org/S52_diffdrive_actions.html)
- [MuJoCo XML reference — joint, actuator và sensor](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo modeling — contact parameters và preventing slip](https://mujoco.readthedocs.io/en/stable/modeling.html)
