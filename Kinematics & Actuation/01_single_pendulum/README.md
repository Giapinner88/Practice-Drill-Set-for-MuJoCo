# Bài 11: Simple Pendulum (Khớp bản lề và Con lắc đơn)

## 1. Cơ sở lý thuyết vật lý
Hệ con lắc là ví dụ kinh điển nhất về một hệ thống phi tuyến và thiếu cơ cấu chấp hành (underactuated system) khi momen xoắn điều khiển bị giới hạn. 

Phương trình chuyển động (Equation of Motion) của con lắc toán học (giả định toàn bộ khối lượng $m$ tập trung tại một điểm cách trục quay khoảng $l$) được thiết lập qua phương trình Euler-Lagrange:
$$ml^2 \ddot{\theta} + b\dot{\theta} + mgl \sin(\theta) = \tau$$

Trong đó:
* $\theta, \dot{\theta}, \ddot{\theta}$: Lần lượt là góc quay, vận tốc góc và gia tốc góc.
* $b$: Hệ số cản nhớt (Viscous damping) tại khớp.
* $\tau$: Momen xoắn chủ động từ cơ cấu chấp hành.

Hệ thống được biểu diễn hoàn chỉnh qua vector trạng thái $\mathbf{x} = [\theta, \dot{\theta}]^T$. Động lực học hệ thống chi phối hai điểm cân bằng (Equilibria):
* **Cân bằng bền (Stable Equilibrium):** $\mathbf{x}^* = [0, 0]^T$ (Vị trí con lắc đứng thẳng xuống).
* **Cân bằng không bền (Unstable Equilibrium):** $\mathbf{x}^* = [\pi, 0]^T$ (Vị trí con lắc lộn ngược - Mục tiêu của bài toán Swing-up).

## 2. Phân tích thẻ sâu trong MuJoCo
* **Thẻ `<inertial>`:** Việc khai báo thẻ này sẽ ghi đè hoàn toàn thuật toán tự động tính khối lượng từ hình học (Geom) của MuJoCo. Thuộc tính `pos="0 0 -1"` và `mass="1"` biến cơ cấu này thành một mô hình toán học điểm (Point-mass pendulum). `diaginertia="1e-6 1e-6 1e-6"` được gán giá trị epsilon để tránh suy biến ma trận nghịch đảo.
* **Quy tắc bàn tay phải & Quaternion:** Camera và các góc nhìn luôn sử dụng hệ quaternion (VD: `quat="0.7071 0.7071 0 0"` thay vì Euler) để tránh rủi ro Gimbal Lock trong tính toán ma trận xoay. Trục dao động mặc định nằm trên mặt phẳng XZ (quanh trục Y).
* **Actuator & Cảm biến:** Thẻ `<motor>` được ánh xạ trực tiếp thành $\tau$, trong khi `<sensor>` trích xuất chính xác vector trạng thái $\mathbf{x}$.

## 3. Ghi chú Sim-to-Real
* **Underactuated Bound:** Thẻ motor được cấu hình `ctrlrange="-3.0 3.0"`. Trọng lực yêu cầu $9.81$ Nm để giữ con lắc ở phương ngang, do đó lực $\pm3.0$ Nm là không đủ để nhấc bổng cơ cấu. Bộ điều khiển phải bơm năng lượng (energy pumping) qua nhiều chu kỳ dao động.
* **Nhiễu Cảm biến & Độ trễ:** Dữ liệu trích xuất từ sensor hiện tại là Ground Truth. Ở môi trường thực tế, cần mô phỏng thêm Gaussian noise và delay của chuẩn giao tiếp động cơ.

## 4. Lý thuyết Điều khiển Cục bộ (Local Control)
Trong phần này, chúng ta giải quyết bài toán ổn định con lắc tại vị trí cân bằng bền $\mathbf{x}^* = [0, 0]^T$ (hoặc một góc nghiêng nhỏ $\theta_{ref}$). 

Khi con lắc dao động quanh vị trí $\theta \approx 0$, ta có thể xấp xỉ tuyến tính $\sin(\theta) \approx \theta$. Phương trình động lực học trở thành hệ tuyến tính bậc 2:
$$ml^2 \ddot{\theta} + b\dot{\theta} + mgl\theta = \tau$$

Để điều khiển vị trí và triệt tiêu dao động, chúng ta áp dụng luật điều khiển Tỷ lệ - Vi phân (PD Controller):
$$\tau = -k_p (\theta - \theta_{ref}) - k_d \dot{\theta}$$

Trong đó:
* Khâu P (Proportional - $k_p$): Tạo lực kéo con lắc về vị trí đích $\theta_{ref}$. Đóng vai trò như một "lò xo ảo" (virtual spring).
* Khâu D (Derivative - $k_d$): Triệt tiêu động năng dựa trên vận tốc $\dot{\theta}$. Đóng vai trò như một "bộ giảm chấn ảo" (virtual damper / damping injection).

## 5. Lưu ý Sim-to-Real: Đạo hàm vận tốc và Nhiễu
Trong MuJoCo, cảm biến `<jointvel>` cung cấp giá trị $\dot{\theta}$ hoàn hảo trực tiếp từ bộ giải vật lý. Tuy nhiên, trên hệ thống thực tế, vận tốc thường được tính bằng đạo hàm số của vị trí: $\dot{\theta} \approx \frac{\theta_t - \theta_{t-1}}{\Delta t}$. Quá trình này sẽ khuếch đại nhiễu cảm biến (sensor noise), khiến khâu D ($k_d \dot{\theta}$) sinh ra tín hiệu điều khiển $\tau$ chấn động mạnh (chattering), gây hại cho động cơ.

## Tài liệu tham khảo
1. https://www.myphysicslab.com/pendulum/pendulum-en.html
2. https://underactuated.csail.mit.edu/pend.html 