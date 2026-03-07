# Bài 16 – Simple Pendulum

## 🎯 Mục tiêu
- Hiểu cấu trúc vật lý của con lắc đơn.
- Dùng actuator để tác động moment lên con lắc.
- Viết điều khiển PD để giữ con lắc tại vị trí thẳng đứng (unstable equilibrium).
- Tính toán năng lượng hệ thống.

## 🧠 Ghi chú kỹ thuật

### Mô hình vật lý
- Trục quay là hinge ở gốc treo.
- Trọng lực kéo con lắc dao động.

### Dữ liệu cần đo
- `jointpos` để biết góc hiện tại.
- `jointvel` để tính vận tốc góc.
- Có thể tính:
  - Động năng: `T = 0.5 * I * ω^2`
  - Thế năng: `U = m * g * h`
  - Tổng cơ năng để kiểm tra bảo toàn năng lượng.

### Điều khiển PD
```python
# Giả sử đã có θ (joint pos) và ω (joint vel)
u = -Kp * (θ - θ_desired) - Kd * ω
