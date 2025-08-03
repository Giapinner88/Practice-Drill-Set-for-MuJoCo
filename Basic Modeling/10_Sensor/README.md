# Bài 10 – Sensor trong MuJoCo

## Mục tiêu
- Ghi nhận trạng thái và lực của hệ cơ học.
- Gắn cảm biến để theo dõi dữ liệu vật lý theo thời gian.

## Nội dung
- Một cánh tay quay quanh trục Z, có động cơ điều khiển.
- Cảm biến đo:
  - Góc và tốc độ quay (jointpos, jointvel).
  - Lực mô-men do actuator tạo ra (jointforce).
  - Gia tốc và vận tốc góc tại một site (accelerometer, gyro).

## Hướng dẫn thực hành
1. Chạy mô phỏng, điều khiển actuator (bằng slider).
2. Mở Mujoco Viewer và bật tab `Sensor` để quan sát dữ liệu cảm biến.
3. Thay đổi trọng lực hoặc thêm va chạm để quan sát phản ứng của sensor.

## Mở rộng
- Thêm cảm biến tiếp xúc (`touch`) tại các `geom` tiếp xúc.
- Kết hợp nhiều loại cảm biến để mô phỏng robot thật.
- Ghi log dữ liệu cảm biến và dùng Python xử lý.

## Gợi ý debug
- Mỗi cảm biến chỉ hoạt động nếu gắn đúng `joint`, `body`, hoặc `site`.
- Kiểm tra `range`, `timestep`, và `ctrlrange` để giữ dữ liệu hợp lý.
