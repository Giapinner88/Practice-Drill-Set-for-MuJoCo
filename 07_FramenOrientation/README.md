# Bài 7 – Frame và Orientation trong MuJoCo

## Mục tiêu
- Hiểu frame toàn cục và local trong MuJoCo.
- Sử dụng thuộc tính `pos` để đặt vị trí tương đối.
- Sử dụng `quat` (quaternion) và `axisangle` để đặt hướng xoay.
- Quan sát ảnh hưởng của các hệ tọa độ lên mô hình.

## Nội dung
Tạo 1 `body` gốc ở vị trí (0,0,0) với `geom` hình cầu làm tham chiếu.

Tạo 2 `body` con đặt lệch vị trí:
- `child_body_quat` dùng quaternion xoay ~45° quanh trục Z.
- `child_body_axisangle` dùng axisangle xoay 90° quanh trục Z.

Gán geom hộp để dễ quan sát hướng xoay.

## Hướng dẫn chạy
1. Mở model trong MuJoCo viewer.
2. Quan sát vị trí, hướng của các body con so với gốc.
3. Thay đổi `pos`, `quat`, `axisangle` để hiểu rõ hơn.

## Mở rộng
- Thử dùng `euler` angles (bạn cần convert sang `quat` hoặc `axisangle`).
- Tạo thêm các body con đa cấp để hiểu chuỗi biến đổi frame.
- Kết hợp với sensors hoặc actuator dựa trên frame local.

