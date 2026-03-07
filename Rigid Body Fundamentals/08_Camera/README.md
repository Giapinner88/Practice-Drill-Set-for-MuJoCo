# Bài 8 – Camera trong MuJoCo

## Mục tiêu
- Làm quen với nhiều chế độ camera:
  - `fixed`: giữ nguyên góc nhìn theo body.
  - `trackcom`: di chuyển theo center-of-mass (COM) của hệ body.
  - `targetbody`: luôn hướng về một body khác.
  - `xyaxes`: kiểm soát hướng nhìn bằng trực tiếp định nghĩa hệ trục camera.
- Hiểu rõ camera trong MuJoCo **luôn nhìn theo trục -Z của frame**.

## Mô tả
- Body tĩnh ở gốc tọa độ.
- Body có joint trượt di chuyển dọc trục X.
- Gắn nhiều camera với chế độ khác nhau để quan sát và so sánh.

## Các điểm kỹ thuật
- Dùng `mode="targetbody"` để camera luôn hướng tới body "moving".
- Camera `cam_xyaxes` minh họa cách chỉ định frame nhìn bằng `xyaxes`.
- Camera không thay đổi theo thời gian nếu dùng `mode="fixed"`, trừ khi body chứa nó di chuyển.

## Cách thử nghiệm
1. Chạy mô phỏng và thử chuyển giữa các camera (số 1–4).
2. Điều chỉnh vị trí `pos`, thử thay `quat`, hoặc dùng `euler` để hiểu cách camera xoay.
3. Thêm actuator điều khiển chuyển động và xem camera phản ứng ra sao.

## Mở rộng
- Gắn camera lên `end-effector` của cánh tay robot.
- Sử dụng `sensor` kiểu `camera` để xuất hình ảnh bằng Python.
- Kết hợp `targetbodycom` để quan sát toàn bộ hệ từ xa như một drone.

