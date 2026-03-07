# Bài 9 – Light trong MuJoCo

## Mục tiêu
- Làm quen với cách khai báo và sử dụng ánh sáng trong môi trường mô phỏng.
- Hiểu sự khác biệt giữa các loại ánh sáng và ảnh hưởng của chúng lên cảnh vật.
- Học cách dùng thuộc tính `castshadow`, `dir`, `pos`, `directional`.

## Nội dung
- Một vật thể hình hộp có thể di chuyển theo trục Y.
- 3 đèn chiếu sáng:
  - `spot_light`: chiếu từ trên xuống (có bóng).
  - `side_light`: chiếu chéo từ góc bên (không đổ bóng).
  - `body_light`: gắn theo body di chuyển (góc chiếu động).

## Hướng dẫn thực hành
1. Quan sát sự thay đổi ánh sáng khi chuyển body.
2. So sánh bóng đổ khi bật/tắt `castshadow`.
3. Di chuyển camera để thấy ảnh hưởng từ từng đèn.
4. Tắt spot light để thấy side light rõ hơn

## Mở rộng
- Dùng `actuator` để thay đổi góc chiếu sáng theo thời gian.
- Tăng/giảm cường độ sáng bằng `cutoff`, `attenuation`, hoặc `specular`.
- Kết hợp ánh sáng động để mô phỏng đèn xe, đèn robot cầm tay.

## Gợi ý lệnh
- Dùng hotkey `Ctrl + L` trong Mujoco viewer để bật/tắt đèn.
