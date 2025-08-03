# Bài 1 – Empty World

## Mục tiêu:
- Tạo file MJCF hợp lệ với cấu trúc tối thiểu.
- Render khung cảnh trắng trống trong MuJoCo Viewer.
- Làm quen với tag `<mujoco>`, `<option>`, `<visual>`, và `<worldbody>`.

## Nội dung chính:
File XML không có vật thể nào, nhưng vẫn có gravity và cài đặt camera để Viewer không bị crash.

## Thực hành:
1. Mở file `model.xml` trong MuJoCo Viewer (hoặc dùng `mujoco.mj_render()` nếu chạy qua Python).
2. Quan sát: viewer sẽ hiển thị một khung cảnh hoàn toàn trống (hoặc nền xám).
3. Thay đổi giá trị `gravity` hoặc `znear/zfar` để xem sự khác biệt khi render.

## Gợi ý mở rộng:
- Thêm một camera và light (để học bài 16–17).
- Thêm `<compiler>` hoặc `<size>` để hiểu cách bộ biên dịch MJCF xử lý mô hình.

