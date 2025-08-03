# Bài 3 – Gravity & Ground

## Mục tiêu:
- Làm quen với tác động của trọng lực đến vật thể động.
- Tạo mặt đất tĩnh bằng geom kiểu "plane".
- Cho một khối box rơi tự do và tương tác với mặt đất.

## Nội dung chính:
- `<option gravity="0 0 -9.81"/>`: trọng lực 9.81 m/s² theo trục Z âm.
- Mặt đất được định nghĩa bằng `<geom type="plane">`.
- Một body có geom dạng box được đặt ở cao độ Z = 2 m.

## Thực hành:
1. Load file `model.xml` trong MuJoCo Viewer.
2. Quan sát box rơi xuống và va chạm với mặt đất.
3. Thay đổi trọng lực (ví dụ 0 0 -3 hoặc 0 0 -20), chạy lại và so sánh kết quả.
4. Thay đổi `timestep` hoặc thêm `friction="1"` vào geom để thấy khác biệt.

## Gợi ý mở rộng:
- Tạo nhiều vật thể rơi từ các vị trí khác nhau.
- Thêm `<sensor type="touch">` để kiểm tra thời điểm box chạm đất.
- Dùng camera để theo dõi box trong quá trình rơi (xem bài 16).
