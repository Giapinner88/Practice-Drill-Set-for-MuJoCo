# Bài 11 – Constraint và Contact trong MuJoCo

## Mục tiêu
- Mô phỏng va chạm, tiếp xúc vật lý giữa vật thể.
- Áp dụng ràng buộc chuyển động như giữ khoảng cách, cố định góc…

## Nội dung
- Một hộp rơi tự do xuống mặt đất, tạo contact force.
- Một tay gắp quay với joint, gắn với điểm neo bằng ràng buộc `connect`.
- Dùng sensor đo lực phản ứng tại khớp và điểm tiếp xúc.

## Hướng dẫn thực hành
1. Quan sát hộp rơi – kiểm tra va chạm giữa `geom`.
2. Kiểm tra `Contact` tab trong MuJoCo để xem danh sách va chạm đang xảy ra.
3. Quan sát lực tại joint – là kết quả phản lực từ ràng buộc.
4. Sửa độ ma sát (`friction`) hoặc trọng lực để thấy rõ hiệu ứng.

## Loại ràng buộc trong MuJoCo
| Loại         | Thẻ         | Mô tả                         |
|--------------|-------------|-------------------------------|
| `connect`    | `<connect>` | Giữ khoảng cách hai site      |
| `weld`       | `<weld>`    | Dán hai body (pose không đổi) |
| `joint`      | `<joint>`   | Áp ràng buộc chuyển động      |
| `tendon`     | `<tendon>`  | Ràng buộc mềm kiểu cáp/kéo    |

## Gợi ý mở rộng
- Thử thay `connect` thành `weld` để hiểu ràng buộc cứng khác gì ràng buộc mềm.
- Sử dụng Python API để trích xuất thông tin tiếp xúc (contact data).
- Tạo mô hình 2 vật nối bằng `tendon` và đo lực dây kéo.

## Debug nhanh
- Ràng buộc chỉ hoạt động nếu các `site` được định nghĩa hợp lệ.
- Dễ gây ra lỗi mô phỏng nếu body quá nhẹ, timestep quá nhỏ.
