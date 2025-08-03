# Bài 5 – Mesh Loading trong MuJoCo

## Mục tiêu
- Hiểu cách sử dụng thẻ `<mesh>` để load file mesh 3D (.stl, .obj).
- Áp dụng mesh cho `<geom>` để mô phỏng vật thể phức tạp.
- Điều chỉnh scale, vị trí mesh trong mô hình.

## Nội dung
Bài tập tạo một vật thể mesh (con thỏ Stanford) được tải từ file `assets/bunny.stl`. Mesh được gán cho `<geom type="mesh">` trong thân `bunny_body`.

Ngoài ra, có mặt đất bằng `<geom type="plane">` để bạn dễ quan sát mesh khi sim.

## Gợi ý mở rộng
- Thử đổi mesh file khác, hoặc scale khác.
- Thêm actuator để điều khiển vật thể mesh.
- Kết hợp sensor để phát hiện va chạm với mesh.
