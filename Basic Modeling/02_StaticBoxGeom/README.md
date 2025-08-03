# Bài 2 – Static Box Geom

## Mục tiêu:
- Thêm các `<geom>` vào thế giới để tạo vật thể tĩnh.
- Hiểu ý nghĩa các thuộc tính: `type`, `size`, `pos`, `rgba`.
- Quan sát vật thể box và mặt phẳng ground trong MuJoCo Viewer.

## Nội dung chính:
- Mặt phẳng (plane) dùng làm mặt đất.
- Box nhỏ đặt trên mặt đất với vị trí (0,0,0.2).
- Box có màu đỏ, kích thước 0.4m x 0.4m x 0.4m (2*size vì size là bán kính).

## Thực hành:
1. Mở `model.xml` trong MuJoCo Viewer.
2. Quan sát box và mặt phẳng ground.
3. Thay đổi màu sắc hoặc kích thước box, chạy lại để thấy thay đổi.
4. Thêm các geom khác như `sphere`, `capsule`, thử nghiệm.

## Gợi ý mở rộng:
- Tìm hiểu khác biệt giữa `<geom>` nằm trong `<worldbody>` và trong `<body>`.
- Thêm `<body>` có `<geom>` để xem vật thể động.

