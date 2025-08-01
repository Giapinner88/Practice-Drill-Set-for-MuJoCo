# Bài 4 – Body Hierarchy

## Mục tiêu:
- Làm quen với cấu trúc phân cấp body.
- Sử dụng `joint` để mô phỏng chuyển động quay (hinge).
- Hiểu các thuộc tính: `axis`, `range`, `limited`, `damping`.

## Nội dung chính:
- Tạo một body cố định làm đế (base).
- Gắn một thanh capsule vào base qua một hinge joint.
- Cho phép thanh dao động tự do như con lắc dưới trọng lực.
- Dùng range để giới hạn góc xoay, damping để kiểm soát chuyển động.

## Thực hành:
1. Load file `model.xml` trong MuJoCo Viewer.
2. Tăng/giảm `damping` để điều chỉnh mức độ dao động.
3. Điều chỉnh range để giới hạn chuyển động con lắc.

## Gợi ý mở rộng:
1. Thêm actuator để quay hinge joint