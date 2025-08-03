# Bài 12 – Tendon & Soft Constraint trong MuJoCo

## 🎯 Mục tiêu
- Làm quen với tendon loại `spatial`, tạo liên kết mềm giữa các điểm trong mô hình.
- Kiểm soát chiều dài, độ cứng, và lực đàn hồi của dây mô phỏng.
- Gắn tendon vào các site động trong mô hình.

## 🧠 Kiến thức chính
- `<tendon>` dùng để mô phỏng dây, gân, lực ràng buộc mềm.
- `<spatial>` liên kết các site bất kỳ trong mô hình.
- `springlength`, `stiffness`, và `damping` xác định đặc tính cơ học.

## 💡 Gợi ý mở rộng
- Gắn tendon vào các `geom` hoặc `site` chuyển động.
- Quan sát lực căng sinh ra bằng sensor hoặc render.
- Thử dùng `fixed` tendon hoặc `muscle` để mô phỏng co rút chủ động.

## 🛠 Cách chạy
1. Mở `model.xml` trong viewer MuJoCo hoặc dùng Python để load.
2. Kéo khớp tay để quan sát dây bị kéo căng và phục hồi theo lực đàn hồi.
3. Thay đổi `springlength` hoặc `stiffness` để cảm nhận ảnh hưởng.

---
