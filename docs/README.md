# Tài liệu dành cho người biên soạn

Thư mục này chứa tài liệu quản lý và quy ước viết bài. Nội dung hướng dẫn học tập phải đặt trong README của từng bài, không đặt quy tắc biên soạn vào README cấp repo.

## Tài liệu

- [lesson_standard.md](lesson_standard.md): cấu trúc và cách viết một bài.
- [coverage.md](coverage.md): các nhóm tag/API đã có hoặc còn thiếu.
- [roadmap.md](roadmap.md): thứ tự phát triển nội dung.

## Nguyên tắc học thuật

- Mọi đại lượng phải ghi đơn vị và quy ước hệ tọa độ.
- Phân biệt giả thiết mô hình, kết quả giải tích và kết quả đo từ simulation.
- Một lệnh chạy thành công không tự chứng minh mô hình đúng.
- Mỗi kết luận định lượng cần phép đo, baseline hoặc điều kiện kiểm chứng.
- Bài điều khiển phải nêu giới hạn actuator, timestep và trạng thái cân bằng.
- Kết quả RL phải ghi seed, số episode, checkpoint và tiêu chí đánh giá.

## Kiểm tra trước khi commit

```bash
python tools/validate_repo.py
python -m pytest -q
git diff --check
```
