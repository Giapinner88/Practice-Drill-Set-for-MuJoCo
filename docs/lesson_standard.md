# Chuẩn một bài học

## Cấu trúc tối thiểu

```text
lesson_name/
├── README.md
├── model.xml
├── simulate.py
└── tests/                 # thêm khi bài có kiểm chứng riêng
```

Các model phụ phải có tên mô tả mục đích, ví dụ `file_textures.xml`; tránh `model1.xml`, `model2.xml`.

## Cấu trúc README

Mỗi README phải trả lời được các câu hỏi sau:

1. Người học sẽ làm được gì sau bài này?
2. Cần biết gì trước khi bắt đầu?
3. Tag/API nào được giới thiệu lần đầu?
4. Giả thiết vật lý, hệ tọa độ và đơn vị là gì?
5. Chạy lệnh nào và quan sát đại lượng nào?
6. Kết quả nào được kỳ vọng, với dung sai nào nếu có?
7. Thí nghiệm nào có thể bác bỏ một diễn giải sai?
8. Bài tập nào buộc người học sửa model hoặc code?
9. Nguồn chính thống nào hỗ trợ phần lý thuyết/API?

## Quy ước cho script

- Dùng `Path(__file__)` để tìm asset/model.
- Có `main()` và guard `if __name__ == "__main__"`.
- Có `--duration` tùy chọn để giới hạn simulation time.
- Viewer mặc định chạy đến khi người dùng đóng cửa sổ/nhấn ESC.
- Hỗ trợ `--headless` nếu bài không phụ thuộc GUI; headless phải có thời lượng mặc định hữu hạn.
- Tách simulation time khỏi wall-clock time.
- Không ghi artifact ra root của repo.
- Nếu tạo artifact, ghi vào `lesson/artifacts/` và in đường dẫn.
- Không dùng index số “bí mật” khi có thể truy cập theo tên.

## Chuẩn kiểm chứng

- XML well-formed chỉ là kiểm tra cú pháp, chưa phải model hợp lệ.
- Model hợp lệ khi `MjModel.from_xml_path` biên dịch thành công.
- Smoke test phải tạo `MjData`, gọi `mj_forward`, chạy một số bước và kiểm tra giá trị hữu hạn.
- Kết luận về vật lý cần so sánh với nghiệm giải tích, invariant, mốc so sánh hoặc phép đo độc lập.
- Bài control phải báo actuator limits, timestep, initial condition và metric.
- Bài RL phải báo config, seed, training steps, evaluation episodes và checkpoint provenance.

## Quy ước viết

- Tiếng Việt là ngôn ngữ giải thích chính; giữ thuật ngữ tiếng Anh lần đầu xuất hiện.
- Tên class/hàm/tag phải đúng chữ hoa-thường: `MjModel`, `MjData`, `mj_step`.
- Công thức inline dùng `$...$`; công thức tách dòng đặt giữa hai dòng dấu dollar kép. Không dùng delimiter dạng backslash–parenthesis hoặc backslash–bracket trong Markdown.
- Mọi vector phải nêu frame; mọi đại lượng vật lý phải nêu đơn vị.
- Không dùng “chính xác”, “hội tụ”, “ổn định” hoặc “sim-to-real” nếu chưa nêu điều kiện chứng minh.
