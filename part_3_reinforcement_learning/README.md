# Phần 3 — Reinforcement Learning với `mjlab`

Phần này đang ở giai đoạn thiết kế. Chưa có kết quả training nào được xem là baseline chính thức.

## Điều kiện bắt đầu

Trước khi triển khai, cần chốt:

- repository/package `mjlab` chính xác và phiên bản commit;
- Python, MuJoCo và accelerator backend tương thích;
- cấu trúc config, checkpoint và artifact;
- protocol đánh giá nhiều seed;
- một controller cổ điển ở Phần 2 làm baseline khi phù hợp.

## Tiến trình dự kiến

1. Chuyển một model đã kiểm chứng thành task.
2. Định nghĩa observation, action, reward, reset và termination bằng đơn vị rõ ràng.
3. Viết kiểm thử reset/step trước khi training.
4. Chạy baseline nhỏ có seed cố định.
5. Đánh giá checkpoint trên seed tách biệt.
6. Chỉ sau đó mới thêm domain randomization hoặc curriculum.

Xem [roadmap](../roadmap.md) và [lesson standard](../docs/lesson_standard.md).
