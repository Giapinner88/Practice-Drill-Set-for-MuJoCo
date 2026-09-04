# P1-07 — Frames và orientation

## Sau bài này, bạn có thể

Biểu diễn pose tương đối, đọc quaternion theo thứ tự MuJoCo và nhận biết trường hợp Euler angles gây nhầm lẫn.

## Quy ước

- `pos="x y z"` là translation trong frame cha, đơn vị m.
- `quat="w x y z"` dùng scalar-first quaternion.
- `axisangle="ax ay az angle"` dùng trục–góc; đơn vị góc theo `<compiler angle>`.
- `euler` tuân theo `eulerseq` của compiler.
- `xyaxes` hoặc `zaxis` xây orientation từ các vector định hướng.

Các biểu diễn orientation trong cùng một element là lựa chọn thay thế nhau, không được khai báo đồng thời. Hai quaternion `q` và `−q` biểu diễn cùng một rotation, vì vậy so sánh quaternion bằng khoảng cách Euclid trực tiếp có thể gây hiểu sai.

## Thí nghiệm

```bash
python part_1_foundations/07_frames_and_orientation/simulate.py
```

Model đặt các box bằng nhiều representation nhưng cùng góc quay mục tiêu. Hãy kiểm tra world orientation sau `mj_forward` qua `data.xquat`, không chỉ dựa vào hình ảnh viewer.

## Bài tập

1. Chuyển rotation 45° quanh Z thành quaternion scalar-first.
2. Đổi `compiler angle` từ degree sang radian và sửa model tương ứng.
3. Tạo body cha quay 30° rồi body con quay 45°; giải thích vì sao không thể cộng vector Euler một cách tổng quát.

## Tài liệu

- [Frame orientations](https://mujoco.readthedocs.io/en/stable/modeling.html#frame-orientations)
