# P1-02 — Static geoms

## Sau bài này, bạn có thể

Phân biệt `body` và `geom`, sử dụng các primitive geometry và đọc đúng thuộc tính `size` của từng loại.

## Ý tưởng chính

`geom` mô tả hình học dùng cho rendering, collision và suy diễn inertia. `body` là frame trong cây động học. Một `geom` đặt trực tiếp trong `<worldbody>` được gắn cứng vào world: trọng lực không làm nó rơi.

Model minh họa plane, box, capsule, sphere, cylinder và ellipsoid. Thuộc tính `size` không phải lúc nào cũng là kích thước đầy đủ:

- box: ba half-size;
- sphere: bán kính;
- cylinder: bán kính và half-length;
- capsule: bán kính và half-length nếu không dùng `fromto`;
- plane: hai kích thước phục vụ rendering, còn collision plane là vô hạn.

## Tags và thuộc tính

| Thành phần | Vai trò |
| --- | --- |
| `<geom>` | khai báo geometry |
| `type` | loại primitive |
| `size` | tham số kích thước theo từng loại |
| `pos` | vị trí trong frame cha, đơn vị m |
| `rgba` | màu và alpha trong khoảng 0–1 |

## Chạy thí nghiệm

```bash
python part_1_foundations/02_static_geoms/simulate.py
```

Kết quả kỳ vọng: tất cả vật thể giữ nguyên pose vì đều thuộc worldbody, dù gravity khác zero.

## Bài tập

1. Tính kích thước đầy đủ của box từ `size` rồi kiểm tra bằng thước đo trong viewer.
2. Bọc sphere trong một `<body>` nhưng chưa thêm joint; dự đoán chuyển động.
3. Thêm `<freejoint/>` và giải thích vì sao kết quả thay đổi.

## Sai lầm thường gặp

- Hiểu `size="0.2 0.2 0.2"` là box rộng 0.2 m thay vì 0.4 m.
- Cho rằng mọi geom có mass đều tự động chuyển động. Chuyển động phụ thuộc joint của body.

## Tài liệu

- [MJCF geom reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom)
