# P1-05 — Mesh loading và collision geometry

## Sau bài này, bạn có thể

Nạp mesh từ `<asset>`, tách visual mesh khỏi collision mesh và kiểm tra scale cũng như collision filtering.

## Thiết kế model

```xml
<asset>
  <mesh name="mesh_vis" file="meshes/link1_visual.stl" scale="0.01 0.01 0.01"/>
  <mesh name="mesh_col" file="meshes/link1_collision.stl" scale="0.01 0.01 0.01"/>
</asset>
```

Visual geom dùng `contype="0" conaffinity="0"`, nên không tham gia contact. Collision geom được dùng riêng cho broad-phase và narrow-phase collision. Trong viewer, bạn có thể bật từng geom group để nhìn hai mesh chồng lên nhau.

STL không lưu đơn vị vật lý. `scale="0.01 0.01 0.01"` là một giả thiết chuyển đổi đơn vị của asset này, không phải quy tắc chung cho mọi STL.

## Chạy thí nghiệm

```bash
python part_1_foundations/05_mesh_loading/simulate.py
```

Trong viewer, bật/tắt geom group để so visual và collision representation. Kết quả mong đợi là hai mesh cùng frame và có kích thước tương thích; nếu không, cần sửa `scale` hoặc frame export từ CAD.

## Bài tập

1. Dùng cùng visual mesh cho collision và so thời gian step trên một cảnh có nhiều bản sao.
2. Đổi scale một trục và ghi nhận sai lệch geometry/inertia.
3. Thay collision mesh bằng primitive hoặc convex decomposition và nêu trade-off.

## Tài liệu

- [Model assets](https://mujoco.readthedocs.io/en/stable/modeling.html#model-assets)
- [MJCF mesh](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh)
