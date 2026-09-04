# P1-04 — Body hierarchy và local/world frames

## Sau bài này, bạn có thể

Đọc cây body, phân biệt local pose với world pose và giải thích vì sao một body không có joint vẫn giữ frame riêng nhưng chuyển động cùng body cha.

## Cây trong bài

```text
world
└── base                 welded to world
    └── arm_link         hinge: shoulder
        └── tool         welded to arm_link
            └── site: tool_center
```

`body_pos` trong `MjModel` là vị trí local được biên dịch từ XML. `xpos` và `xquat` trong `MjData` là pose world được tính bởi forward kinematics. Thay đổi góc `shoulder` làm world pose của `arm_link`, `tool` và `tool_center` thay đổi, trong khi `model.body_pos` không đổi.

Body `base` không có joint nên weld với world. `tool` không có joint nên weld với `arm_link`; nó không thêm DOF, nhưng frame riêng vẫn hữu ích để gắn geom, site, sensor hoặc camera.

## Chạy thí nghiệm

```bash
python part_1_foundations/04_body_hierarchy/simulate.py --initial-angle-deg 45
python part_1_foundations/04_body_hierarchy/simulate.py --headless --duration 0.1 --initial-angle-deg 90
```

Script in ra parent ID, local position và world pose trước/sau khi đặt joint angle. Ở mọi góc, khoảng cách từ shoulder tới tâm tool phải bằng 0.8 m trong sai số số học.

## Phép kiểm chứng

1. Xác nhận `parent(tool) == arm_link` và `parent(arm_link) == base`.
2. Xác nhận model chỉ có một generalized coordinate và một velocity: `nq == nv == 1`.
3. Tính chuẩn Euclid giữa `xpos(tool)` và `xpos(arm_link)`; kết quả phải giữ 0.8 m khi đổi góc.

## Bài tập

1. Thêm một joint vào `tool`; dự đoán `nq`, `nv` trước khi chạy.
2. Đổi `tool pos` sang `0.2 0 -0.8`; kiểm tra khoảng cách và world trajectory.
3. Đọc pose của `tool_center` qua named access và so với body `tool`.

## Tài liệu

- [Kinematic tree](https://mujoco.readthedocs.io/en/stable/modeling.html#kinematic-tree)
- [MJCF body](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body)
