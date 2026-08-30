# P1-04 — Body hierarchy và free joint

## Chuẩn đầu ra

Người học đọc được cây body, phân biệt world pose với local pose và giải thích ánh xạ giữa joint, generalized coordinates và degrees of freedom.

## Cây động học

MuJoCo tổ chức body thành cây có gốc là world. `pos` và orientation của một body được khai báo tương đối với frame cha. Một body không có joint được weld vào cha; một body có `freejoint` chuyển động 6 DOF so với world.

Trong model:

- `falling_box` có local position ban đầu `(0, 0, 2)` m;
- `root_joint` là free joint;
- `box_geometry` nằm tại gốc frame của body;
- inertia được compiler suy ra từ geometry vì body không khai báo `<inertial>`.

Suy diễn inertia từ geometry phù hợp cho bài nhập môn, nhưng không đủ để đại diện robot thật có motor, vỏ rỗng và phân bố khối lượng không đồng đều.

## Python API cần quan sát

```python
body_id = model.body("falling_box").id
print(data.xpos[body_id])
print(model.nq, model.nv)
```

`data.xpos` chỉ hợp lệ sau `mj_forward` hoặc một bước simulation. Không đồng nhất `data.xpos` với đoạn tương ứng của `qpos`: chúng biểu diễn các tập tọa độ khác nhau.

## Chạy và bài tập

```bash
python part_1_foundations/04_body_hierarchy/simulate.py
```

1. In `qpos` và `xpos` tại thời điểm 0 rồi giải thích kích thước.
2. Bỏ `freejoint`; dự đoán `nq`, `nv` và chuyển động.
3. Thêm body con lệch 0.5 m; so sánh local pose trong XML với world pose trong `xpos`.

## Tài liệu

- [Modeling concepts](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [MJCF body and joint](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body)
