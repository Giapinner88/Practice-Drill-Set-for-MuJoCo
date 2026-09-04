# P1-10 — Inertial properties

## Sau bài này, bạn có thể

Mô tả 10 inertial parameters của rigid body, phân biệt inertia suy diễn với inertia khai báo tường minh và kiểm tra tensor quán tính trước khi đưa vào model.

## Lý thuyết

Một rigid body có:

- mass $m$: 1 tham số;
- center of mass $c$: 3 tham số;
- inertia tensor đối xứng tại CoM: 6 tham số độc lập.

`diaginertia="Ixx Iyy Izz"` biểu diễn principal moments trong inertial frame. `fullinertia="Ixx Iyy Izz Ixy Ixz Iyz"` cho phép nhập tensor đầy đủ; compiler sẽ chéo hóa và kiểm tra tính hợp lệ.

Principal moments phải dương và thỏa bất đẳng thức tam giác, ví dụ $I_{xx} \le I_{yy}+I_{zz}$. Dữ liệu CAD phải được đổi về đúng kg, m, kg·m² và đúng frame trước khi nhập.

## Đối chứng trong model

- `uniform_cylinder`: compiler suy inertia từ geom và density mặc định.
- `offset_cylinder`: mass, CoM và diagonal inertia được khai báo tường minh.

Hai body có geometry giống nhau nhưng inertial parameters khác. Hãy thả chúng từ cùng độ cao, sau đó đọc `body_mass`, `body_ipos` và `body_inertia` để giải thích chuyển động.

## Chạy thí nghiệm

```bash
python part_1_foundations/10_inertial_properties/simulate.py
```

## Bài tập

1. In `model.body_mass`, `body_ipos`, `body_inertia` cho hai body.
2. Tính inertia giải tích của cylinder đồng chất rồi so compiler output.
3. Nhập một tensor vi phạm bất đẳng thức tam giác và đọc lỗi compiler.
4. Chuyển một tensor CAD có product of inertia khác zero sang `fullinertia` với đơn vị SI.

## Tài liệu

- [Inertial properties](https://mujoco.readthedocs.io/en/stable/modeling.html#inertial-properties)
- [MJCF inertial](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-inertial)
