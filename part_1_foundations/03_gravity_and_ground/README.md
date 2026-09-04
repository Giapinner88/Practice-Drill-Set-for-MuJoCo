# P1-03 — Gravity và ground contact

## Sau bài này, bạn có thể

Tạo vật thể động bằng `freejoint`, cấu hình gravity và timestep, sau đó đo chuyển động rơi trước khi vật thể chạm đất.

## Mô hình vật lý

Trong pha rơi tự do, bỏ qua cản không khí:

$$
z(t)=z_0+v_{z,0}t-\frac{1}{2}gt^2.
$$

Quan hệ này chỉ áp dụng trước lần tiếp xúc đầu tiên. Sau tiếp xúc, gia tốc phụ thuộc contact solver, restitution hiệu dụng và timestep.

```xml
<option gravity="0 0 -9.81" timestep="0.002"/>
<geom name="ground" type="plane" .../>
<body name="falling_box" pos="0 0 2">
  <freejoint/>
  <geom type="box" .../>
</body>
```

`freejoint` cấp 7 phần tử `qpos` (3 vị trí + quaternion) và 6 phần tử `qvel` (3 vận tốc tịnh tiến + 3 vận tốc góc). Vì vậy `nq` không nhất thiết bằng `nv`.

## Chạy thí nghiệm

```bash
python part_1_foundations/03_gravity_and_ground/simulate.py
python part_1_foundations/03_gravity_and_ground/simulate.py --headless --duration 1
```

Script tự ghi vị trí tại 0.1 s và so với nghiệm ballistic trước contact. Với timestep 0.002 s, sai số phải ở cỡ milimet hoặc nhỏ hơn và số contact tại mốc kiểm tra phải bằng zero. Sau contact, không dùng công thức rơi tự do để diễn giải trajectory.

## Bài tập

1. Ghi `z(t)` trước contact và so với nghiệm giải tích.
2. Lặp với timestep 0.001, 0.002 và 0.01; báo sai số cực đại.
3. Đặt gravity về zero và kiểm tra vận tốc có thay đổi hay không.

## Tài liệu

- [Computation and simulation pipeline](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [MJCF option](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option)
