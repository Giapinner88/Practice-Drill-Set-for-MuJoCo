# P1-08 — Light sources

## Sau bài này, bạn có thể

Phân biệt directional, point và spot light; điều chỉnh diffuse, specular và shadow mà không thay đổi dynamics.

## Các nguồn sáng trong model

- `sun`: directional light; `dir` xác định hướng tia.
- `red_bulb`: positional light với attenuation.
- `blue_spot`: nguồn có hướng/cutoff để quan sát vùng chiếu.
- `ambient`: positional fill light cường độ thấp trong scene demo.

Các material `shiny` và `matte` cho phản xạ specular khác nhau. Ba vùng dùng các vật thể có hình dạng gần giống nhau để bạn dễ so sánh ánh sáng.

Lighting không thay đổi state dynamics. Nếu mọi actuator/state/physics parameter giữ nguyên, thay đổi light không được làm thay đổi `qpos` hoặc contact.

## Chạy thí nghiệm

```bash
python part_1_foundations/08_lights/simulate.py
```

## Bài tập

1. Tắt từng light và ghi lại vùng ảnh bị ảnh hưởng.
2. Đổi `castshadow` và `shadowsize`; phân biệt chất lượng ảnh với chi phí render.
3. Chạy headless physics hai lần với cấu hình light khác nhau và so state.

## Tài liệu

- [MJCF light](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-light)
- [Visual configuration](https://mujoco.readthedocs.io/en/stable/XMLreference.html#visual)
