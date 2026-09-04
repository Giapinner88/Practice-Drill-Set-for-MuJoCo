# P1-01 — Empty world và simulation loop

## Sau bài này, bạn có thể

Biên dịch MJCF thành `mujoco.MjModel`, tạo `mujoco.MjData`, gọi `mujoco.mj_step` và giải thích vì sao model và data là hai đối tượng khác nhau.

## Kiến thức tiên quyết

- Python cơ bản và môi trường ảo.
- Chưa yêu cầu kiến thức cơ học đa vật.

## Mô hình tối thiểu

```xml
<mujoco model="empty_world">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody/>
</mujoco>
```

`<mujoco>` là phần tử gốc. `<worldbody>` là gốc cố định của cây động học. `<option>` chứa tham số của simulation; `timestep="0.002"` nghĩa là mỗi lần `mj_step` tiến simulation time thêm 2 ms.

## Model và data

```python
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
```

- `MjModel` chứa cấu trúc và tham số đã biên dịch: số body, joint, actuator, timestep, mass và geometry.
- `MjData` chứa trạng thái thay đổi: thời gian, `qpos`, `qvel`, `qacc`, `ctrl`, contact và các đại lượng trung gian.

Empty world có `model.nq == 0`, `model.nv == 0`; tuy vậy `data.time` vẫn tăng sau `mj_step`. Viewer không cần `mj_step` để “tồn tại”; gọi bước thời gian ở đây chỉ để minh họa simulation clock.

## Chạy thí nghiệm

```bash
python part_1_foundations/01_empty_world/simulate.py
python part_1_foundations/01_empty_world/simulate.py --headless --duration 0.02
```

Kết quả headless kỳ vọng: `nq=0`, `nv=0`, `nu=0`; thời gian cuối không nhỏ hơn thời lượng yêu cầu và chỉ sai khác tối đa một timestep.

## Bài tập

1. Đổi timestep thành `0.01`; đếm số lần gọi `mj_step` để đạt 1 s simulation time.
2. Bỏ `mj_step` khỏi vòng viewer và quan sát `data.time`.
3. In `model.nbody`, `model.ngeom`, `data.qpos.shape`; giải thích từng giá trị.

## Tài liệu

- [Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)
- [Simulation API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)
