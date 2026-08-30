# P1-08 — Cameras và moving viewpoint

## Chuẩn đầu ra

Người học cấu hình camera cố định, camera gắn trên body và camera theo dõi target; đồng thời dùng actuator/sensor tối thiểu để tạo cảnh chuyển động kiểm chứng camera.

## Camera frame

Camera MuJoCo nhìn theo trục local −Z; local +Y là hướng lên của ảnh. `pos` và orientation được biểu diễn trong frame của body chứa camera.

Model có:

- `fixed_cam`: camera cố định trong world;
- `tracking_cam`: camera mode `targetbody` hướng vào `moving_box`;
- `robot_cam_forward` và `robot_cam_up`: camera gắn trên body chuyển động;
- một slide joint, motor và hai sensor để tạo trajectory quan sát được.

`fovy` là vertical field of view theo degree. Camera trong viewer và off-screen `mujoco.Renderer` là hai cách tiêu thụ cùng camera model; bài này mới tập trung vào viewer.

## Chạy thí nghiệm

```bash
python part_1_foundations/08_cameras/simulate.py
```

Chọn lần lượt bốn camera trong viewer. Script áp tín hiệu sin lên actuator; kiểm tra `data.sensordata` thay đổi cùng chuyển động của box.

## Bài tập

1. Đo biên độ `qpos` và giải thích ảnh hưởng của `gear` và `ctrlrange`.
2. Chuyển camera cố định sang body-local camera và so pose.
3. Thêm off-screen rendering 320×240 rồi lưu đúng một frame vào `artifacts/`.

## Tài liệu

- [MJCF camera](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera)
- [Rendering](https://mujoco.readthedocs.io/en/stable/python.html#rendering)
