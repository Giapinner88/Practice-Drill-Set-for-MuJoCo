# P1-09 — Cameras và dữ liệu ảnh

## Sau bài này, bạn có thể

Cấu hình camera cố định, camera gắn trên body và camera theo dõi target; đọc pose của camera trong world frame; lấy ảnh RGB và depth dưới dạng mảng NumPy; render một trajectory thành video.

## Camera frame

Camera MuJoCo nhìn theo trục local −Z; local +Y là hướng lên của ảnh. `pos` và orientation được biểu diễn trong frame của body chứa camera.

Model có:

- `fixed_cam`: camera cố định trong world;
- `tracking_cam`: camera mode `targetbody` hướng vào `moving_box`;
- `robot_cam_forward` và `robot_cam_up`: camera gắn trên body chuyển động;
- một slide joint, position actuator và hai sensor để tạo trajectory quan sát được.

`fovy` là vertical field of view theo degree. Camera trong viewer và off-screen `mujoco.Renderer` sử dụng cùng camera được khai báo trong MJCF.

## Chạy thí nghiệm

```bash
python part_1_foundations/09_cameras/simulate.py
```

Chọn lần lượt bốn camera trong viewer. Script đặt target position dạng sin, biên độ 1.25 m và tần số 0.1 Hz. Actuator có `kp=80`; joint có damping và range ±2 m. Script báo tracking RMSE và vị trí cực đại để phân biệt target, actuator dynamics và joint limits.

## Đọc dữ liệu từ camera

Chạy ví dụ off-screen:

```bash
python part_1_foundations/09_cameras/capture.py
```

Ba lệnh tạo một ảnh camera là:

```python
renderer = mujoco.Renderer(model, height=240, width=320)
renderer.update_scene(data, camera="fixed_cam")
rgb = renderer.render()
```

`update_scene` lấy trạng thái hiện tại từ `data` và chọn camera. `render` trả về ảnh RGB có shape `(240, 320, 3)` và dtype `uint8`. Pixel tại hàng `y`, cột `x` được đọc bằng `rgb[y, x]`.

Để đọc depth từ cùng viewpoint:

```python
renderer.enable_depth_rendering()
renderer.update_scene(data, camera="fixed_cam")
depth = renderer.render()
```

Depth có shape `(240, 320)`, dtype `float32`; mỗi phần tử là khoảng cách theo trục nhìn, đơn vị mét. `capture.py` in giá trị RGB và depth tại pixel trung tâm, đồng thời lưu hai ảnh minh họa trong `artifacts/`.

Pose camera sau forward kinematics nằm trong:

```python
camera_id = model.camera("fixed_cam").id
position = data.cam_xpos[camera_id]
rotation = data.cam_xmat[camera_id].reshape(3, 3)
```

Nếu bạn sửa trực tiếp `data.qpos`, hãy gọi `mujoco.mj_forward(model, data)` trước `update_scene`. Sau `mj_step`, pose và geometry trong `data` đã được cập nhật cho state mới.

## Render video

`render_video.py` chạy cùng chuyển động hình sin, lấy ảnh RGB từ `fixed_cam` ở 30 FPS và tạo video MP4 dài 5 giây:

```bash
python part_1_foundations/09_cameras/render_video.py
```

Physics vẫn chạy với timestep $0.002\ \mathrm{s}$, còn camera chỉ chụp khi simulation time đi qua mốc frame tiếp theo:

```python
if data.time >= next_frame_time and len(frames) < frame_count:
    renderer.update_scene(data, camera="fixed_cam")
    frames.append(renderer.render())
    next_frame_time += 1.0 / video_fps
```

Vì vậy video 30 FPS không làm thay đổi physics timestep. MuJoCo tạo các RGB frame; `ffmpeg` mã hóa chúng thành `artifacts/fixed_cam.mp4`. Ví dụ này giữ toàn bộ 150 frame trong RAM để code dễ đọc. Với video dài hoặc resolution lớn, nên ghi từng frame trực tiếp vào encoder.

## Bài tập

1. Đo biên độ `qpos` và giải thích ảnh hưởng của `kp`, joint damping và `ctrlrange`.
2. Đổi `CAMERA_NAME` trong `capture.py` sang `robot_cam_forward`, rồi so sánh `cam_xpos` trước và sau khi body chuyển động.
3. Tìm pixel gần nhất trong depth image và giải thích vật nào tạo ra giá trị đó.
4. Thay resolution thành 640×480 và kiểm tra shape của hai mảng trả về.
5. Đổi `CAMERA_NAME` trong `render_video.py` thành `tracking_cam`, rồi so sánh chuyển động tương đối của box trong hai video.
6. Đổi `VIDEO_FPS` thành 15 và 60 nhưng giữ nguyên `VIDEO_DURATION_S`; kiểm tra số frame và thời lượng video.

## Tài liệu

- [MJCF camera](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera)
- [Rendering](https://mujoco.readthedocs.io/en/stable/python.html#rendering)
