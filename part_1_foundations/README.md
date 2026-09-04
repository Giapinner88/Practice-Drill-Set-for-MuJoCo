# Phần 1 — MuJoCo Foundations

Học các bài theo thứ tự từ 01 đến 10. Mỗi bài sử dụng lại kiến thức của bài trước và thêm một nhóm tag hoặc API mới.

| Bài | Nội dung |
| --- | --- |
| [01 — Empty world](01_empty_world) | `MjModel`, `MjData` và simulation loop |
| [02 — Static geoms](02_static_geoms) | Các geometry cơ bản và quy ước `size` |
| [03 — Gravity and ground](03_gravity_and_ground) | Vật thể tự do, trọng lực và contact với ground |
| [04 — Body hierarchy](04_body_hierarchy) | Cây body, local pose và world pose |
| [05 — Mesh loading](05_mesh_loading) | Visual mesh và collision mesh |
| [06 — Materials and textures](06_materials_and_textures) | Texture tạo sẵn và texture từ file |
| [07 — Frames and orientation](07_frames_and_orientation) | Euler, quaternion, axis-angle và `zaxis` |
| [08 — Lights](08_lights) | Directional, point và spot light |
| [09 — Cameras](09_cameras) | Camera cố định/gắn body/tracking, RGB, depth và video |
| [10 — Inertial properties](10_inertial_properties) | Mass, center of mass và inertia tensor |

Chạy `simulate.py` không kèm `--duration` để giữ viewer mở. Dùng `--headless` khi bạn chỉ cần kết quả số; nếu không đặt duration, headless chạy 5 giây simulation time.
