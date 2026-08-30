# P1-06 — Materials và textures

## Chuẩn đầu ra

Người học phân biệt texture (dữ liệu ảnh/thủ tục), material (tham số hiển thị có thể tái sử dụng) và geom (đối tượng tham chiếu material).

## Hai model đối chứng

- `model.xml`: texture `builtin="checker"`, không phụ thuộc file ảnh.
- `file_textures.xml`: nạp texture gỗ/vải từ thư mục `assets/`.

Model chính minh họa chuỗi tham chiếu:

```xml
<texture name="tex_checker" type="2d" builtin="checker" .../>
<material name="mat_ground" texture="tex_checker" .../>
<geom type="plane" material="mat_ground" .../>
```

Các tham số như `specular`, `shininess`, `reflectance`, `metallic` và `roughness` điều khiển appearance của renderer. Chúng không thay đổi friction, contact hoặc khối lượng. Không nên suy ra tính chất cơ học từ vẻ ngoài của material.

## Chạy thí nghiệm

```bash
python part_1_foundations/06_materials_and_textures/simulate.py
python part_1_foundations/06_materials_and_textures/simulate.py --model file_textures.xml
```

Kết quả kỳ vọng: model đầu chạy không cần ảnh ngoài; model thứ hai chỉ biên dịch khi đường dẫn asset hợp lệ.

## Bài tập

1. Đổi `texrepeat` và phân biệt nó với thay đổi kích thước geom.
2. Giữ nguyên contact parameters nhưng đổi material; xác nhận trajectory không đổi.
3. Xóa tạm một file texture và đọc thông báo compiler để xác định asset lỗi.

## Tài liệu

- [MJCF texture](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-texture)
- [MJCF material](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-material)
