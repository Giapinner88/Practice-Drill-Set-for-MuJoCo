# Bài 06: Vật liệu và Họa tiết bề mặt (Material & Texture)

## 1. Cơ sở Lý thuyết
Sau khi nhập khẩu hình dáng vật lý (Mesh) ở Bài 05, chúng ta cần hoàn thiện diện mạo quang học (visual appearance) của hệ thống. Trong MuJoCo, quá trình kết xuất đồ họa (rendering) không chỉ để "cho đẹp", mà nó đặc biệt quan trọng nếu mô hình của bạn được dùng để tạo ra tập dữ liệu (dataset) huấn luyện cho các thuật toán Thị giác Máy tính (Computer Vision) hoặc Học tăng cường dựa trên hình ảnh (Vision-based RL).

MuJoCo tách biệt diện mạo thành hai khái niệm lớp:
1.  **Texture (Họa tiết):** Là dữ liệu gốc dạng ảnh (image data) hoặc dữ liệu sinh từ hàm toán học (procedural data) như bầu trời (skybox) hay bàn cờ (checker).
2.  **Material (Vật liệu):** Là lớp bọc bên ngoài Texture. Nó quyết định cách ánh sáng tương tác với họa tiết đó thông qua các đặc tính quang học vật lý (physically-based rendering properties) như độ phản xạ (reflectance), độ phát xạ (emission), hoặc độ bóng (specular).

Một `geom` không thể gọi trực tiếp `texture`. Nó phải gọi `material`, và `material` sẽ sử dụng `texture` làm bản đồ màu (color map).



## 2. Cấu trúc Mô hình (model.xml)
Tương tự như `<mesh>`, cả `<texture>` và `<material>` đều phải được định nghĩa trong khu vực `<asset>` để hệ thống nạp vào bộ nhớ trước khi sử dụng.

```xml
<mujoco model="material_and_texture">
    <compiler angle="degree"/>
    
    <asset>
        <texture name="tex_checker" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.8 0.8 0.8" width="512" height="512"/>
        
        <material name="mat_ground" texture="tex_checker" reflectance="0.3" specular="0.5" shininess="0.5"/>
        
        <material name="mat_metal" rgba="0.7 0.7 0.8 1" reflectance="0.8" metallic="0.8"/>
    </asset>

    <worldbody>
        <light pos="0 0 5" dir="0 0 -1" directional="true" castshadow="true"/>
        
        <geom name="ground" type="plane" size="5 5 0.1" material="mat_ground"/>
        
        <body name="metal_sphere" pos="0 0 1">
            <freejoint/>
            <geom type="sphere" size="0.5" material="mat_metal"/>
        </body>
    </worldbody>
</mujoco>
```
## 3. Phân tích Thẻ (Tag Analysis)
`<texture>`:
- `type="2d"`: Loại bản đồ ảnh. Nó sẽ được trải phẳng lên bề mặt vật thể. (MuJoCo còn hỗ trợ dạng skybox để bọc toàn bộ môi trường, hoặc cube cho vật thể 3D).

`builtin="checker"`: Tính năng sinh họa tiết từ thuật toán nội bộ của MuJoCo thay vì phải load một file .png bên ngoài.

- `rgb1` và `rgb2`: Hai màu chủ đạo của bàn cờ.

`<material>`:
- `reflectance="0.3"`: Hệ số phản xạ môi trường. Nếu set lên 1.0, vật thể sẽ trở thành một tấm gương hoàn hảo.

- `metallic="0.8"`: Định nghĩa tính chất vật liệu (dielectric vs. conductor). Kim loại sẽ nhuộm màu ánh sáng phản xạ bằng chính màu gốc (rgba) của nó, trong khi phi kim loại phản xạ ánh sáng trắng của nguồn sáng.

Tại thẻ `<geom>`:
- `material="mat_ground"`: Tham chiếu tới tên của vật liệu đã định nghĩa trong `<asset>`. Lưu ý: Khi đã sử dụng material, thuộc tính rgba nếu được khai báo trong `<geom>` sẽ bị bỏ qua (overriden).


> 📌 **Lưu ý: Trải Họa tiết Thực tế (Custom Textures) & UV Mapping**
>
> Trong các dự án RL thị giác (Vision-based RL), bạn có thể cần ngẫu nhiên hóa bề mặt vật thể (Domain Randomization) bằng cách nạp các file ảnh thực tế:
> ```xml
> <texture name="tex_custom" type="2d" file="textures/wood.png"/>
> ```
> Nếu bạn áp dụng vật liệu chứa ảnh này lên một `<geom type="mesh">` và thấy hình ảnh bị kéo giãn, méo mó hoặc lặp lại sai tỷ lệ, **lỗi không nằm ở MuJoCo**. 
> 
> Trình mô phỏng không biết cách "gói" (wrap) một bức ảnh 2D lên bề mặt 3D phức tạp. Việc phân bổ điểm ảnh này phụ thuộc hoàn toàn vào hệ tọa độ **UV Mapping** – dữ liệu bắt buộc phải được thiết lập và nhúng sẵn vào file `.obj` hoặc `.stl` từ các phần mềm thiết kế đồ họa (như Blender, Maya) trước khi xuất file.