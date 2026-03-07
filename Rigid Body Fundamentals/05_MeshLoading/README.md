# Bài 05: Nhập khẩu Lưới Đa giác và Tối ưu hóa Va chạm (Mesh Loading & Collision Optimization)

## 1. Cơ sở Lý thuyết
Khi mô hình hóa các hệ thống thực tế (ví dụ: một khớp vai robot được in 3D), việc sử dụng các hình khối nguyên thủy (primitives như `box`, `cylinder`, `sphere`) là không đủ để mô tả hình dáng vật lý. MuJoCo cho phép nhập khẩu trực tiếp các lưới đa giác (meshes) dưới định dạng `.stl` hoặc `.obj`.

Tuy nhiên, trong mô phỏng động lực học, chúng ta phải đối mặt với **Nghịch lý Đồ họa - Tính toán (Graphics-Computation Trade-off)**: Một mô hình càng đẹp (nhiều đa giác) thì tính toán va chạm càng chậm.

Khi xuất một file `.stl` từ phần mềm CAD (Computer-Aided Design), ma trận đỉnh (vertex matrix) thường được lưu trữ dưới dạng lưới đa giác chi tiết (detailed polygonal mesh). Trình mô phỏng vật lý như MuJoCo tính toán va chạm dựa trên giao tuyến của các mặt phẳng (planes) và các cạnh (edges) của lưới này. Nếu một chi tiết cơ khí có chứa các lỗ ren (threaded holes), góc bo (fillets), hoặc các bề mặt cong phức tạp (splines), số lượng đa giác có thể lên tới hàng chục nghìn. Việc buộc bộ giải (solver) phải tính toán khoảng cách ngắn nhất (shortest distance) giữa hai lưới chi tiết như vậy tại mỗi bước thời gian (`timestep = 0.002s`) sẽ dẫn đến sự bùng nổ độ phức tạp tính toán (computational explosion), thường là $O(N2)$ với $N$ là số lượng mặt của hai lưới.

Để giải quyết vấn đề này, tiêu chuẩn công nghiệp (industry standard) yêu cầu chia tách một chi tiết cơ khí thành hai lớp biểu diễn độc lập bên trong cùng một `<body>`:
1.  **Visual Mesh (Lưới Hiển thị):** Giữ nguyên độ chi tiết cao nhất (high-poly) từ file CAD để phục vụ việc quan sát, kết xuất hình ảnh (rendering), hoặc huấn luyện thị giác máy tính (computer vision). Lưới này sẽ bị tắt chức năng tính toán va chạm.
2.  **Collision Mesh (Lưới Va chạm):** Một phiên bản đã được đơn giản hóa (low-poly) hoặc tối ưu bằng thuật toán Bao lồi (**Convex Hull**). Nó bao bọc vừa vặn lấy Visual Mesh nhưng có số lượng bề mặt (faces) tối thiểu, giúp bộ giải tính toán lực tiếp xúc nhanh chóng và ổn định.

## 2. Cấu trúc Mô hình (Phân tách Chức năng)
Giả sử chúng ta có hai file: `link1_visual.stl` (chi tiết cao) và `link1_collision.stl` (đã được làm thô hoặc bao lồi bằng phần mềm thứ ba như Blender/MeshLab).

Chúng ta khai báo tài nguyên (assets) trước khi sử dụng chúng trong cây động học:

```xml
<mujoco model="mesh_loading">
    <compiler angle="degree"/>
    
    <asset>
        <mesh file="assets/meshes/link1_visual.stl" name="mesh_vis"/>
        <mesh file="assets/meshes/link1_collision.stl" name="mesh_col"/>
    </asset>

    <worldbody>
        <body name="robot_link_1" pos="0 0 1">
            <freejoint/>
            
            <geom type="mesh" mesh="mesh_vis" contype="0" conaffinity="0" group="1" rgba="0.5 0.5 0.5 1"/>
            
            <geom type="mesh" mesh="mesh_col" rgba="0 1 0 0.2" group="3"/>
        </body>
    </worldbody>
</mujoco>
```

## 3. Phân tích Thẻ (Tag Analysis)

- `<asset>`: Khu vực khai báo tập trung. Mọi tài nguyên như file lưới (`<mesh>`), vật liệu (`<material>`), hoặc họa tiết (`<texture>`) phải được tải vào bộ nhớ tại đây trước khi được gọi tên bởi các thẻ `<geom>` bên dưới. Điều này giúp tránh việc tải lặp lại cùng một file nhiều lần.

- `<mesh file="...">`: Đường dẫn tương đối (relative path) tới file 3D. MuJoCo hỗ trợ tốt nhất định dạng `.stl` nhị phân (binary STL) để tối ưu tốc độ đọc.

- `contype="0" conaffinity="0"`: Đây là cơ chế Lọc Va chạm (Collision Filtering). Việc đặt cả hai giá trị về 0 yêu cầu engine bỏ qua lưới Visual Mesh này trong mọi phép thử giao tuyến (intersection tests).

- Thuộc tính group: Phân lớp hiển thị. Bạn có thể sử dụng giao diện người dùng (UI) của mujoco.viewer để bật/tắt hiển thị từng nhóm. Theo quy ước ngầm, nhóm 1 thường chứa hình ảnh đẹp, nhóm 3 chứa khung va chạm.