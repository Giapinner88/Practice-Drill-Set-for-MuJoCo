# Bài 07: Khung Tọa độ và Định hướng Không gian (Frame & Orientation)

## 1. Cơ sở Lý thuyết
Trong cấu trúc cây động học (Kinematic Tree) của MuJoCo, vị trí và tư thế của một `body` nhánh con luôn được xác định tương đối so với hệ quy chiếu (Frame) của `body` nhánh cha chứa nó, chứ không phải so với gốc tọa độ toàn cục (World Frame).

Để xác định một tư thế (Pose) trong không gian 3D, chúng ta cần 6 thông số:
1.  **Dịch tiến (Translation):** Vector vị trí 3D $(x, y, z)$.
2.  **Xoay (Rotation/Orientation):** Được biểu diễn thông qua nhiều chuẩn toán học khác nhau.



MuJoCo hỗ trợ nhiều cách khai báo định hướng để tiện lợi cho người dùng, nhưng bên dưới engine, tất cả đều được trình biên dịch chuyển đổi về **Quaternion** (đại số siêu phức 4 chiều) để tránh hiện tượng khóa các-đăng (Gimbal Lock) và tối ưu hóa tốc độ nhân ma trận.

## 2. Cấu trúc Mô hình (Các Phương pháp Định hướng)
Dưới đây là các thẻ `geom` biểu diễn cùng một khối hộp, nhưng được xoay nghiêng 45 độ quanh trục Z cục bộ bằng các phương pháp toán học khác nhau:

```xml
<mujoco model="frames_and_orientation">
    <compiler angle="degree"/> <worldbody>
        <body name="reference_frame" pos="0 0 1">
            
            <geom type="box" size="0.2 0.2 0.2" pos="1 0 0" axisangle="0 0 1 45" rgba="1 0 0 1"/>

            <geom type="box" size="0.2 0.2 0.2" pos="0 1 0" quat="0.92388 0 0 0.38268" rgba="0 1 0 1"/>

            <geom type="box" size="0.2 0.2 0.2" pos="-1 0 0" euler="0 0 45" rgba="0 0 1 1"/>
            
            <geom type="box" size="0.2 0.2 0.2" pos="0 -1 0" zaxis="1 1 1" rgba="1 1 0 1"/>

        </body>
    </worldbody>
</mujoco>
```

## 3. Phân tích Thẻ (Tag Analysis)

Tất cả các thuộc tính định hướng dưới đây đều có thể áp dụng cho `<body/>`, `<geom/>`, `<site/>`, `<joint/>` và `<camera/>`:

- `pos="x y z"`: Vector dịch tiến tương đối so với hệ quy chiếu cha.

- `axisangle="ax ay az angle"`: Định nghĩa một trục xoay (vector chỉ phương) và một góc xoay quanh trục đó. Rất trực quan khi lập trình phần cứng robot.

- `quat="w x y z"`: Đại số Quaternion. Mặc dù khó nhẩm bằng đầu, đây là định dạng bắt buộc nếu bạn xuất thông số trực tiếp từ thuật toán SLAM hoặc thư viện tính toán động học (như KDL, Pinocchio).

- `euler="roll pitch yaw"`: Góc xoay Euler. Cần đặc biệt chú ý đến thứ tự xoay (Euler convention). Thuộc tính eulerseq trong thẻ `<compiler>` cho phép đổi thứ tự này (ví dụ: `eulerseq="zyx"`), nhưng nếu không khai báo, MuJoCo dùng xyz.

- `zaxis="vx vy vz"`: Đặt lại hướng của trục Z cục bộ sao cho nó song song với vector ($vx,vy,vz$). Trục X và Y sẽ được MuJoCo tự động tính toán bằng tích có hướng (cross product) để tạo thành hệ tọa độ vuông góc.

> 📌 **Lưu ý: Euler Angles vs. Quaternions trong Điều khiển**
>
> Khi dựng mô hình trong file XML (giai đoạn thiết kế tĩnh), việc sử dụng góc `euler` (Roll-Pitch-Yaw) hoặc `axisangle` là hoàn toàn hợp lý vì nó trực quan, dễ nhẩm bằng đầu và dễ chỉnh sửa.
>
> Tuy nhiên, khi bạn chuyển sang lập trình điều khiển thông qua Python API (ví dụ: đọc trạng thái robot từ `mjData.qpos` hoặc điều khiển quỹ đạo tay máy bằng Jacobian), bạn **bắt buộc** phải tập thói quen làm việc với **Quaternion (`quat`)**.
>
> 1.  **Tránh Gimbal Lock:** Việc nội suy (interpolation) quỹ đạo giữa hai điểm bằng góc Euler sẽ dẫn đến hiện tượng khóa các-đăng (mất một bậc tự do khi góc Pitch đạt $\pm 90^\circ$), khiến robot phản ứng giật cục hoặc hệ phương trình động học bị suy biến (singularity).
> 2.  **Đồng bộ với API:** Các hàm tính toán nội bộ của MuJoCo (như `mj_jac` để tính Jacobian, hay mảng `mjData.xquat`) đều trả về kết quả dưới định dạng Quaternion $[w, x, y, z]$. Việc cố gắng chuyển đổi ngược lại Euler ở mỗi chu kỳ điều khiển (ví dụ: $0.002s$) là một sự lãng phí tài nguyên tính toán không cần thiết.
> 3.  **Toán học mượt mà:** Nội suy dạng cầu (SLERP - Spherical Linear Interpolation) trên Quaternion luôn đảm bảo quãng đường xoay ngắn nhất và mượt mà nhất trong không gian 3D.