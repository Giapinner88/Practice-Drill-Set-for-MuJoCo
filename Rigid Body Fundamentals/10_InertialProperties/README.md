# Bài 10: Khối lượng và Ma trận Quán tính Định lượng (Inertial Properties)

## 1. Cơ sở Lý thuyết
Một vật rắn tuyệt đối trong không gian 3D được đặc trưng động lực học hoàn toàn bởi 10 thông số (Inertial Parameters):
1.  **Khối lượng ($m$):** 1 thông số vô hướng (scalar).
2.  **Vị trí Khối tâm (Center of Mass - CoM):** 3 thông số tọa độ $(c_x, c_y, c_z)$ tương đối so với hệ quy chiếu cục bộ của `body`.
3.  **Tensor Quán tính (Inertia Tensor - $I$):** Một ma trận đối xứng $3 \times 3$, được xác định bởi 6 thông số độc lập dọc theo các trục quán tính chính.

Nếu không khai báo rõ ràng, MuJoCo sẽ tự động xấp xỉ khối lượng và ma trận quán tính dựa trên thể tích hình học của `<geom>`, giả định vật thể là đặc và đồng chất. Đối với robot thực tế (nơi linh kiện bên trong phân bố không đều), chúng ta **bắt buộc** phải ghi đè (override) sự nội suy này bằng thẻ `<inertial>`.

> 📌 **Lưu ý: Nhập Ma trận Quán tính từ phần mềm CAD**
>
> Khi bạn thiết kế linh kiện trên các phần mềm CAD (như SolidWorks, Fusion 360, Inventor) và trích xuất đặc tính vật lý, ma trận quán tính thu được thường là một tensor đối xứng $3 \times 3$ đầy đủ, bao gồm cả các moment quán tính chính (nằm trên đường chéo) và các moment quán tính chéo (products of inertia):
>
> 
>
> $$I = \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{xy} & I_{yy} & I_{yz} \\ I_{xz} & I_{yz} & I_{zz} \end{bmatrix}$$
>
> Nếu các thành phần chéo ($I_{xy}, I_{xz}, I_{yz}$) khác không ($0$), bạn **không thể** nhét trực tiếp $I_{xx}, I_{yy}, I_{zz}$ vào thuộc tính `diaginertia` vì điều đó làm sai lệch động lực học quay của vật thể. Để giải quyết, MuJoCo cung cấp 2 phương án:
>
> **Phương án 1: Sử dụng `fullinertia` (Khuyên dùng cho người mới)**
> Thay vì dùng `diaginertia`, bạn khai báo tường minh cả 6 giá trị độc lập theo thứ tự:
> ```xml
> <inertial pos="..." mass="..." fullinertia="Ixx Iyy Izz Ixy Ixz Iyz"/>
> ```
>
> **Phương án 2: Chéo hóa ma trận (Matrix Diagonalization)**
> Bạn tính toán các trị riêng (eigenvalues) và vector riêng (eigenvectors) của ma trận $I$. 
> - 3 trị riêng chính là giá trị để điền vào `diaginertia`.
> - 3 vector riêng tạo thành một ma trận xoay (Rotation Matrix). Bạn chuyển đổi ma trận xoay này thành quaternion và truyền vào thuộc tính `quat` của thẻ `<inertial>`. Phương án này giúp hệ tọa độ cục bộ của bộ giải đồng phương với hệ tọa độ quán tính chính, tối ưu hóa tốc độ tính toán của MuJoCo.

## 2. Cấu trúc Mô hình (Đối chứng A/B)
Để thấy rõ tầm quan trọng của thẻ `<inertial>`, chúng ta thả rơi hai khối trụ (cylinder) có cùng kích thước hình học từ cùng một độ cao:

```xml
<body name="uniform_cylinder" pos="-0.5 0 2">
    <freejoint/>
    <geom type="cylinder" size="0.1 0.4" rgba="0.2 0.8 0.2 1"/>
</body>

<body name="offset_cylinder" pos="0.5 0 2">
    <inertial pos="0 0.2 0.3" mass="2.0" diaginertia="0.05 0.05 0.05"/>
    <freejoint/>
    <geom type="cylinder" size="0.1 0.4" rgba="0.8 0.2 0.2 1"/>
</body>
```

## 3. Phân tích Thẻ (Tag Analysis)

`<inertial>`: Thẻ định nghĩa đặc tính động lực học của body chứa nó. Thẻ này luôn đi kèm với khối lượng và phải được đặt trước các thẻ `<joint>` và `<geom>`.

    `pos="0 0.2 0.3"`: Định nghĩa tọa độ của khối tâm (CoM) bị lệch khỏi tâm hình học. Cụ thể, khối tâm bị đẩy lệch sang trục Y (0.2) và dồn lên phía trên trục Z (0.3). Điều này làm vật thể trở nên mất trọng tâm (top-heavy).

    `mass="2.0"`: Khối lượng tổng cộng của vật thể là 2.0 kg. Khai báo này vô hiệu hóa quá trình tính toán dựa trên thể tích của `<geom>`.

    `diaginertia="0.05 0.05 0.05"`: 3 moment quán tính chính (diagonal inertia) $I_{xx}​,I_{yy}​,I_{zz}$​ dọc theo các trục tọa độ của CoM. MuJoCo yêu cầu bạn cung cấp các giá trị này (đơn vị $\text{kg}⋅\text{m}^2$). Nếu hệ tọa độ quán tính không trùng với hệ tọa độ của body, bạn có thể dùng thêm thuộc tính quat để xoay tensor này, thay vì khai báo toàn bộ ma trận (full inertia matrix) rất phức tạp.

**Vấn đề thảo luận**

Việc bạn trích xuất ma trận quán tính từ phần mềm thiết kế phần cứng thường sẽ cho ra một ma trận đầy đủ có cả các thành phần chéo ($I_{xy}, I_{xz}, I_{yz}$). Vì thẻ `<inertial>` của MuJoCo mặc định chỉ dùng `diaginertia` (đường chéo), bạn có muốn tôi thiết kế một hộp lưu ý (Note box) ngắn trong bài 10 này để hướng dẫn sinh viên cách sử dụng thuộc tính `fullinertia` hoặc dùng quaternion (`quat`) để xoay hệ tọa độ quán tính chính không? Đây là một kỹ năng bắt buộc khi làm "Sim-to-Real".