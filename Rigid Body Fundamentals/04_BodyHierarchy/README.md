# Bài 04: Cấu trúc Phân cấp và Bậc tự do Cơ sở (Body Hierarchy & Free Joint)

## 1. Cơ sở Lý thuyết
Động lực học đa vật (Multibody Dynamics) trong MuJoCo được tổ chức dưới dạng một đồ thị có hướng (directed graph), cụ thể là một **Cây Động học (Kinematic Tree)**. 
- Gốc của cây luôn là `worldbody` (Hệ quy chiếu toàn cục $\Sigma_0$).
- Các nhánh của cây là các `body` (Vật thể mang khối lượng). Mỗi `body` có hệ quy chiếu cục bộ riêng.
- Liên kết giữa gốc và nhánh, hoặc giữa nhánh cha và nhánh con, được định nghĩa bởi các `joint` (Khớp). 

Nếu bạn đặt một `body` bên trong `worldbody` mà **không có** `joint`, nó ngầm định bị hàn cứng (welded) vào gốc. Để giải phóng toàn bộ 6 bậc tự do (3 tịnh tiến, 3 quay) cho vật thể đó tự do bay lơ lửng trong không gian (như một vật thể ném đi), chúng ta cần một loại khớp đặc biệt: **Free Joint**.

## 2. Cấu trúc Mô hình (Giản lược)
Khối hộp tĩnh ở Bài 03 nay được bọc bên trong một thẻ `<body>`. 

```xml
<body name="falling_box" pos="0 0 2">
    <freejoint name="box_free_joint"/>
    <geom type="box" size="0.5 0.5 0.5" rgba="0.2 0.8 0.2 1"/>
</body>
```

## 3. Phân tích Thẻ (Tag Analysis)
`<body>`: Thực thể mang khối lượng.

    pos="0 0 2": Tọa độ của hệ quy chiếu cục bộ (local frame) của body này so với hệ quy chiếu của cha nó (trong trường hợp này là worldbody). Việc tách tọa độ ra khỏi thẻ <geom> giúp quản lý chuyển động dễ dàng hơn.

`<freejoint>`: Thẻ viết tắt (shortcut) cho một liên kết 6 bậc tự do (6-DOF).

    Khớp này giải phóng hoàn toàn vật thể khỏi hệ quy chiếu gốc. Nó cung cấp cho body 3 tọa độ vị trí (X, Y, Z) và 4 tọa độ quaternion (biểu diễn góc quay 3D). Khi có freejoint, bộ giải (solver) sẽ bắt đầu nội suy ma trận khối lượng hữu hạn cho vật thể và tích phân các phương trình định luật II Newton.

**Vấn đề thảo luận:**

Ở đoạn mã trên, tôi không hề khai báo thẻ `<inertial>` để chỉ định khối lượng ($m$) cho `falling_box`, nhưng khối hộp vẫn rơi xuống dưới tác dụng của trọng lực. Điều này có nghĩa là MuJoCo đang âm thầm tự gán cho nó một giá trị khối lượng nào đó dựa trên thẻ `<geom>` bên trong nó. 

Cơ chế "đoán khối lượng" (mass inferring) mặc định này dựa trên yếu tố vật lý nào? Và nếu vật thể của bạn là một robot có các linh kiện như động cơ, pin, hay khung vỏ rỗng bên trong, liệu việc phụ thuộc vào tính năng tự nội suy này của MuJoCo có gây ra sai lệch nghiêm trọng cho bài toán động lực học nghịch (Inverse Dynamics) khi tính toán lực điều khiển (torque) sau này không?