# Bài 01: Khởi tạo Không gian Tham chiếu Cơ sở (Empty World)

## 1. Cơ sở Lý thuyết
Trong mô phỏng động lực học, một "môi trường trống" không đơn thuần là một màn hình hiển thị không có đồ họa. Về mặt cơ học giải tích, việc khởi tạo môi trường đồng nghĩa với việc định nghĩa **Hệ quy chiếu quán tính toàn cục (Global Inertial Reference Frame)**, thường được ký hiệu là hệ tọa độ gốc $\Sigma_0$. 

Bất kỳ một chuỗi động học (kinematic chain) nào được xây dựng sau này, từ các cơ cấu bản lề đơn giản cho đến các hệ thống có tính phi tuyến cao như con lắc ngược mềm (flexible inverted pendulum), đều phải được gắn gốc tọa độ vào hệ quy chiếu này để bộ giải (solver) có thể thiết lập và giải hệ phương trình vi phân chuyển động.

## 2. Cấu trúc Mô hình (model.xml)
Để khởi tạo một hệ quy chiếu hợp lệ trong MuJoCo, tệp XML cần một cấu trúc bao đóng tối thiểu như sau:

```xml
<mujoco model="empty_world">
    <worldbody>
        </worldbody>
</mujoco>
```

## 3. Phân tích Thẻ (Tag Analysis)

<mujoco>: Thẻ gốc (root element). Định nghĩa ranh giới của toàn bộ mô hình. Thuộc tính model chỉ đóng vai trò định danh.

<worldbody>: Đây là thẻ quan trọng nhất trong bài này. Nó đại diện cho môi trường vật lý tĩnh, có khối lượng vô hạn và không thể di chuyển. Tọa độ của <worldbody> mặc định nằm tại (0,0,0) trong không gian Descartes. Mọi vật thể được định nghĩa bên ngoài thẻ này sẽ bị trình biên dịch của MuJoCo từ chối.

**Vấn đề trao đổi:**
Trong đoạn mã Python phía trên, tôi đã sử dụng vòng lặp `while viewer.is_running():` và gọi hàm `mujoco.mj_step(model, data)`. 

Mặc dù hệ thống hiện tại có 0 bậc tự do (0 DOF) và không có bất kỳ vật thể nào, hàm `mj_step` vẫn được gọi liên tục để tiến thời gian (time step) lên phía trước. Theo bạn, dưới góc độ tính toán tài nguyên (computational overhead), việc duy trì hàm `mj_step` trong một môi trường tĩnh tuyệt đối như thế này có gây lãng phí chu kỳ của CPU không, hay nó là một yêu cầu bắt buộc của kiến trúc phần mềm MuJoCo để duy trì trạng thái của `mjData`?