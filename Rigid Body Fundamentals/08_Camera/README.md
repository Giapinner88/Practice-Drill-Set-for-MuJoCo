# Bài 08: Thiết lập Camera và Trích xuất Dữ liệu Thị giác (Camera & Off-screen Rendering)

## 1. Cơ sở Lý thuyết
Trong các hệ thống robot dựa trên thị giác (Vision-based Robotics) hoặc Học tăng cường (RL), camera không chỉ đóng vai trò hiển thị đồ họa cho người dùng xem, mà nó là một **cảm biến (sensor)**. Cảm biến này có nhiệm vụ thu nhận trạng thái môi trường dưới dạng ma trận điểm ảnh (pixel arrays) để đưa vào các thuật toán xử lý ảnh (OpenCV) hoặc mạng nơ-ron tích chập (CNN).

MuJoCo cung cấp thẻ `<camera>` dựa trên mô hình lỗ kim (Pinhole Camera Model), cho phép gắn camera cố định vào môi trường (CCTV/Allo-centric) hoặc gắn lên các bộ phận chuyển động của robot (Eye-in-hand/Ego-centric).



**Lưu ý quan trọng về Hệ quy chiếu:**
MuJoCo sử dụng hệ tọa độ thuận chiều (Right-handed). Mặc định tại gốc tọa độ:
- Trục X: Hướng sang phải (Màu đỏ).
- Trục Y: Hướng vào trong màn hình (Màu xanh lá).
- Trục Z: Hướng lên trên (Màu xanh dương).

Tuy nhiên, **hệ quy chiếu cục bộ của thẻ `<camera>` lại khác biệt:**
- Camera luôn "nhìn" dọc theo trục **-Z** cục bộ của nó.
- Đỉnh đầu (up-vector) của camera là trục **+Y** cục bộ.
- Cạnh phải của ảnh là trục **+X** cục bộ.

## 2. Cấu trúc Mô hình (model.xml)
Chúng ta thiết lập hai camera: một camera cố định bám sát (track) vật thể đang trượt, và một camera gắn trực tiếp lên vật thể đó.

```xml
<mujoco model="camera_setup">
    <compiler angle="degree"/>
    <option gravity="0 0 -9.81" timestep="0.002"/>
    
    <worldbody>
        <light pos="0 0 5" dir="0 0 -1" directional="true"/>
        <geom type="plane" size="5 5 0.1" rgba="0.8 0.9 0.8 1"/>
        
        <camera name="fixed_cam" pos="0 -3 2" target="moving_box" fovy="60"/>

        <body name="moving_box" pos="0 0 0.5">
            <joint type="slide" axis="1 0 0"/>
            <geom type="box" size="0.2 0.2 0.2" rgba="0.2 0.8 0.2 1"/>
            
            <camera name="robot_cam" pos="0.3 0 0" euler="0 90 -90" fovy="90"/>
        </body>
    </worldbody>
</mujoco>
```

## 3. Phân tích Thẻ (Tag Analysis)

`<camera>`: Khai báo một cảm biến hình ảnh dựa trên mô hình lỗ kim (Pinhole Model).

- `name="tên_camera"`: (Bắt buộc) Định danh duy nhất để gọi camera này từ Python API hoặc bộ chọn (selector) của Viewer.

- `pos="x y z"` và `quat="w x y z"`: Vị trí và định hướng tương đối so với body chứa nó. Nhắc lại: Camera mặc định nhìn theo chiều âm của trục Z (cục bộ) và có đỉnh đầu hướng theo trục Y dương.

- `fovy="60"`: Góc mở dọc của ống kính (Field of View - Y) tính bằng độ.

  - Mặc định là $45 \degree$ (tương đương góc nhìn chuẩn của mắt người).

  - Nếu tăng lên $90 \degree$ hoặc $120 \degree$, camera sẽ thu được nhiều không gian hơn (tốt cho RL để bao quát môi trường), nhưng hình ảnh sẽ bị hiệu ứng mắt cá (fisheye distortion) ở viền ảnh.

- `target="tên_body_khác"`: (Tùy chọn) Kích hoạt tính năng theo dõi (Tracking). Nếu bạn chỉ định tên một body khác, MuJoCo sẽ liên tục nội suy quaternion của camera để hướng nhìn luôn khóa mục tiêu vào tâm của body đó, bất chấp sự dịch chuyển của cả hai. (Lưu ý: Nếu dùng target, các khai báo quat hay euler trước đó sẽ bị ghi đè).

- `ipd="0.068"`: (Tùy chọn) Khoảng cách giữa hai đồng tử (Inter-pupillary distance) tính bằng mét. Thông số này chỉ có ý nghĩa khi bạn render cảnh mô phỏng ra kính thực tế ảo (VR headsets) hỗ trợ 3D stereoscopic.

- `sensorsize="... ..."`: (Nâng cao) Kích thước vật lý của cảm biến ảnh trên máy ảnh giả lập, ảnh hưởng đến tiêu cự. Thường ít được sử dụng trừ khi bạn mô phỏng chính xác phần cứng của một camera công nghiệp cụ thể.