# Lộ trình Thực hành MuJoCo: Từ Cơ sở Động lực học đến Tích hợp Hệ thống
*(MuJoCo Practical Drill Set: From Dynamics Fundamentals to System Integration)*

## 1. Giới thiệu (Introduction)
Kho lưu trữ này được thiết kế như một giáo trình thực hành mã nguồn mở (open-source tutorial series) dành cho những người mới bắt đầu làm quen với trình mô phỏng vật lý MuJoCo. Thay vì chỉ cung cấp các tài liệu lý thuyết, dự án này tập trung vào phương pháp "học qua từng bài toán" (problem-based learning), giúp người học dần làm chủ từ cú pháp XML cơ bản đến việc điều khiển các hệ thống robot phức hợp thông qua Python API.

Mục tiêu cốt lõi: Cung cấp một bộ tài liệu có **tính kế thừa cao**, dễ tiếp cận, làm cầu nối giữa lý thuyết cơ học/điều khiển và môi trường mô phỏng thực tế.

## 2. Lộ trình Học tập (Learning Syllabus)
Các bài thực hành được chia thành 4 chặng (modules), được thiết kế với độ khó tăng dần. Người học bắt buộc phải nắm vững các khái niệm ở chặng trước để xây dựng hệ thống ở chặng sau.

### Chặng 1: Nền tảng Không gian và Vật rắn (Fundamentals of Rigid Bodies)
*Mục tiêu: Hiểu cách MuJoCo định nghĩa không gian, cấu trúc phân cấp (hierarchy) và các đặc tính tĩnh học của vật rắn.*
- **Bài 1-3:** Khởi tạo môi trường (World, Gravity, Ground).
- **Bài 4-7:** Cấu trúc động học phân cấp (Body trees, Frames, Quaternions).
- **Bài 8-10:** Xử lý đồ họa và thuộc tính cơ học (Meshes, Materials, Inertial properties, Camera, Lighting).

### Chặng 2: Động học, Động lực học và Cơ cấu Chấp hành (Kinematics & Actuation)
*Mục tiêu: Đưa hệ thống vào trạng thái chuyển động, xác định các bậc tự do (DOF) và thiết lập luật điều khiển cơ sở.*
- **Bài 11-13:** Khớp nối cơ bản (Hinge, Slide) - Bài toán Con lắc và Cơ cấu tay quay con trượt.
- **Bài 14-16:** Ràng buộc phần cứng (Joint limits, Springs, Damping).
- **Bài 20-22, 26-29:** Phân tích các bộ truyền động (Position/Velocity/Torque/Muscle Actuators) và ánh xạ lực (Gain, Bias).
- **Bài 32, 41-42:** Truyền động gián tiếp (Tendons) và Động học cánh tay máy (Multi-DOF Manipulators).

### Chặng 3: Cơ học Tiếp xúc và Cảm biến (Contact Mechanics & Perception)
*Mục tiêu: Mô hình hóa sự tương tác giữa robot và môi trường – ranh giới quyết định tính chân thực của mô phỏng (Sim-to-Real).*
- **Bài 17-18, 33, 35:** Mô hình va chạm mềm (Soft constraints, `solref`, `solimp`) và Ma sát khớp.
- **Bài 24:** Lọc va chạm (Collision filtering - `contype`, `conaffinity`).
- **Bài 25, 30-31, 37:** Cảm biến nội tại và ngoại cảm (Force/Torque sensors, Touch, Tactile).
- **Bài 34, 38:** Giả lập môi trường thực (Nhiễu tín hiệu - Noise, Lực ngoại lai - External forces).

### Chặng 4: Tích hợp Hệ thống (System Integration via Python API)
*Mục tiêu: Thoát khỏi môi trường XML thuần túy, sử dụng Python để nhúng các thuật toán điều khiển và AI.*
- **Bài 39-40:** Giao tiếp MuJoCo - Python (`mujoco.mj_step`, trích xuất ma trận trạng thái).
- **Bài 43-45:** Cấu trúc Robot di động (Differential drive, Omnidirectional/Mecanum base).
- **Bài 46:** Cơ sở Robot tự hành có chân (Simple Walking Biped).
- **Bài 47, 50:** Tích hợp toàn diện: Hệ thống Robot + Môi trường + Luật điều khiển (Python Policy).

## 3. Cấu trúc một Bài học Tiêu chuẩn
Mỗi thư mục bài học trong kho lưu trữ này được chuẩn hóa với các thành phần sau để đảm bảo tính minh bạch:
1. `model.xml`: File định nghĩa cấu trúc cơ học.
2. `simulate.py`: Script Python để khởi chạy, thu thập dữ liệu hoặc áp dụng lực điều khiển.
3. `README.md` (cục bộ): Giải thích lý thuyết vật lý cốt lõi của bài toán, các thẻ XML trọng tâm được sử dụng, và phân tích kết quả kỳ vọng.

## 4. Cài đặt và Sử dụng (Installation & Usage)
```bash
# Yêu cầu: Python >= 3.8
git clone [https://github.com/Giapinner88/Practice-Drill-Set-for-MuJoCo.git](https://github.com/Giapinner88/Practice-Drill-Set-for-MuJoCo.git)
cd Practice-Drill-Set-for-MuJoCo
pip install mujoco numpy
