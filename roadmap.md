# Bản đồ Học tập MuJoCo (MuJoCo Learning Roadmap)

Danh mục này liệt kê 50 bài thực hành từ cơ bản đến nâng cao. Lộ trình được thiết kế theo phương pháp **"Học qua dự án nhỏ" (Project-based learning)**, giúp người học dần làm chủ trình mô phỏng MuJoCo, từ việc định nghĩa một vật rắn đơn giản đến thao tác điều khiển robot phức hợp thông qua Python API.

---

## Chặng 1: Nền tảng Không gian và Vật rắn (Rigid Body Fundamentals)
*Mục tiêu: Hiểu cách MuJoCo định nghĩa không gian 3D, cấu trúc phân cấp (cha - con) và thiết lập các thuộc tính tĩnh học, quang học cơ bản.*

| Bài | Chủ đề | Tags Trọng tâm | Mục tiêu Kiến thức |
| :--- | :--- | :--- | :--- |
| **01** | Empty world | `<mujoco>` | Khởi tạo cấu trúc file XML chuẩn và render môi trường trống. |
| **02** | Static object (box) | `<geom>` | Định nghĩa hình học cơ sở và hệ tọa độ cục bộ. |
| **03** | Ground and gravity | `<option>`, `<size>` | Thiết lập mặt phẳng tham chiếu và tham số trọng trường. |
| **04** | Body hierarchy | `<body>` | Xây dựng cây động học (kinematic tree) qua quan hệ cha-con. |
| **05** | Mesh loading | `<mesh>` | Nhập khẩu khối hình học phức tạp từ file `.stl`. |
| **06** | Material and texture | `<material>`, `<texture>` | Tùy chỉnh bề mặt vật thể phục vụ thị giác (rendering). |
| **07** | Frame & orientation | `pos`, `quat`, `axisangle` | Thao tác dịch chuyển và xoay hệ tọa độ trong không gian 3D. |
| **08** | Camera setup | `<camera>` | Gắn và cấu hình camera góc nhìn thứ ba/thứ nhất. |
| **09** | Light sources | `<light>` | Xử lý đổ bóng và nguồn sáng môi trường. |
| **10** | Inertial properties | `<inertial>` | Định nghĩa khối lượng, khối tâm và ma trận quán tính định lượng. |

---

## Chặng 2: Động học, Động lực học và Cơ cấu Chấp hành (Kinematics & Actuation)
*Mục tiêu: Đưa hệ thống vào trạng thái chuyển động, cấp bậc tự do (DOF) và thiết lập các cơ cấu truyền động cơ sở.*

| Bài | Chủ đề | Tags Trọng tâm | Mục tiêu Kiến thức |
| :--- | :--- | :--- | :--- |
| **11** | Simple pendulum | `<joint>` | Tạo khớp bản lề (hinge) và phân tích dao động con lắc đơn. |
| **12** | Double pendulum | `<joint>`, `<body>` | Khảo sát động học chuỗi mở (open-chain) nhiều bậc tự do. |
| **13** | Slider-crank | `type="slide"` | Khớp tịnh tiến và biến đổi chuyển động. |
| **14** | Rotational spring | `<actuator>` | Tích hợp lò xo xoắn vào khớp quay. |
| **15** | Linear spring-damper | `springref` | Khảo sát hệ số cản (damping) và dao động tắt dần. |
| **16** | Joint Constraints | `limited="true"` | Thiết lập biên độ góc quay vật lý cho khớp. |
| **20** | Torque control | `ctrlrange` | Áp dụng moment xoắn (torque) thuần túy lên khớp. |
| **21** | Velocity control | `velocity` | Sử dụng bộ điều khiển vận tốc nội bộ của MuJoCo. |
| **22** | Mass vs. acceleration | `mass`, `gravity` | Kiểm chứng Định luật II Newton trong môi trường mô phỏng. |
| **26** | Position actuator | `type="position"` | Sử dụng actuator điều khiển vị trí (PD controller ẩn). |
| **27** | Velocity actuator | `type="velocity"` | Tinh chỉnh bộ bám vận tốc. |
| **28** | Muscle actuator | `type="muscle"` | Khảo sát đặc tính phi tuyến của cơ bắp nhân tạo. |
| **29** | Gain & Bias | `bias`, `gainprm` | Ánh xạ (mapping) tín hiệu điều khiển đầu vào thành lực cơ học. |
| **32** | Tendon dynamics | `<tendon>` | Truyền động gián tiếp bằng cấu trúc cáp/dây (spatial/fixed). |
| **36** | Multi-actuator control | Nhiều `<actuator>` | Phân phối lực cho hệ thống nhiều bậc tự do. |
| **41** | 2-joint arm | `<joint>`, `<actuator>` | Tổng hợp: Xây dựng cánh tay robot phẳng (planar arm). |
| **42** | 3D manipulator | Khớp 6 DOF | Tổng hợp: Xây dựng cánh tay máy không gian. |

---

## Chặng 3: Cơ học Tiếp xúc và Cảm biến (Contact Mechanics & Perception)
*Mục tiêu: Xử lý tương tác vật lý giữa robot - môi trường (va chạm, ma sát) và thu thập dữ liệu trạng thái thông qua hệ thống cảm biến giả lập.*

| Bài | Chủ đề | Tags Trọng tâm | Mục tiêu Kiến thức |
| :--- | :--- | :--- | :--- |
| **17** | Contact dynamics | `solref`, `solimp` | Tinh chỉnh mô hình tiếp xúc mềm (soft-contact) của MuJoCo. |
| **18** | Joint friction | `friction` | Khảo sát ma sát tĩnh và ma sát động tại khớp. |
| **19** | Gyroscopic effects | `inertia`, `spin` | Phân tích hiệu ứng con quay hồi chuyển. |
| **23** | Asymmetric density | `<inertial>` | Khảo sát động lực học của vật thể có trọng tâm lệch. |
| **24** | Collision filtering | `contype`, `conaffinity` | Tối ưu hóa tính toán bằng cách loại trừ các va chạm không cần thiết. |
| **25** | Core Sensors | `<sensor>` | Gắn và cấu hình cảm biến lực/mô-men, IMU. |
| **30** | Tactile sensing | `<sensor>` | Mô phỏng cảm biến xúc giác trên bề mặt vật thể. |
| **31** | Touch-triggered force | `site`, `touch` | Xử lý sự kiện (event) khi xảy ra tương tác vật lý. |
| **33** | Soft constraints | `solref`, `solimp` | Làm mềm các ràng buộc (equality constraints) để tránh lỗi suy biến. |
| **35** | Friction fine-tuning | `stiffness`, `damping` | Hiệu chuẩn vật lý tiếp xúc cho các bề mặt đặc thù. |
| **37** | Load plate (Forceplate) | `force`, `torque` | Trích xuất phản lực mặt đất (GRF) cho robot có chân. |
| **48** | Vision perception | `camera`, `sensor` | Trích xuất hình ảnh (RGB/Depth) phục vụ bài toán thị giác. |

---

## Chặng 4: Tích hợp Hệ thống và Python API (System Integration)
*Mục tiêu: Đưa mô hình vào thực tiễn bằng cách sử dụng Python để can thiệp vòng lặp mô phỏng, xử lý nhiễu và thiết kế luật điều khiển phức hợp.*

| Bài | Chủ đề | Tags Trọng tâm | Mục tiêu Kiến thức |
| :--- | :--- | :--- | :--- |
| **34** | Actuator noise & delay | `noise`, `timeconst` | Giả lập độ trễ và nhiễu tín hiệu của phần cứng thực tế. |
| **38** | External disturbances | `appliedforce` | Đưa nhiễu ngoại cảnh (gió, va đập) vào hệ thống đang chạy. |
| **39** | User-defined actuators | `plugin`, `actdyn` | Mở rộng thư viện bằng cách tự định nghĩa động lực học actuator. |
| **40** | Python Control API | `mujoco.mj_step` | Nhúng luật điều khiển từ Python vào vòng lặp (simulation loop). |
| **43** | Mobile robot (2 wheels) | `slide`, `hinge` | Tích hợp: Mô phỏng xe tự hành cơ bản. |
| **44** | Differential drive | `gear`, `joint` | Tích hợp: Xe robot 2 bánh dẫn động vi sai. |
| **45** | Omnidirectional base | Bánh Mecanum | Tích hợp: Động học xe di chuyển đa hướng. |
| **46** | Walking biped | `<tendon>`, `<actuator>` | Tích hợp: Cơ sở robot hai chân (bipedal locomotion). |
| **47** | Gripper manipulation | `site`, `touch` | Tích hợp: Bài toán kẹp và thao tác vật thể. |
| **49** | Custom Plugins | `plugin` | Tích hợp các module vật lý bên ngoài (vd: thủy động lực học). |
| **50** | Final System Control | Toàn bộ API | Đồ án tổng hợp: Chạy chính sách điều khiển hoàn chỉnh (Policy) bằng Python. |