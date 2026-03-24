# Bản đồ Học tập MuJoCo (MuJoCo Learning Roadmap)

Danh mục này liệt kê 50 bài thực hành từ cơ bản đến nâng cao. Lộ trình được thiết kế theo phương pháp **"Học qua dự án nhỏ" (Project-based learning)**, giúp người học dần làm chủ trình mô phỏng MuJoCo, từ việc định nghĩa một vật rắn đơn giản đến thao tác điều khiển robot phức hợp thông qua Python API. Sự tập trung đặc biệt được đặt vào **Underactuated Robotics** và thu hẹp khoảng cách **Sim-to-Real**.

---

## Chặng 1: Nền tảng Không gian và Vật rắn (Rigid Body Fundamentals)
*Mục tiêu: Hiểu cách MuJoCo định nghĩa không gian 3D, cấu trúc phân cấp (cha - con), và tách bạch hoàn toàn giữa hiển thị thị giác (Visual) và tính toán vật lý (Inertial).*

| Bài | Chủ đề | Tags Trọng tâm | Mục tiêu Kiến thức |
| :--- | :--- | :--- | :--- |
| **01** | Empty world | `<mujoco>` | Khởi tạo cấu trúc file XML chuẩn, hệ quy chiếu toàn cục. |
| **02** | Static object (box) | `<geom>` | Định nghĩa hình học cơ sở, phân biệt body và geom. |
| **03** | Ground and gravity | `<option>`, `<size>` | Thiết lập mặt phẳng tham chiếu, bộ giải (integrator). |
| **04** | Body hierarchy | `<body>`, `freejoint` | Xây dựng cây động học (kinematic tree), bậc tự do gốc. |
| **05** | Inertial properties | `<inertial>` | Ghi đè tự động nội suy; ma trận quán tính, khối lượng điểm. |
| **06** | Mesh loading | `<mesh>` | Tách biệt Visual/Collision mesh, Auto-Convex Hull. |
| **07** | Material and texture | `<material>` | Khái niệm PBR, UV mapping phục vụ Vision RL. |
| **08** | Frame & orientation | `quat`, `euler` | Thao tác không gian bằng Quaternion, tránh Gimbal Lock. |
| **09** | Camera setup | `<camera>` | Cấu hình camera Ego-centric và Allo-centric. |
| **10** | Light sources | `<light>` | Xử lý đổ bóng, định hướng nguồn sáng. |

---

## Chặng 2: Động lực học & Điều khiển Hệ Underactuated (Dynamics & Actuation)
*Mục tiêu: Thiết lập không gian trạng thái (State-space), mô hình hóa toán học các cơ cấu chấp hành và khảo sát các hệ thống thiếu cơ cấu chấp hành (Underactuated systems).*

| Bài | Chủ đề | Tags Trọng tâm | Mục tiêu Kiến thức |
| :--- | :--- | :--- | :--- |
| **11** | Simple Pendulum | `<joint type="hinge">` | Không gian pha (Phase space), Energy-shaping control. |
| **12** | Cartpole | `type="slide"` | Khớp trượt vô hạn, Tương tác quán tính (Inertial coupling), LQR. |
| **13** | Double Pendulum | `<body>` lồng nhau | Động lực học chuỗi mở (Open-chain), Hiệu ứng hỗn loạn (Chaos). |
| **14** | Acrobot & Pendubot | `ctrlrange` | Hệ 2-DOF Underactuated, bài toán Swing-up phi tuyến. |
| **15** | Slider-Crank | `<equality>` | Đóng vòng động học (Kinematic loops), Ràng buộc đẳng thức. |
| **16** | Passive Dynamics | `springref` | Khảo sát lò xo xoắn, hệ số cản (damping), dao động tắt dần. |
| **17** | Joint Limits | `limited="true"` | Ràng buộc bất đẳng thức, biên độ vật lý của khớp. |
| **18** | Position Actuator | `type="position"` | Phân tích bộ điều khiển PD ẩn nội bộ của MuJoCo. |
| **19** | Velocity Actuator | `type="velocity"` | Bộ bám vận tốc, triệt tiêu sai số tích phân. |
| **20** | Torque Actuator | `type="motor"` | Ánh xạ lực thuần túy (Direct torque control). |
| **21** | Muscle Actuator | `type="muscle"` | Khảo sát đặc tính sinh lý học và độ cứng phi tuyến của cơ bắp. |
| **22** | Tendon Dynamics (Fixed) | `<tendon>` | Truyền động gián tiếp qua dây cáp cố định chiều dài. |
| **23** | Tendon Dynamics (Spatial)| `<spatial>` | Ròng rọc không gian (Spatial pulleys), định tuyến cáp 3D. |
| **24** | Multi-actuator Mapping | `gainprm`, `bias` | Ánh xạ tín hiệu điều khiển ảo thành lực cơ học thực tế. |

---

## Chặng 3: Cơ học Tiếp xúc & Cảm biến (Contact Mechanics & Perception)
*Mục tiêu: Xử lý tương tác vật lý khắt khe (va chạm, ma sát) và xây dựng pipeline trích xuất dữ liệu cảm biến (Observation space).*

| Bài | Chủ đề | Tags Trọng tâm | Mục tiêu Kiến thức |
| :--- | :--- | :--- | :--- |
| **25** | Bouncing Ball | `solref`, `solimp` | Bản chất của Contact Solver: Độ cứng và Damping tiếp xúc. |
| **26** | Contact Friction | `condim`, `friction` | Khảo sát ma sát trượt (sliding), xoay (torsional) và lăn (rolling). |
| **27** | Joint Friction & Loss | `armature`, `frictionloss`| Quán tính rotor, ma sát tĩnh/động tại khớp (Sim-to-Real gap). |
| **28** | Gyroscopic Effects | `inertia`, `spin` | Phân tích con quay hồi chuyển, vật thể có trọng tâm lệch. |
| **29** | Collision Filtering | `contype`, `conaffinity`| Tối ưu hóa tính toán bằng bitmask, loại trừ va chạm nội bộ. |
| **30** | Soft Constraints | `solref` trong `<equality>`| Làm mềm các ràng buộc cứng để tránh suy biến ma trận. |
| **31** | Kinematic Sensors | `<jointpos>`, `<jointvel>`| Bypass API cấp thấp, trích xuất vector trạng thái chuẩn hóa. |
| **32** | IMU Sensor | `<accelerometer>`, `<gyro>`| Mô phỏng nhiễu Gaussian và trôi dạt (drift) của IMU. |
| **33** | Force/Torque Sensor | `<force>`, `<torque>` | Đo lường nội lực và momen tại điểm liên kết (site). |
| **34** | Tactile Sensing | `<touch>` | Mô phỏng mảng cảm biến xúc giác trên bề mặt vật thể. |
| **35** | Forceplate (GRF) | `<force>` toàn cục | Trích xuất Phản lực mặt đất (Ground Reaction Force). |
| **36** | Vision: RGB Rendering | `mujoco.Renderer` | Trích xuất mảng pixel ảnh màu phục vụ CNN/Vision-RL. |
| **37** | Vision: Depth & Seg | `enable_depth` | Trích xuất bản đồ chiều sâu (Depth map) và Segmentation. |

---

## Chặng 4: Tích hợp Hệ thống & Triển khai (System Integration & Sim-to-Real)
*Mục tiêu: Đưa mô hình vào thực tiễn, đóng gói môi trường Gym/RL, xử lý nhiễu hệ thống và triển khai luật điều khiển trên các cấu trúc Robot phức hợp.*

| Bài | Chủ đề | Tags Trọng tâm | Mục tiêu Kiến thức |
| :--- | :--- | :--- | :--- |
| **38** | Actuator Noise & Delay | `noise`, `timeconst` | Giả lập độ trễ giao tiếp và nhiễu phần cứng (Sim-to-Real). |
| **39** | External Disturbances | `appliedforce` | Tiêm nhiễu ngoại lực (gió, va đập) bằng Python API. |
| **40** | User-defined Dynamics | `actdyn`, `plugin` | Tự định nghĩa phương trình vi phân cho Actuator tùy chỉnh. |
| **41** | Mobile Base: Diff-Drive | `slide`, `hinge` | Tích hợp: Xe tự hành 2 bánh dẫn động vi sai. |
| **42** | Mobile Base: Omni | Bánh Mecanum | Tích hợp: Động học robot di chuyển đa hướng. |
| **43** | Planar Manipulator | 2-DOF Arm | Tích hợp: Động học ngược (IK) và Jacobian cho tay máy phẳng. |
| **44** | Spatial Manipulator | 6-DOF Arm | Tích hợp: Điều khiển không gian tác động (Task-space control). |
| **45** | Gripper Mechanics | `<equality>` | Thiết kế cơ cấu kẹp song song (Parallel jaw gripper). |
| **46** | Object Manipulation | `site`, `touch` | Bài toán gắp thả (Pick and Place) và ma sát kẹp giữ. |
| **47** | Legged: 1D Hopper | Hệ Underactuated | Tích hợp: Robot nhảy lò cò 1 chân, điều khiển chu kỳ chạm đất. |
| **48** | Legged: Bipedal Walker| `<tendon>`, `<actuator>`| Tích hợp: Động lực học đi bộ hai chân cơ sở. |
| **49** | Custom C++ Plugins | `<plugin>` | Mở rộng MuJoCo: Tích hợp thuật toán thủy động lực học/khí động học. |
| **50** | Policy Deployment | Toàn bộ API | Đồ án: Đóng gói môi trường Gym, chạy mạng Neural Network / RL Agent. |