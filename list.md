
## Cấu trúc XML cơ bản 

| Bài | Chủ đề               | Tags học                   | Nội dung chính                              |
| --- | -------------------- | -------------------------- | ------------------------------------------- |
| 1   | Empty world          | `<mujoco>`                 | Tạo file trống và render nền                |
| 2   | Static object (box)  | `<geom>`                   | Tạo vật thể tĩnh hình hộp                   |
| 3   | Ground and gravity   | `<option>`, `<size>`       | Tạo mặt đất, chỉnh trọng lực                |
| 4   | Body hierarchy       | `<body>`                   | Nested body: cha – con                      |
| 5   | Mesh loading         | `<mesh>`                   | Tải mesh `.stl` đơn giản                    |
| 6   | Material and texture | `<material>`, `<texture>`  | Gán vật liệu                                |
| 7   | Frame & orientation  | `pos`, `quat`, `axisangle` | Quay và đặt vị trí                          |
| 8   | Camera               | `<camera>`                 | Thêm camera thủ công                        |
| 9   | Light                | `<light>`                  | Tạo nguồn sáng tùy chỉnh                    |
| 10  | Inertial             | `<inertial>`               | Set khối lượng, khối tâm, ma trận quán tính |

## Động học và động lực học cơ bản

| Bài | Chủ đề                     | Tags học                    | Nội dung chính                     |
| --- | -------------------------- | --------------------------- | ---------------------------------- |
| 11  | Simple pendulum            | `<joint>`                   | Tạo con lắc đơn                    |
| 12  | Double pendulum            | `<joint>`, `<body>`         | Xoay và kiểm tra mô phỏng liên kết |
| 13  | Slider-crank               | `type="slide"`              | Học liên kết tuyến tính            |
| 14  | Rotational spring          | `<actuator>`                | Gắn lò xo xoắn                     |
| 15  | Linear spring-damper       | `<actuator>`, `springref`   | Damping tuyến tính                 |
| 16  | Constraints: hinge         | `<joint>`, `limited="true"` | Giới hạn góc quay                  |
| 17  | Contact model: box–plane   | `solref`, `solimp`          | Thử nghiệm va chạm                 |
| 18  | Joint friction             | `friction`                  | Điều chỉnh lực ma sát khớp         |
| 19  | Gyroscopic effects         | `inertia`, `spin`           | Mô hình con quay                   |
| 20  | Torque control             | `ctrlrange`                 | Áp moment điều khiển               |
| 21  | Velocity control           | `velocity`                  | Điều khiển theo vận tốc            |
| 22  | Mass vs. acceleration      | `mass`, `gravity`           | Thí nghiệm định luật II Newton     |
| 23  | Non-uniform density object | `inertial`                  | Tạo vật thể lệch tâm               |
| 24  | Collision filtering        | `contype`, `conaffinity`    | Va chạm chọn lọc                   |
| 25  | Sensors (touch, force)     | `<sensor>`                  | Gắn cảm biến vào geom hoặc joint   |

## Tương tác, điều khiển và actuator

| Bài | Chủ đề                      | Tags học               | Nội dung chính                     |
| --- | --------------------------- | ---------------------- | ---------------------------------- |
| 26  | Position actuator           | `type="position"`      | Actuator điều khiển vị trí         |
| 27  | Velocity actuator           | `type="velocity"`      | Điều khiển theo vận tốc            |
| 28  | Muscle actuator             | `type="muscle"`        | Cơ bắp, đặc tính co rút            |
| 29  | Gain bias actuator          | `bias`, `gainprm`      | Mapping control input to force     |
| 30  | Tactile sensor + touch geom | `<sensor>`             | Mô phỏng tiếp xúc da/cảm ứng       |
| 31  | Touch-triggered force       | `site`, `touch`        | Tạo phản ứng khi chạm              |
| 32  | Tendons                     | `<tendon>`             | Dây kéo cơ học                     |
| 33  | Soft constraints            | `solref`, `solimp`     | Chỉnh độ mềm contact               |
| 34  | Actuator noise & delay      | `noise`, `timeconst`   | Giả lập tín hiệu điều khiển        |
| 35  | Contact dynamics tuning     | `stiffness`, `damping` | Fine-tune vật lý tiếp xúc          |
| 36  | Multiple actuators          | Nhiều `<actuator>`     | Multi-DOF control                  |
| 37  | Load sensors (forceplate)   | `force`, `torque`      | Nhúng cảm biến tải trọng           |
| 38  | Wind or external force      | `appliedforce`         | Mô phỏng gió hoặc lực từ bên ngoài |
| 39  | User-defined actuators      | `plugin`, `actdyn`     | Mở rộng actuator                   |
| 40  | Actuator control via Python | `mujoco.mj_step`       | Kết nối với Python để điều khiển   |


## Robot & hệ thống phức hợp

| Bài | Chủ đề                         | Tags học                          | Nội dung chính                      |
| --- | ------------------------------ | --------------------------------- | ----------------------------------- |
| 41  | 2-joint arm                    | `<body>`, `<joint>`, `<actuator>` | Cánh tay 2 bậc tự do                |
| 42  | 3D manipulator                 | 6 DOF                             | Rô-bốt 3D cơ bản                    |
| 43  | Mobile robot (2 wheels)        | `slide`, `hinge`                  | Xe di động cơ bản                   |
| 44  | Differential drive             | `gear`, `joint`                   | Xe robot 2 bánh dẫn động vi sai     |
| 45  | Omnidirectional base           | 3 bánh Mecanum                    | Điều khiển chuyển động phức hợp     |
| 46  | Walking biped (simple)         | `<tendon>`, `<actuator>`          | Người máy đi bộ                     |
| 47  | Gripper                        | `site`, `touch`                   | Gắp và cầm nắm vật thể              |
| 48  | Visual & depth camera          | `camera`, `sensor`                | Mô phỏng thị giác                   |
| 49  | Plugin simulation (e.g. water) | `plugin`                          | Mô phỏng môi trường mở rộng         |
| 50  | Tổng hợp hệ robot + điều khiển | Toàn bộ                           | Robot điều khiển bằng policy Python |
