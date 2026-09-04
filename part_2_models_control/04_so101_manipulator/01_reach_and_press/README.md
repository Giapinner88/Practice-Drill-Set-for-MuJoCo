# Task 01 — Reach and press

## Câu hỏi trung tâm

Làm thế nào đưa đầu công tác của một manipulator đến target Cartesian rồi xác nhận thao tác nhấn bằng cả chuyển động của nút và lực tiếp xúc?

## Sau bài này, bạn có thể

- đọc pose của một `site` trong world frame;
- lấy geometric Jacobian bằng `mj_jacSite`;
- giải inverse kinematics bằng damped least squares;
- ánh xạ vận tốc Cartesian thành position reference cho actuator;
- dùng slide joint để mô hình hóa nút có lò xo;
- đọc joint sensor và touch sensor từ `sensordata`;
- đánh giá task bằng sai số, dwell time, force, joint margin và Jacobian conditioning.

## Model SO-101

Robot và 13 mesh STL được nhập từ `Course_1_SO101_Basics` tại commit `bc600307bf7b0406385d2899d3fe28d0a5525d15` của nhánh `course-1`. Cây động học, inertia, giới hạn khớp, actuator và keyframe của robot được giữ nguyên. Bài này thêm một collision sphere nhỏ tại đầu công tác cùng button station, slide joint và sensor phục vụ task. Button station đặt tại $(x,y,z)=(0.20,0.15,0)\ \mathrm{m}$ trong world frame, lệch sang bên trái robot để thao tác cần phối hợp nhiều khớp.

```text
robot joints : 5 arm joints + 1 gripper joint
task joint   : 1 button slide joint
nq = nv      : 7
nu           : 6 position actuators của SO-101
timestep     : 0.002 s
```

Keyframe `home` lưu cả `qpos` và `ctrl`. `mj_resetDataKeyframe` nạp hai vector cùng lúc để position actuator không kéo robot đột ngột về reference khác.

## Các site và sensor

| Tên | Vai trò |
| --- | --- |
| `gripperframe` | Điểm end-effector dùng để tính pose và Jacobian |
| `approach_target` | Target đầu tiên, nằm phía trên nút |
| `press_target` | Target thứ hai, tạo độ lún mong muốn |
| `button_touch` | Thể tích nhận contact normal force |
| `button_depth` | Joint-position sensor của `button_slide` |
| `button_force` | Touch sensor trên button cap |

`data.site_xpos` trả về vị trí site trong world frame. Jacobian tịnh tiến của `gripperframe` được lấy bằng:

```python
mujoco.mj_jacSite(model, data, jacobian_position, jacobian_rotation, site_id)
```

`mj_jacSite` trả về Jacobian theo toàn bộ `nv`. Controller chỉ chọn năm cột ứng với arm joints; cột của gripper và button không thuộc biến điều khiển IK.

## Damped-least-squares IK

Với sai số Cartesian $e=p_d-p$, vận tốc task mong muốn là

$$
v_d=\operatorname{clip}(k e, v_{max}).
$$

Joint velocity được tính bằng

$$
\dot q=J^T\left(JJ^T+\lambda^2I\right)^{-1}v_d.
$$

Bài dùng $k=4$, $v_{max}=0.08\ \mathrm{m/s}$, $\lambda=0.03$ và giới hạn $|\dot q_i|\leq1.2\ \mathrm{rad/s}$. Damping giữ phép giải hữu hạn khi Jacobian gần suy biến. Position reference được tích phân rồi chặn trong `actuator_ctrlrange`:

$$
q^d_{k+1}=\operatorname{clip}\left(q^d_k+\dot q_k\Delta t, q_{min},q_{max}\right).
$$

Đây là position-only IK; orientation của gripper không nằm trong objective.

## Hai pha của task

1. **Reach:** `gripperframe` phải ở trong bán kính `6 mm` quanh `approach_target` liên tục `0.15 s`.
2. **Press:** button phải lún hơn `6 mm` và touch force lớn hơn `0.25 N` liên tục `0.10 s`.

Yêu cầu đồng thời depth và force giúp phân biệt “đã ra lệnh đi xuống” với contact thực sự. Button dùng slide joint có range `0–15 mm`, stiffness `120 N/m` và damping `2 N*s/m`.

## Chạy bài

### Tự điều khiển bằng slider

Mở model ở keyframe `home`, nhấn `Tab` để hiện bảng điều khiển rồi kéo sáu slider trong mục **Control**. Script này không ghi đè `data.ctrl`, vì vậy actuator giữ đúng giá trị bạn đặt trên viewer.

```bash
python part_2_models_control/04_so101_manipulator/01_reach_and_press/simulate.py
```

Hãy thử lần lượt shoulder pan, shoulder lift, elbow flex và wrist flex; quan sát dấu của chuyển động trước khi phối hợp các khớp để chạm nút. Viewer chạy đến khi bạn đóng cửa sổ.

### Chạy demo end-to-end

`demo.py` tự thực hiện hai pha reach và press bằng controller trong `controller.py`:

```bash
python part_2_models_control/04_so101_manipulator/01_reach_and_press/demo.py
python part_2_models_control/04_so101_manipulator/01_reach_and_press/demo.py --headless --duration 4
```

Demo headless tạo `artifacts/reach_press_diagnostics.png` và in các metric của trajectory.

Với model và gain mặc định, trajectory tham chiếu đạt success tại khoảng $3.246\ \mathrm{s}$, button lún tối đa khoảng $7.057\ \mathrm{mm}$, touch force đạt khoảng $0.847\ \mathrm{N}$ và không bão hòa actuator. Joint-limit margin nhỏ nhất khoảng $0.623\ \mathrm{rad}$; singular value nhỏ nhất của Jacobian tịnh tiến khoảng $0.0672$ và condition number lớn nhất khoảng $5.55$.

## Đọc kết quả

- `distance` là sai số đến target đang active; nó nhảy khi task chuyển từ reach sang press.
- `button_depth` phải vượt `6 mm` nhưng không nên vượt joint range `15 mm`.
- `button_force` chỉ khác không khi contact thuộc vùng `button_touch`.
- `minimum_joint_margin` âm nghĩa là state đã vượt joint range mềm.
- `minimum_jacobian_singular_value` tiến về không báo hiệu cấu hình gần singular.
- Một trajectory đẹp nhưng không đạt cả depth, force và dwell time vẫn chưa hoàn thành task.

## Bài tập

1. Đặt `DAMPING = 0` và đưa target đến gần biên workspace; quan sát condition number và joint velocity.
2. Bỏ giới hạn task speed hoặc joint speed, sau đó đo actuator saturation.
3. Dùng thêm `jacobian_rotation` để giữ một trục của gripper thẳng đứng khi nhấn.
4. Đổi stiffness của button và kiểm tra quan hệ giữa depth với touch force.
5. Dời `button_station`, chạy lại headless và xác định target nào nằm ngoài workspace.
6. Sau khi press thành công, thêm pha retreat và kiểm tra button trở về nhờ spring force.

## Tài liệu

- [MuJoCo API — `mj_jacSite`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)
- [MJCF reference — joint và sensor](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
