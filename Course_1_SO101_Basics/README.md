# Course 1 — SO-101: Căn bản MuJoCo

Bài học tối giản nhất về MuJoCo, dùng cánh tay robot **SO-101** (5 DOF + gripper) làm ví dụ.

```
Course_1_SO101_Basics/
├── model.xml      # Toàn bộ mô tả robot + scene, một file duy nhất
├── simulate.py    # Vòng lặp mô phỏng + viewer
├── README.md
└── assets/        # 13 file mesh STL của SO-101
```

## Chạy thử

```bash
python simulate.py
```

Cần `pip install mujoco`. Nhấn **ESC** tại cửa sổ viewer để thoát.

---

## 1. Hai đối tượng cốt lõi: `mjModel` và `mjData`

Đây là khái niệm quan trọng nhất khi học MuJoCo. Mọi thứ khác đều xoay quanh nó.

| | `mjModel` | `mjData` |
|---|---|---|
| Bản chất | Phần **tĩnh** | Phần **động** |
| Chứa gì | Cây động học, khối lượng, quán tính, mesh, giới hạn khớp, actuator | `qpos`, `qvel`, `ctrl`, lực, `time` |
| Thay đổi khi chạy? | Không | Có, mỗi timestep |
| Tạo ra bằng | `MjModel.from_xml_path("model.xml")` | `MjData(model)` |

```python
model = mujoco.MjModel.from_xml_path("model.xml")   # biên dịch XML một lần
data  = mujoco.MjData(model)                        # cấp phát bộ nhớ trạng thái
```

Ẩn dụ dễ nhớ: `mjModel` là **bản thiết kế** robot, `mjData` là **robot thật đang chuyển động** tại một thời điểm.

## 2. Không gian trạng thái của SO-101

Chạy `simulate.py`, chương trình in ra:

```
nq (số toạ độ vị trí) = 6
nv (số bậc tự do)     = 6
nu (số actuator)      = 6
```

* **`nq`** — kích thước `data.qpos`, vector vị trí khớp.
* **`nv`** — kích thước `data.qvel`, vector vận tốc khớp.
* **`nu`** — kích thước `data.ctrl`, vector tín hiệu điều khiển.

Ở đây `nq == nv` vì SO-101 chỉ gồm khớp bản lề (`hinge`), mỗi khớp đúng 1 bậc tự do. Điều này **không phải lúc nào cũng đúng**: một `freejoint` chiếm 7 ô trong `qpos` (3 vị trí + 4 quaternion) nhưng chỉ 6 ô trong `qvel` (3 tịnh tiến + 3 vận tốc góc). Quaternion cần 4 số để biểu diễn nhưng chỉ có 3 bậc tự do quay.

## 3. Chuỗi động học (kinematic chain)

`model.xml` mô tả robot bằng các thẻ `<body>` **lồng nhau** — con kế thừa chuyển động của cha:

```
base
└── shoulder          (joint: shoulder_pan   — xoay đế quanh Z)
    └── upper_arm     (joint: shoulder_lift  — nâng/hạ vai)
        └── lower_arm (joint: elbow_flex     — gập khuỷu)
            └── wrist (joint: wrist_flex     — gập cổ tay)
                └── gripper           (joint: wrist_roll — xoay cổ tay)
                    └── moving_jaw    (joint: gripper    — đóng/mở kẹp)
```

Xoay `shoulder_pan` thì **toàn bộ** cánh tay xoay theo, vì mọi body phía dưới đều là con cháu của `shoulder`. Vị trí `pos` và hướng `quat` của mỗi `<body>` là **tương đối so với body cha**, không phải toạ độ thế giới.

Mỗi khớp có giới hạn vật lý, khai báo qua thuộc tính `range` (đơn vị radian, do `<compiler angle="radian"/>`):

| Khớp | Giới hạn (rad) |
|---|---|
| `shoulder_pan` | ±1.92 |
| `shoulder_lift` | ±1.75 |
| `elbow_flex` | ±1.69 |
| `wrist_flex` | ±1.66 |
| `wrist_roll` | −2.74 … +2.84 |
| `gripper` | −0.17 … +1.75 |

## 4. Geom: `visual` và `collision`

Mỗi bộ phận trong `model.xml` xuất hiện **hai lần**, phân biệt bằng `class`:

```xml
<geom type="mesh" class="visual"    mesh="upper_arm_so101_v1" .../>
<geom type="mesh" class="collision" mesh="upper_arm_so101_v1" .../>
```

Xem khai báo trong `<default>`:

* **`visual`** — `contype="0" conaffinity="0"`, tức là **không tham gia va chạm**, chỉ để hiển thị đẹp. Thuộc `group="2"`.
* **`collision`** — tham gia tính toán va chạm. Thuộc `group="3"`, mặc định bị ẩn trong viewer.

Tách đôi như vậy là chuẩn mực trong robotics: hình học hiển thị có thể rất chi tiết, còn hình học va chạm nên đơn giản để mô phỏng chạy nhanh. Trong viewer, nhấn phím **`0`–`4`** để bật/tắt từng group và nhìn thấy lớp collision.

## 5. Actuator vị trí và `data.ctrl`

SO-101 dùng servo STS3215, mô hình hoá bằng actuator loại `<position>`:

```xml
<position class="sts3215" name="elbow_flex" joint="elbow_flex"
          forcerange="-3.35 3.35" ctrlrange="-1.69 1.69"/>
```

Với actuator loại này, `data.ctrl[i]` **không phải moment xoắn** mà là **vị trí mục tiêu** (radian). MuJoCo tự chạy bộ điều khiển PD bên trong:

$$\tau = k_p (\text{ctrl} - q) - k_v \dot{q}$$

trong đó `kp = 998.22`, `kv = 2.731` (khai báo ở class `sts3215`), và moment kết quả bị chặn bởi `forcerange`.

Hệ quả thực tế: robot **không** bám lệnh tuyệt đối. Chạy `simulate.py`, các khớp gánh trọng lực (`shoulder_pan`, `shoulder_lift`, `elbow_flex`) luôn có sai số bám (tracking error) khác 0 — đó chính là độ võng tĩnh của bộ điều khiển PD, hoàn toàn đúng vật lý.

## 6. Vòng lặp mô phỏng

```python
data.qpos[:] = HOME_QPOS
mujoco.mj_forward(model, data)      # tính lại đại lượng dẫn xuất, KHÔNG tiến thời gian

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        data.ctrl[i] = ...          # ra lệnh
        mujoco.mj_step(model, data) # tiến đúng 1 timestep (2 ms)
        viewer.sync()               # đẩy trạng thái lên đồ hoạ
```

Ba hàm cần phân biệt rõ:

| Hàm | Tác dụng |
|---|---|
| `mj_forward` | Tính vị trí body, ma trận quán tính, lực… từ `qpos`/`qvel` hiện tại. **Không** tiến thời gian. Dùng sau khi đặt tư thế thủ công. |
| `mj_step` | Tích phân hệ đi một timestep. `data.time` tăng lên. |
| `viewer.sync` | Chỉ vẽ lại. Không ảnh hưởng vật lý. |

`launch_passive` nghĩa là **bạn** giữ quyền điều khiển vòng lặp (viewer chỉ hiển thị thụ động). Đối lập với `launch`, khi MuJoCo tự chạy vòng lặp riêng của nó.

Đoạn `time.sleep` cuối vòng lặp giữ nhịp **thời gian thực**: một bước mô phỏng 2 ms cũng tốn đúng 2 ms ngoài đời. Bỏ nó đi thì mô phỏng chạy nhanh hết mức CPU cho phép — điều bạn muốn khi huấn luyện RL, nhưng không muốn khi đang quan sát bằng mắt.

---

## Bài tập

**Bài 1 — Đọc trạng thái.** In `data.qpos` và `data.time` mỗi 100 bước. Xác nhận `data.time` tăng đúng bội số của `model.opt.timestep`.

**Bài 2 — Sai số bám.** In `data.ctrl[i] - data.qpos[i]` cho cả 6 khớp. Khớp nào sai số lớn nhất? Giải thích tại sao bằng lập luận trọng lực và moment tải.

**Bài 3 — Độ cứng bộ điều khiển.** Sửa `kp` của class `sts3215` trong `model.xml` từ `998.22` xuống `100`, rồi lên `5000`. Quan sát: `kp` nhỏ thì cánh tay võng và trễ; `kp` lớn thì bám sát hơn nhưng có thể dao động hoặc mất ổn định số.

**Bài 4 — Giới hạn khớp.** Bỏ đoạn kẹp `min(max(target, lo), hi)` trong `simulate.py` và tăng biên độ sin lên `2.0`. Khớp có vượt quá `range` khai báo trong `model.xml` không? MuJoCo xử lý giới hạn khớp như một ràng buộc mềm (soft constraint), không phải tường cứng tuyệt đối.

**Bài 5 — Trọng lực.** Đổi `<option gravity="0 0 -9.81"/>` (thêm vào `model.xml`) thành `0 0 0`. Sai số bám ở Bài 2 thay đổi thế nào? Kết quả này chứng minh điều gì về nguồn gốc của sai số?

**Bài 6 — Tắt viewer.** Bỏ `viewer` và `time.sleep`, chạy 10000 bước rồi đo thời gian thực bằng `time.perf_counter()`. Mô phỏng nhanh gấp bao nhiêu lần thời gian thực? Đây là lý do việc huấn luyện RL trong mô phỏng khả thi.

---

## Nguồn

Model và mesh lấy từ [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (`Simulation/SO101`, bản hiệu chuẩn `so101_new_calib`). File `model.xml` ở đây là bản gộp `scene.xml` + `so101_new_calib.xml` + `joints_properties.xml` thành một file duy nhất cho gọn.
