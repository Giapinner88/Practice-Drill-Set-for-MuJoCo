# Buổi 2 — Cart-pole: Động lực học trong MuJoCo

> **Giáo án cho người trình bày.** File này là kịch bản để chuẩn bị buổi dạy, không phải tài liệu phát cho học viên.

**Thời lượng:** 120 phút · **Yêu cầu trước:** Buổi 1 (SO-101 cơ bản)

Buổi 1 trả lời *"làm sao mô tả một hệ?"*. Buổi 2 trả lời câu tiếp theo: *"MuJoCo mô phỏng hệ đó thế nào, và tham số nào quyết định kết quả?"*

Cả buổi chỉ dùng **một mô hình — cart-pole**. Hai bậc tự do, một actuator: đủ đơn giản để mỗi lần sửa một tham số là thấy ngay khác biệt, mà vẫn đủ giàu để có coupling thật.

Cách dạy xuyên suốt: **sửa `model.xml`, chạy lại, nhìn viewer**. Không có script phân tích nào — học viên tune bằng tay, và chính việc phải tự đoán trước rồi kiểm chứng mới là phần dạy được.

```
Course_2_Cartpole_Dynamics/
├── model.xml              # cart-pole, chú thích từng tham số — file để tune
├── model_broken.xml       # bản cố tình sai 3 lỗi — dùng ở phần debug
├── simulate.py            # chạy + xem, giống hệt form của buổi 1
├── make_slides_media.py   # sinh hình + video cho slide (chạy khi soạn bài)
├── media/                 # kết quả: 10 PNG + 5 MP4, bỏ thẳng vào slide
└── README.md              # giáo án này
```

```bash
python simulate.py            # tune tham số trong model.xml rồi chạy lại
python simulate.py --broken   # model lỗi, dùng ở phần 110–117 phút

python make_slides_media.py           # sinh lại toàn bộ media
python make_slides_media.py damping   # chỉ sinh lại một mục
python make_slides_media.py --list    # xem danh sách mục
```

Cần `pip install mujoco matplotlib`. Muốn xuất video thì cần thêm `ffmpeg` trong PATH — không có cũng chạy được, chỉ bỏ qua phần `.mp4`.

Nhấn **ESC** để thoát viewer.

---

## Media cho slide

Chạy `make_slides_media.py` một lần, lấy file trong `media/` chèn vào slide. Cột cuối là mốc thời gian nên dùng.

| File | Nội dung | Dùng ở phút |
|---|---|---:|
| `00_overview.mp4` | cart-pole tự đổ — mở đầu buổi | 10–15 |
| `01_coupling.mp4` · `.png` | đẩy cart, pole lắc theo dù không có actuator | 15–20 |
| `02_gravity.png` | ba mức trọng lực, g = 0 pole đứng yên | 95–110 |
| `02_gravity_zero.mp4` | pole treo bất động trong môi trường g = 0 | 95–110 |
| `03_mass.png` | pole nặng hơn → cart tăng tốc chậm hơn | 40–45 |
| `04_inertia.png` | pole dài gấp đôi đổ chậm hơn, kèm số inertia | 50–55 |
| `05_damping.png` | bốn mức damping, có chú thích overdamped | 60–65 |
| `05_damping_0.mp4` | damping = 0: đung đưa mãi | 60–65 |
| `05_damping_1_0.mp4` | damping = 1.0: bò chậm, không dao động | 60–65 |
| `06_ctrlrange.png` | ctrl = 50 trùng khít ctrl = 10 | 75–80 |
| `07_timestep.png` | quỹ đạo lệch dần + sai số RMS thang log | 95–110 |
| `08_broken_vs_correct.png` | đúng vs sai, đã bật Joint + Com | 110–117 |
| `09_state_sequence.png` | chuỗi ảnh pole đổ, minh hoạ qpos | 80–90 |

Ảnh 1280×720, video 60 fps — vừa khung slide 16:9.

> **Khi sửa `model.xml`:** script sinh media tạo biến thể bằng cách thay chuỗi trong XML gốc. Nếu bạn đổi cách viết các thuộc tính đó, script sẽ **báo lỗi ngay** thay vì âm thầm sinh ra hình trong đó mọi đường trùng khít nhau. Gặp lỗi `variant(...): khớp 0 chỗ` thì mở `variant()` cập nhật lại pattern.

---

## Mục tiêu buổi học

Sau buổi này, người học cần:

- Hiểu cart-pole dưới góc nhìn một hệ cơ học.
- Dựng được một cart-pole bằng MJCF từ con số không.
- Hiểu các tham số vật lý chính: `mass`, `inertia`, `damping`, `frictionloss`, `gravity`, `timestep`.
- Thiết lập được joint và actuator.
- Biết `qpos`, `qvel`, `ctrl` chứa gì.
- Quan sát được ảnh hưởng của việc đổi tham số lên dynamics.
- Dùng được Viewer để debug model.

Mental model chính:

```text
MJCF parameters → MuJoCo model → State + Input → Physics → Motion
```

---

# TIMELINE

| Thời gian | Nội dung | Hoạt động | Media |
|---:|---|---|---|
| 0–10 | Recap buổi 1 | giảng | — |
| 10–25 | Hiểu hệ cart-pole | giảng | `00_overview` · `01_coupling` |
| 25–40 | Dựng cart-pole bằng MJCF | đọc `model.xml` | — |
| 40–55 | Mass, inertia và geometry | **tune tay** | `03_mass` · `04_inertia` |
| 55–70 | Joint dynamics | **tune tay** | `05_damping` |
| 70–80 | Actuator setup | **tune tay** | `06_ctrlrange` |
| 80–95 | State inspection | giảng + viewer | `09_state_sequence` |
| 95–110 | Hands-on experiments | **tune tay** | `02_gravity` · `07_timestep` |
| 110–117 | Viewer debugging | `--broken` | `08_broken_vs_correct` |
| 117–120 | Tổng kết | giảng | — |

---

## 0–10 phút — Recap buổi 1

Nhắc lại chuỗi đã dựng ở buổi trước:

```text
MJCF → MjModel → MjData → qpos / qvel / ctrl → mj_step()
```

Buổi trước ta coi `mujoco.mj_step(model, data)` như một **black box**. Buổi này tập trung đúng một câu:

> Những tham số nào quyết định kết quả bên trong black box đó?

**Chốt:** cart-pole là model tối thiểu để thấy rõ điều đó — chỉ 2 DOF nên mọi thay đổi đều nhìn thấy được ngay.

> 🎬 `media/00_overview.mp4` — chiếu ngay lúc này để lớp thấy hệ sẽ làm việc suốt buổi.

---

## 10–25 phút — Hiểu hệ cart-pole

### 10–15 phút — Cấu tạo

```text
World
 └── Cart      (trượt theo trục x)
      └── Pole (quay quanh trục y)
```

$$
q = \begin{bmatrix} x \\ \theta \end{bmatrix}
\qquad
x_s = \begin{bmatrix} x \\ \theta \\ \dot x \\ \dot\theta \end{bmatrix}
$$

Ở mức cần cho MuJoCo, chỉ cần nhớ:

```text
qpos → configuration (hệ đang ở đâu)
qvel → motion state  (đang chuyển động thế nào)
```

### 15–20 phút — Input và coupling

Cart được tác động bằng một lực duy nhất, $u = F$:

```text
force → cart moves → pole reacts
```

**Điểm phải nhấn:** pole **không có actuator nào**. Nó chuyển động hoàn toàn do quán tính và trọng lực, gián tiếp qua cart. Hệ 2 DOF nhưng chỉ 1 input gọi là **underactuated** — đây là lý do cart-pole thành bài toán kinh điển của điều khiển, và là cầu nối sang phần LQR ở part 2.

> 🎬 `media/01_coupling.mp4` + 📊 `media/01_coupling.png` — biểu đồ cho thấy lực tắt ở 0.35 s, cart dừng hẳn lúc chạm biên, **mà pole vẫn lắc tiếp**. Đó là bằng chứng trực quan nhất cho coupling.

### 20–25 phút — Dynamics ở mức cần thiết

$$
M(q)\ddot q + C(q,\dot q)\dot q + g(q) = Bu
$$

**Không derive.** Chỉ cần học viên đọc được từng số hạng ứng với tham số nào trong MJCF:

| Số hạng | Ý nghĩa | Ở đâu trong MJCF |
|---|---|---|
| $M(q)$ | mass / inertia — cản gia tốc | `mass=`, `<inertial>`, hình học geom |
| $C(q,\dot q)\dot q$ | Coriolis, ly tâm | MuJoCo tự tính từ $M$ |
| $g(q)$ | trọng lực kéo pole xuống | `<option gravity=...>` |
| $Bu$ | lực từ actuator | `<actuator>`, `data.ctrl` |

MuJoCo giải phương trình này ở **mỗi** simulation step.

---

## 25–40 phút — Dựng cart-pole bằng MJCF

Mở [model.xml](model.xml) và đọc theo thứ tự dưới. File đã chú thích sẵn từng khối.

### 25–30 phút — World và simulation settings

```xml
<option timestep="0.002" gravity="0 0 -9.81"/>
```

`timestep` là độ phân giải thời gian:

| timestep | Hệ quả |
|---|---|
| nhỏ (0.001) | chính xác hơn, tốn CPU hơn |
| lớn (0.05) | chạy nhanh, sai số tích luỹ, có thể mất ổn định |

Sẽ quay lại tune ở phần 95–110.

### 30–35 phút — Cart body

```xml
<body name="cart" pos="0 0 1.0">
    <joint name="cart_slide" type="slide" axis="1 0 0"
           limited="true" range="-1 1" damping="0.05"/>
    <geom name="cart_geom" type="box" size="0.15 0.1 0.05" mass="1.0"/>
</body>
```

Cấu trúc lặp lại ở mọi model MuJoCo:

```text
body
 ├── joint  (cử động thế nào so với cha)
 └── geom   (hình dạng gì, nặng bao nhiêu)
```

### 35–40 phút — Pole body

```xml
<body name="pole" pos="0 0 0.05">
    <joint name="pole_hinge" type="hinge" axis="0 1 0" damping="0.01"/>
    <geom name="pole_geom" type="capsule" fromto="0 0 0  0 0 0.6"
          size="0.02" mass="0.1"/>
</body>
```

**Điểm phải nhấn:** pole nằm **bên trong** `<body name="cart">`. Đó chính là coupling ở mức cấu trúc model:

```text
cart moves → gốc pole di chuyển theo cart → pole bị "giật"
```

*Câu hỏi ném ra lớp:* nếu đặt pole ngang hàng với cart trong `<worldbody>` thì sao? — Hai vật độc lập hoàn toàn, không còn là cart-pole nữa.

> `slide` cho tịnh tiến dọc `axis` (mét), `hinge` cho quay quanh `axis` (radian). Cart-pole cần đúng một cái mỗi loại.

---

## 40–55 phút — Mass, inertia và geometry

### 40–45 phút — Mass · TUNE

Cho học viên sửa `mass` của pole trong [model.xml](model.xml), chạy lại `simulate.py` sau mỗi lần:

```text
mass="0.05"  →  "0.1"  →  "0.5"  →  "1.0"
```

*Bắt đoán trước khi chạy:* pole nặng hơn thì cart phản ứng thế nào?

**Đáp án mong đợi:** cùng một lực đẩy, pole càng nặng thì cart tăng tốc càng chậm — lực phải gia tốc **tổng** khối lượng cart + pole.

> 📊 `media/03_mass.png` — chiếu **sau** khi lớp đã đoán.

### 45–50 phút — Inertia

```text
mass    → cản gia tốc DÀI  (tịnh tiến)
inertia → cản gia tốc GÓC  (quay)
```

MuJoCo **tự tính inertia** từ hình dạng và khối lượng geom. Muốn tự khai báo thì dùng `<inertial>`:

```xml
<inertial pos="0 0 0.3" mass="0.1" diaginertia="0.003 0.003 1e-5"/>
```

Không đi sâu inertia tensor ở buổi này.

### 50–55 phút — Geometry KHÔNG chỉ để nhìn · TUNE

Đây là điểm dễ hiểu nhầm nhất của buổi, đáng dành thời gian nhất:

```text
geom size → shape → mass distribution → inertia → dynamics
```

Cho học viên kéo dài pole trong `fromto`, **giữ nguyên `mass`**:

```xml
fromto="0 0 0  0 0 0.6"    →    fromto="0 0 0  0 0 1.2"
```

*Bắt đoán:* cùng khối lượng, cùng trọng lực — pole dài hơn đổ nhanh hơn hay chậm hơn?

**Đáp án:** chậm hơn rõ rệt. Khối lượng phân bố xa trục quay hơn nên inertia lớn hơn nhiều. Với capsule đồng chất, inertia quanh pivot tỉ lệ với **bình phương** chiều dài — gấp đôi chiều dài thì inertia gấp khoảng bốn.

> 📊 `media/04_inertia.png` — có sẵn số inertia của hai trường hợp ở chân hình, tiện đọc thẳng cho lớp.

**Chốt:** khi dùng inertia tự tính, **đổi `size` của geom là đổi luôn vật lý**, không chỉ đổi hình hiển thị.

---

## 55–70 phút — Joint dynamics

### 55–60 phút — Joint range · TUNE

```xml
<joint type="slide" limited="true" range="-1 1"/>
```

Chạy `simulate.py`, kéo thanh trượt hết cỡ về một phía: cart chạy tới ±1 m rồi **dừng khựng**. MuJoCo thêm constraint giữ khớp trong khoảng cho phép.

Cho thử `range="-0.3 0.3"` để thấy cart bị nhốt chật hơn hẳn.

### 60–65 phút — Damping · TUNE

```xml
<joint name="pole_hinge" damping="..."/>
```

Cho chạy lần lượt và quan sát pole đung đưa:

```text
damping = 0  →  0.01  →  0.1  →  1.0
```

```text
damping lớn hơn → chuyển động tắt nhanh hơn
```

Damping là moment **ngược chiều vận tốc khớp**: $\tau = -d\,\dot q$. Vì luôn ngược chiều chuyển động, nó luôn rút năng lượng khỏi hệ.

**Điểm thú vị đáng ném ra lớp:** `damping = 1.0` **không** đưa pole về vị trí nghỉ nhanh hơn `damping = 0.1`. Damping quá lớn thì pole không dao động nữa mà *bò* chậm chạp xuống — chế độ **overdamped**. Nhiều damping không đồng nghĩa với về đích nhanh.

> 📊 `media/05_damping.png` — bốn mức damping trên cùng một hình.
> 🎬 `media/05_damping_0.mp4` và `05_damping_1_0.mp4` — chiếu cạnh nhau để thấy tương phản dao động mãi / bò chậm.

### 65–70 phút — Friction

```xml
<joint frictionloss="0.01"/>
```

| | Bản chất | Khi $\dot q \to 0$ |
|---|---|---|
| `damping` | tỉ lệ vận tốc: $\tau = -d\dot q$ | lực cản yếu dần theo |
| `frictionloss` | ma sát khô, độ lớn gần như không đổi | vẫn chống lại chuyển động |

`frictionloss` mô hình hoá ma sát khô trong hộp số, ổ trục.

> **Lưu ý khi soạn:** lý thuyết nói ma sát khô làm khớp dừng hẳn sau thời gian hữu hạn, còn damping thuần chỉ tiệm cận về 0. Nhưng MuJoCo giải `frictionloss` bằng constraint mềm, nên nhìn trong viewer sẽ không thấy mốc dừng dứt khoát nào. Dạy đúng bản chất khác nhau là đủ — đừng hứa với lớp một hiện tượng mà viewer không cho thấy.

---

## 70–80 phút — Actuator setup

### 70–75 phút — Force actuator

```xml
<actuator>
    <motor name="cart_motor" joint="cart_slide" gear="1"/>
</actuator>
```

Chuỗi ánh xạ cần thuộc:

```text
data.ctrl[0] → cart_motor → cart_slide → lực dọc trục x lên cart
```

`<motor>` là actuator đơn giản nhất: lực ra tỉ lệ thẳng với `ctrl`, hệ số là `gear`. Với `gear="1"` thì `ctrl = 3` nghĩa là 3 N.

Nhắc lại: chỉ cart có actuator, pole thì không.

### 75–80 phút — Control range · TUNE

```xml
<motor ctrllimited="true" ctrlrange="-10 10"/>
```

Trong viewer, kéo thanh trượt `cart_motor` tới các mức khác nhau và xem cart phản ứng. Sau đó sửa thành `ctrlrange="-2 2"`, chạy lại: thanh trượt giờ chỉ kéo được tới 2, và cart yếu hẳn.

> 📊 `media/06_ctrlrange.png` — đường `ctrl = 50` (nét đứt) nằm **chồng khít** lên `ctrl = 10`. Hình này tự nói hết, gần như không cần giảng thêm.

`ctrlrange` **kẹp** lệnh trước khi nó thành lực — đây là cách mô hình hoá giới hạn vật lý của động cơ thật.

Buổi này chỉ ra **lệnh** bằng tay. Chưa có controller nào tự tính lệnh — đó là buổi 3.

---

## 80–95 phút — State inspection

### 80–90 phút — qpos / qvel là gì

Với cart-pole, mapping đúng như trực giác:

```text
qpos[0] → cart position       qvel[0] → cart velocity
qpos[1] → pole angle          qvel[1] → pole angular velocity
```

> 📊 `media/09_state_sequence.png` — chuỗi bốn ảnh pole đổ dần, dùng để chỉ vào và nói "qpos[1] chính là con số này".

Xem trực tiếp trong viewer: nhấn `Tab`, panel **trái**, mục **Watch** — gõ tên trường (`qpos`, `qvel`) để theo dõi giá trị thay đổi theo thời gian. Không cần viết script nào.

**Nhưng phải cảnh báo:** đừng quen đoán index. Ở model lớn có free joint, **`qpos` và `qvel` không cùng độ dài** — một free joint chiếm 7 ô `qpos` (3 vị trí + 4 quaternion) nhưng chỉ 6 ô `qvel`. Cách tra đúng là qua tên:

```python
jid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pole_hinge")
i_pos = model.jnt_qposadr[jid]
i_vel = model.jnt_dofadr[jid]
```

Hoặc gọn hơn — khai báo `<sensor>` trong XML rồi đọc `data.sensordata`. Model của buổi này đã có sẵn bốn sensor để học viên thấy cú pháp.

### 90–95 phút — Bắc cầu sang part 2

Đây là chỗ nối buổi 2 với repo chính. Bài `part_2_models_control/02_cartpole` trên nhánh `main` dùng đúng hệ cart-pole này nhưng ở mức cao hơn: linearization giải tích, `mjd_transitionFD`, thiết kế LQR, swing-up. Toàn bộ phần đọc state bằng Python đã được xử lý sẵn ở đó.

**Thông điệp:** buổi 2 dạy *tham số*; muốn *điều khiển* thì sang part 2.

---

## 95–110 phút — Hands-on experiments

Phần **cốt lõi**. Học viên tự sửa `model.xml` và chạy lại. Nguyên tắc xuyên suốt phải nhắc đi nhắc lại:

> **Mỗi lần chỉ đổi đúng một tham số.** Đổi hai thứ cùng lúc thì không thể biết cái nào gây ra thay đổi.

Quy trình cho từng thí nghiệm: **đoán trước → sửa → chạy → so với dự đoán**. Phần "đoán trước" là phần dạy được; bỏ nó đi thì chỉ còn là nghịch tham số.

### Thí nghiệm 1 — Gravity

```xml
gravity="0 0 -9.81"   →   "0 0 -1.62"   →   "0 0 0"
```

*Hỏi:* thành phần nào của dynamics biến mất khi $g = 0$?

**Quan sát:** với $g = 0$, pole giữ nguyên góc lệch mãi mãi — số hạng $g(q)$ bằng 0, không còn gì kéo nó xuống. (−1.62 là trọng trường Mặt Trăng, tiện để hỏi lớp đoán.)

> 📊 `media/02_gravity.png` · 🎬 `media/02_gravity_zero.mp4`

### Thí nghiệm 2 — Pole mass

```xml
mass="0.05"   →   "0.2"   →   "1.0"
```

Đẩy cart bằng thanh trượt ở cùng một mức lực, so sánh cart tăng tốc nhanh chậm ra sao.

> **Bẫy khi làm thí nghiệm này:** nếu cart đâm vào giới hạn ±1 m thì mọi cấu hình đều dừng ở cùng chỗ và không so sánh được gì. Nhắc học viên chỉ đẩy trong khoảng ngắn, hoặc tạm đặt `limited="false"`.

### Thí nghiệm 3 — Damping

```xml
damping="0"   →   "0.01"   →   "0.1"   →   "1.0"
```

Quan sát pole đung đưa bao lâu thì tắt. Nhớ nhắc lại điểm **overdamped** ở mục 60–65.

### Thí nghiệm 4 — Input force

Kéo thanh trượt `cart_motor` tới các mức khác nhau, xem cart tăng tốc thế nào và pole phản ứng ra sao.

Rồi sửa `ctrlrange` xuống `-2 2` và làm lại: cart yếu hẳn dù kéo hết cỡ.

### Thí nghiệm 5 — Timestep

```xml
timestep="0.001"   →   "0.002"   →   "0.01"   →   "0.05"   →   "0.1"
```

**Thí nghiệm quan trọng nhất với người làm MuJoCo thật.** Quan sát:

- độ mượt của mô phỏng;
- chuyển động có còn hợp lý không;
- ở `dt` đủ lớn, hệ bắt đầu cư xử lạ — sai số tích luỹ mỗi bước.

> 📊 `media/07_timestep.png` — panel phải là sai số RMS trên thang log, gần như một đường thẳng: sai số tỉ lệ tuyến tính với `dt`, đúng như lý thuyết tích phân Euler bậc 1.

**Chốt:** `timestep` là đánh đổi giữa độ chính xác và tốc độ mà **bạn** phải chọn ở mọi dự án. Không có giá trị nào đúng cho mọi trường hợp.

---

## 110–117 phút — Viewer debugging

```bash
python simulate.py --broken
```

[model_broken.xml](model_broken.xml) chứa **ba lỗi**. Cả ba đều hợp lệ về cú pháp — MuJoCo biên dịch trót lọt, không báo gì, chỉ chuyển động là sai. Đó đúng là loại lỗi khó chịu nhất trong thực tế.

Hướng dẫn lớp bật các lớp hiển thị: `Tab` → panel **trái** → mục **Rendering**:

| Bật | Thấy được gì |
|---|---|
| `Joint` | trục quay / trục trượt của từng joint |
| `Frame → Body` | hệ trục gắn với từng body |
| `Com` | khối tâm từng body |
| `Contact Point` | điểm va chạm đang hoạt động |

```text
model behaves incorrectly
         ↓
 inspect MJCF  +  inspect Viewer
         ↓
    debug model
```

> 📊 `media/08_broken_vs_correct.png` — hai hàng đúng/sai, đã bật sẵn `Joint` + `Com`. Dùng làm slide **chữa bài** sau khi lớp đã tự tìm.

Để lớp tự tìm khoảng 5 phút rồi mới chữa. Đáp án ở cuối file.

---

## 117–120 phút — Tổng kết

```text
        MJCF PARAMETERS
               │
   ┌───────────┼───────────┐
 mass       gravity     timestep
 inertia    damping     joint
 friction   actuator
               │
               ▼
        MuJoCo Physics
               │
               ▼
          qpos / qvel
```

| | Câu hỏi |
|---|---|
| Buổi 1 | How to **describe** a system? |
| **Buổi 2** | How does MuJoCo **simulate** it? |
| Buổi 3 | How do we **command** it? |

---

## Checklist chuẩn bị trước buổi dạy

- [ ] Chạy `python make_slides_media.py`, kiểm tra `media/` đủ 10 PNG + 5 MP4, rồi chèn vào slide.
- [ ] Chạy thử `simulate.py` trên đúng máy sẽ trình chiếu — viewer cần OpenGL, hay hỏng ở máy lạ.
- [ ] Chạy thử `simulate.py --broken`, tự tìm lại ba lỗi qua viewer để quen thao tác bật `Rendering`.
- [ ] Chuẩn bị sẵn một bản `model.xml` sạch để khôi phục sau khi lớp tune lung tung.
- [ ] Làm trước cả 5 thí nghiệm, ghi lại **mình quan sát thấy gì** — mô tả hiện tượng bằng lời sẽ tự tin hơn nhiều so với đọc từ giáo án.
- [ ] Nhớ mốc thời gian: hết phút 95 phải xong lý thuyết, để dành trọn 15 phút cho hands-on.

---

## Những gì học viên cần nhớ sau buổi 2

```text
MJCF parameter → physics property → simulation behavior
```

Mười câu để tự kiểm tra:

1. `mass` ảnh hưởng dynamics thế nào?
2. `damping` khai báo ở đâu, và khác `frictionloss` chỗ nào?
3. `gravity` được thiết lập ở đâu?
4. `slide` và `hinge` khác nhau thế nào?
5. Cart-pole cần bao nhiêu joint? Bao nhiêu actuator? Vì sao hai con số đó khác nhau?
6. Actuator tác động vào joint nào?
7. `ctrl` ánh xạ sang lực thế nào, `ctrlrange` can thiệp ở đâu?
8. `qpos` và `qvel` chứa gì, vì sao không nên đoán index?
9. Đổi `timestep` ảnh hưởng simulation thế nào?
10. Dùng Viewer tìm lỗi model ra sao?

---

<details>
<summary><b>ĐÁP ÁN — ba lỗi trong <code>model_broken.xml</code></b></summary>

**Lỗi 1 — sai trục của slide joint.**

```xml
<joint name="cart_slide" type="slide" axis="0 1 0"/>   <!-- sai -->
<joint name="cart_slide" type="slide" axis="1 0 0"/>   <!-- đúng -->
```

Cart trượt theo trục **y**, tức đâm ngang qua thanh ray thay vì chạy dọc nó. Bật `Joint` sẽ thấy mũi tên trục trượt vuông góc với ray.

**Lỗi 2 — pole đặt lơ lửng.**

```xml
<body name="pole" pos="0 0 0.5">    <!-- sai -->
<body name="pole" pos="0 0 0.05">   <!-- đúng -->
```

Gốc pole cách mặt trên cart 0.45 m, treo giữa không trung. Bật `Frame → Body` sẽ thấy hệ trục của pole tách rời hẳn khỏi cart.

**Lỗi 3 — sai trục của hinge joint.**

```xml
<joint name="pole_hinge" type="hinge" axis="1 0 0"/>   <!-- sai -->
<joint name="pole_hinge" type="hinge" axis="0 1 0"/>   <!-- đúng -->
```

Pole quay quanh trục **x** — đổ sang hai bên — trong khi cart chỉ chạy dọc trục x. Hai chuyển động nằm trong hai mặt phẳng vuông góc nên **coupling biến mất**: đẩy cart thế nào pole cũng không phản ứng.

Đây là lỗi nguy hiểm nhất trong ba lỗi vì model vẫn "trông có vẻ chạy". Đáng để dành thời gian nhất khi chữa.

</details>
