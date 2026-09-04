# Course 2 — Cart-pole: Động lực học trong MuJoCo

**Thời lượng:** 120 phút · **Yêu cầu trước:** Course 1 (SO-101 cơ bản)

Buổi 1 trả lời câu hỏi *"làm sao mô tả một hệ?"*. Buổi 2 trả lời câu hỏi tiếp theo: *"MuJoCo mô phỏng hệ đó như thế nào, và tham số nào quyết định kết quả?"*

Toàn bộ buổi học chỉ dùng **một mô hình duy nhất — cart-pole**. Hai bậc tự do, một actuator: đủ đơn giản để nhìn thấy rõ từng tham số vật lý ảnh hưởng lên chuyển động ra sao, mà vẫn đủ giàu để có coupling thật.

```
Course_2_Cartpole_Dynamics/
├── model.xml          # cart-pole, chú thích từng tham số
├── model_broken.xml   # bản cố tình sai 3 lỗi — dùng ở phần debug
├── simulate.py        # ★ BẮT ĐẦU TỪ ĐÂY — form chuẩn, đẩy cart bằng thanh trượt
├── inspect_state.py   # đọc qpos/qvel/sensor, ghi ctrl — không đồ hoạ
├── experiments.py     # 5 thí nghiệm đổi tham số, in kết quả bằng số
└── README.md
```

## Chạy thử

```bash
python simulate.py            # kéo thanh trượt để đẩy cart
python inspect_state.py       # in state ra terminal
python experiments.py         # chạy cả 5 thí nghiệm
python experiments.py 3       # chỉ chạy thí nghiệm số 3
python simulate.py --broken   # model lỗi, dùng ở phần 110-117 phút
```

Cần `pip install mujoco`. Nhấn **ESC** tại cửa sổ viewer để thoát.

---

## Mục tiêu buổi học

Sau buổi này, người học cần:

- Hiểu cart-pole dưới góc nhìn một hệ cơ học.
- Dựng được một cart-pole bằng MJCF từ con số không.
- Hiểu các tham số vật lý chính: `mass`, `inertia`, `damping`, `friction`, `gravity`, `timestep`.
- Thiết lập được joint và actuator cho cart-pole.
- Đọc được state của hệ qua `qpos`, `qvel` — và **tra index thay vì đoán**.
- Tác động lực qua `ctrl`.
- Quan sát được ảnh hưởng của việc đổi tham số lên dynamics.
- Dùng được Viewer để debug model.
- Hiểu ở mức trực quan MuJoCo đang tính dynamics như thế nào.

Mental model chính của buổi:

```text
MJCF parameters
      ↓
  MuJoCo model
      ↓
 State + Input
      ↓
Physics simulation
      ↓
   Motion
```

---

# TIMELINE

| Thời gian | Nội dung | File liên quan |
|---:|---|---|
| 0–10 | Recap buổi 1 | — |
| 10–25 | Hiểu hệ cart-pole | — |
| 25–40 | Dựng cart-pole bằng MJCF | `model.xml` |
| 40–55 | Mass, inertia và geometry | `model.xml` |
| 55–70 | Joint dynamics | `model.xml` |
| 70–80 | Actuator setup | `model.xml` |
| 80–95 | State inspection | `inspect_state.py` |
| 95–110 | Hands-on experiments | `experiments.py` |
| 110–117 | Viewer debugging | `model_broken.xml` |
| 117–120 | Tổng kết | — |

---

## 0–10 phút — Recap buổi 1

Nhắc lại chuỗi mà buổi 1 đã dựng:

```text
MJCF  →  MjModel  →  MjData  →  qpos / qvel / ctrl  →  mj_step()
```

Buổi trước ta coi:

```python
mujoco.mj_step(model, data)
```

như một **black box**. Buổi này tập trung đúng một câu hỏi:

> Những tham số nào quyết định kết quả bên trong black box đó?

Cart-pole là model tối thiểu để quan sát rõ điều đó: chỉ 2 DOF, nên mọi thay đổi đều nhìn thấy được ngay.

---

## 10–25 phút — Hiểu hệ cart-pole

### 10–15 phút — Cấu tạo

```text
World
 └── Cart      (trượt theo trục x)
      └── Pole (quay quanh trục y)
```

Hai bậc tự do. Generalized coordinates và state:

$$
q = \begin{bmatrix} x \\ \theta \end{bmatrix}
\qquad
x_s = \begin{bmatrix} x \\ \theta \\ \dot x \\ \dot\theta \end{bmatrix}
$$

Ở mức cần cho MuJoCo, chỉ cần nhớ:

```text
qpos → configuration (hệ đang ở đâu)
qvel → motion state  (hệ đang chuyển động thế nào)
```

### 15–20 phút — Input và coupling

Cart được tác động bằng một lực duy nhất, $u = F$:

```text
force  →  cart moves  →  pole reacts
```

Điểm mấu chốt: **pole không có actuator nào cả**. Nó chuyển động hoàn toàn do quán tính và trọng lực, gián tiếp qua cart. Hệ như vậy gọi là **underactuated** — 2 DOF nhưng chỉ 1 input. Đây là lý do cart-pole là bài toán kinh điển của điều khiển.

### 20–25 phút — Dynamics ở mức cần thiết

$$
M(q)\ddot q + C(q,\dot q)\dot q + g(q) = Bu
$$

Không derive. Chỉ cần đọc được từng số hạng tương ứng với tham số nào trong MJCF:

| Số hạng | Ý nghĩa | Ở đâu trong MJCF |
|---|---|---|
| $M(q)$ | mass / inertia — cản gia tốc | `mass=`, `<inertial>`, hình học geom |
| $C(q,\dot q)\dot q$ | Coriolis, ly tâm | MuJoCo tự tính từ $M$ |
| $g(q)$ | trọng lực kéo pole xuống | `<option gravity=...>` |
| $Bu$ | lực từ actuator | `<actuator>`, `data.ctrl` |

MuJoCo giải phương trình này ở **mỗi** simulation step.

---

## 25–40 phút — Dựng cart-pole bằng MJCF

Mở [model.xml](model.xml) và đọc theo thứ tự dưới đây. File đã chú thích sẵn từng khối.

### 25–30 phút — World và simulation settings

```xml
<option timestep="0.002" gravity="0 0 -9.81"/>
```

**`timestep`** là độ phân giải thời gian của mô phỏng:

| timestep | Hệ quả |
|---|---|
| nhỏ (0.001) | chính xác hơn, tốn CPU hơn |
| lớn (0.05) | chạy nhanh, sai số tích luỹ, có thể mất ổn định |

Ta sẽ đo cụ thể cái giá của lựa chọn này ở **Thí nghiệm 5**.

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
 ├── joint  (body này cử động thế nào so với cha)
 └── geom   (body này hình dạng gì, nặng bao nhiêu)
```

`type="slide"` + `axis="1 0 0"` = trượt dọc trục x. Đây là `qpos[0]`.

### 35–40 phút — Pole body

```xml
<body name="pole" pos="0 0 0.05">
    <joint name="pole_hinge" type="hinge" axis="0 1 0" damping="0.01"/>
    <geom name="pole_geom" type="capsule" fromto="0 0 0  0 0 0.6"
          size="0.02" mass="0.1"/>
</body>
```

Chú ý pole nằm **bên trong** `<body name="cart">`. Đó không phải chi tiết trình bày — nó chính là coupling ở mức cấu trúc model:

```text
cart moves  →  gốc pole di chuyển theo cart  →  pole bị "giật"
```

Nếu đặt pole ngang hàng với cart trong `<worldbody>`, hai vật sẽ hoàn toàn độc lập và không còn là cart-pole nữa.

> **`slide` và `hinge` khác nhau thế nào?** `slide` cho tịnh tiến dọc `axis` (đơn vị mét), `hinge` cho quay quanh `axis` (đơn vị radian). Cart-pole cần đúng một cái mỗi loại.

---

## 40–55 phút — Mass, inertia và geometry

### 40–45 phút — Mass

Đổi `mass` của pole trong [model.xml](model.xml) rồi chạy lại `simulate.py`:

```xml
mass="0.1"   →   mass="1.0"
```

> **Câu hỏi:** Pole nặng hơn thì cart phản ứng thế nào?

Đo bằng số: `python experiments.py 2`.

### 45–50 phút — Inertia

Ở mức trực quan:

```text
mass    → cản gia tốc DÀI  (tịnh tiến)
inertia → cản gia tốc GÓC  (quay)
```

MuJoCo **tự tính inertia** từ hình dạng và khối lượng của geom. Muốn tự khai báo thì dùng thẻ `<inertial>`:

```xml
<inertial pos="0 0 0.3" mass="0.1" diaginertia="0.003 0.003 1e-5"/>
```

Không cần đi sâu inertia tensor ở buổi này.

### 50–55 phút — Geometry KHÔNG chỉ để nhìn

Đây là điểm dễ hiểu nhầm nhất của buổi, đáng nhấn mạnh:

```text
geom size  →  shape  →  mass distribution  →  inertia  →  dynamics
```

Khi dùng inertia tự tính, **đổi `size` của geom là đổi luôn vật lý**, không chỉ đổi hình hiển thị.

Kiểm chứng: kéo dài pole từ 0.6 m lên 1.2 m trong `fromto`, giữ nguyên `mass="0.1"`.

| Chiều dài pole | Inertia quanh trục quay | Thời gian đổ tới 1.5 rad |
|---|---|---|
| 0.6 m | 3.28 × 10⁻³ | 0.584 s |
| 1.2 m | 1.26 × 10⁻² (≈ 3.8 lần) | 0.790 s |

Cùng khối lượng, cùng trọng lực — nhưng pole dài hơn đổ **chậm hơn rõ rệt**, chỉ vì khối lượng phân bố xa trục quay hơn.

---

## 55–70 phút — Joint dynamics

### 55–60 phút — Joint range

```xml
<joint type="slide" limited="true" range="-1 1"/>
```

Chạy `simulate.py`, kéo thanh trượt hết cỡ về một phía: cart chạy tới ±1 m rồi **dừng khựng**. MuJoCo thêm một constraint để giữ khớp trong khoảng cho phép.

Đây cũng là một cái bẫy khi đo đạc: nếu cart đâm biên thì mọi cấu hình đều cho cùng một `x`, và phép so sánh mất ý nghĩa. `experiments.py` xử lý bằng cách tạm gỡ giới hạn (`free_rail=True`) ở những thí nghiệm cần đo phản ứng của cart.

### 60–65 phút — Damping

```xml
<joint damping="0.1"/>
```

Thử lần lượt `0`, `0.1`, `1.0` và quan sát pole.

```text
damping lớn hơn  →  chuyển động tắt nhanh hơn
```

Damping là moment **ngược chiều vận tốc khớp**: $\tau = -d\,\dot q$. Vì luôn ngược chiều chuyển động, nó luôn rút năng lượng khỏi hệ.

Đo bằng số: `python experiments.py 3`.

### 65–70 phút — Friction

```xml
<joint frictionloss="0.01"/>
```

Phân biệt hai thứ hay bị lẫn:

| | Bản chất | Khi $\dot q = 0$ |
|---|---|---|
| `damping` | tỉ lệ với vận tốc: $\tau = -d\dot q$ | bằng 0 |
| `frictionloss` | ma sát khô, độ lớn gần như không đổi | vẫn chống lại chuyển động |

Hệ quả thực tế: `damping` chống lại chuyển động *tỉ lệ với tốc độ*, nên khớp càng chậm thì lực cản càng yếu. `frictionloss` giữ gần như nguyên độ lớn kể cả khi khớp sắp đứng yên — nó mô hình hoá ma sát khô trong hộp số, ổ trục.

> Trên lý thuyết, ma sát khô làm khớp dừng hẳn sau thời gian hữu hạn, còn damping thuần thì chỉ tiệm cận về 0. Nhưng MuJoCo giải `frictionloss` bằng constraint mềm, nên nếu bạn đo bằng số sẽ thấy cả hai đều chỉ tiến về những giá trị rất nhỏ chứ không cho một mốc dừng dứt khoát. Hiểu đúng bản chất khác nhau là đủ; đừng dùng thí nghiệm số để "chứng minh" điểm lý thuyết này.

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
data.ctrl[0]  →  cart_motor  →  cart_slide  →  lực dọc trục x lên cart
```

`<motor>` là actuator đơn giản nhất: lực ra tỉ lệ thẳng với `ctrl`, hệ số là `gear`. Với `gear="1"` thì `ctrl = 3` nghĩa là 3 N.

Chỉ cart có actuator. Pole thì không — nhắc lại tính underactuated.

### 75–80 phút — Control range

```xml
<motor joint="cart_slide" ctrllimited="true" ctrlrange="-10 10"/>
```

`ctrlrange` **kẹp** lệnh điều khiển trước khi nó thành lực. Ghi `data.ctrl[0] = 50` thì MuJoCo vẫn chỉ ra 10 N — hệt như `ctrl = 10`. Thí nghiệm 4 in ra đúng hiện tượng này.

Đây là mô hình hoá giới hạn vật lý của động cơ thật. Ở buổi này ta chỉ ra **lệnh**; chưa có controller nào tự tính lệnh cả — đó là buổi 3.

---

## 80–95 phút — State inspection

Chạy [inspect_state.py](inspect_state.py) và đọc code song song.

### 80–85 phút — Load model

```python
model = mujoco.MjModel.from_xml_path("model.xml")
data  = mujoco.MjData(model)
```

### 85–90 phút — Đọc state, và ĐỪNG đoán index

Với cart-pole thì mapping đúng như trực giác:

```text
qpos[0] → cart position       qvel[0] → cart velocity
qpos[1] → pole angle          qvel[1] → pole angular velocity
```

Nhưng **không nên đoán**. Cách tra đúng:

```python
jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pole_hinge")
i_pos = model.jnt_qposadr[jid]   # index trong qpos
i_vel = model.jnt_dofadr[jid]    # index trong qvel
```

Vì sao quan trọng: ở model lớn có free joint, **`qpos` và `qvel` không còn cùng độ dài** — một free joint chiếm 7 ô `qpos` (3 vị trí + 4 quaternion) nhưng chỉ 6 ô `qvel`. Đoán index ở đó là sai chắc chắn.

Cách thứ hai, gọn hơn — khai báo `<sensor>` trong XML rồi đọc `data.sensordata`:

```xml
<sensor>
    <jointpos joint="pole_hinge" name="pole_angle"/>
</sensor>
```

### 90–95 phút — Ghi control input

```python
data.ctrl[0] = 3.0
mujoco.mj_step(model, data)
```

```text
ctrl  →  motion  →  qpos/qvel thay đổi
```

`inspect_state.py` in bảng state theo thời gian dưới lực 3 N. Đọc bảng đó sẽ thấy: cart chạy sang phải đều đặn, còn pole **ngả về phía ngược lại** rồi vượt qua 0 và đổ hẳn. Không có actuator nào tác động lên pole — toàn bộ chuyển động của nó là hệ quả gián tiếp. Đó là coupling, quan sát được bằng số.

---

## 95–110 phút — Hands-on experiments

Phần **cốt lõi** của buổi. Chạy `python experiments.py`, hoặc từng cái một với `python experiments.py <số>`.

Nguyên tắc xuyên suốt: **mỗi lần chỉ đổi đúng một tham số**. Đổi hai thứ cùng lúc thì không thể biết cái nào gây ra thay đổi.

### Thí nghiệm 1 — Gravity

```text
gravity="0 0 -9.81"   vs   "0 0 -1.62"   vs   "0 0 0"
```

> Thành phần nào của dynamics biến mất khi $g = 0$?

Với $g = 0$, pole **giữ nguyên góc lệch 0.2 rad mãi mãi** — số hạng $g(q)$ bằng 0, không có gì kéo nó xuống nữa.

### Thí nghiệm 2 — Pole mass

```text
0.05 kg   →   0.2 kg   →   1.0 kg
```

Cùng lực 5 N: pole càng nặng, cart đi được càng ít trong cùng khoảng thời gian. Lực phải gia tốc **tổng** khối lượng cart + pole.

### Thí nghiệm 3 — Damping

```text
0   →   0.01   →   0.1   →   1.0
```

Đo bằng trung bình $|\dot\theta|$ trong 1 giây cuối — tức là hệ **còn đung đưa hay đã đứng yên**. (Đọc $\dot\theta$ tức thời ở đúng thời điểm cuối là con số may rủi: nó phụ thuộc pha dao động.)

Kết quả có một điểm đáng bàn: `damping = 1.0` cho số **nhỉnh hơn** `damping = 0.1`. Không phải sai số — damping quá lớn thì pole không dao động nữa mà *bò* chậm về vị trí thấp nhất, và sau 8 s vẫn đang bò. Đó là chế độ **overdamped**: nhiều damping không đồng nghĩa với về đích nhanh.

### Thí nghiệm 4 — Input force

```text
ctrl = 1   →   5   →   10   →   50
```

Trong 0.3 s đầu, quãng đường xấp xỉ tỉ lệ thuận với lực ($x = \tfrac12 a t^2$, $a = F/m$). Và dòng cuối cho kết quả **hệt** `ctrl = 10`, vì `ctrlrange` đã kẹp lệnh lại.

### Thí nghiệm 5 — Timestep

```text
0.0005   →   0.002   →   0.01   →   0.05   →   0.1
```

So với một mô phỏng tham chiếu ở `dt = 0.00005`. Sai lệch lớn dần và **không tuyến tính**: tới `dt = 0.1` thì lệch hơn 0.5 rad — gần 30°, một mô phỏng không còn dùng được.

Đây là thí nghiệm quan trọng nhất với người làm MuJoCo thật: `timestep` là đánh đổi giữa độ chính xác và tốc độ mà **bạn** phải chọn, ở mọi dự án.

---

## 110–117 phút — Viewer debugging

```bash
python simulate.py --broken
```

[model_broken.xml](model_broken.xml) chứa **ba lỗi**. Cả ba đều hợp lệ về cú pháp — MuJoCo biên dịch trót lọt, không báo gì cả, chỉ có chuyển động là sai. Đó đúng là loại lỗi khó chịu nhất trong thực tế.

Bật các lớp hiển thị: nhấn `Tab`, panel **trái**, mục **Rendering**:

| Bật | Thấy được gì |
|---|---|
| `Joint` | trục quay / trục trượt của từng joint |
| `Frame → Body` | hệ trục gắn với từng body |
| `Com` | khối tâm từng body |
| `Contact Point` | điểm va chạm đang hoạt động |

Mental model của việc debug model:

```text
model behaves incorrectly
         ↓
   inspect MJCF  +  inspect Viewer
         ↓
     debug model
```

Hãy tự tìm trước khi mở đáp án ở cuối file.

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

Vị trí của buổi 2 trong mạch chung:

| | Câu hỏi |
|---|---|
| Buổi 1 | How to **describe** a system? |
| **Buổi 2** | How does MuJoCo **simulate** it? |
| Buổi 3 | How do we **command** it? |

---

## Những gì cần nhớ sau buổi 2

Quan hệ cốt lõi:

```text
MJCF parameter  →  physics property  →  simulation behavior
```

Tự trả lời được 10 câu sau là đạt:

1. `mass` ảnh hưởng dynamics thế nào?
2. `damping` khai báo ở đâu trong MJCF, và khác `frictionloss` chỗ nào?
3. `gravity` được thiết lập ở đâu?
4. `slide` và `hinge` joint khác nhau thế nào?
5. Cart-pole cần bao nhiêu joint? Bao nhiêu actuator? Vì sao hai con số đó khác nhau?
6. Actuator tác động vào joint nào?
7. `ctrl` được ánh xạ sang lực như thế nào, và `ctrlrange` can thiệp ở đâu?
8. `qpos` và `qvel` chứa gì, và vì sao không nên đoán index?
9. Đổi `timestep` ảnh hưởng simulation thế nào?
10. Dùng Viewer để tìm lỗi model ra sao?

---

<details>
<summary><b>ĐÁP ÁN — ba lỗi trong <code>model_broken.xml</code></b> (mở sau khi đã tự tìm)</summary>

**Lỗi 1 — sai trục của slide joint.**

```xml
<joint name="cart_slide" type="slide" axis="0 1 0"/>   <!-- sai -->
<joint name="cart_slide" type="slide" axis="1 0 0"/>   <!-- đúng -->
```

Cart trượt theo trục **y**, tức là đâm xuyên qua thanh ray thay vì chạy dọc nó. Bật `Joint` trong Rendering sẽ thấy mũi tên trục trượt vuông góc với ray.

**Lỗi 2 — pole đặt lơ lửng.**

```xml
<body name="pole" pos="0 0 0.5">    <!-- sai -->
<body name="pole" pos="0 0 0.05">   <!-- đúng -->
```

Gốc pole nằm cách mặt trên của cart 0.45 m, treo giữa không trung. Bật `Frame → Body` sẽ thấy hệ trục của pole tách rời hẳn khỏi cart.

**Lỗi 3 — sai trục của hinge joint.**

```xml
<joint name="pole_hinge" type="hinge" axis="1 0 0"/>   <!-- sai -->
<joint name="pole_hinge" type="hinge" axis="0 1 0"/>   <!-- đúng -->
```

Pole quay quanh trục **x**, tức là đổ sang hai bên, trong khi cart chỉ chạy dọc trục x. Hai chuyển động nằm trong hai mặt phẳng vuông góc nhau nên **coupling biến mất**: đẩy cart thế nào pole cũng không phản ứng. Đây là lỗi nguy hiểm nhất trong ba lỗi vì model vẫn "trông có vẻ chạy".

</details>
