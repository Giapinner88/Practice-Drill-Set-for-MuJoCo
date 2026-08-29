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

# PHẦN A — Giải phẫu file `model.xml`

File XML mô tả robot trong MuJoCo gọi là **MJCF** (MuJoCo XML Format). Toàn bộ `model.xml` có khung như sau:

```xml
<mujoco model="so101_basics">   <!-- gốc, bọc tất cả -->
  <compiler .../>               <!-- quy ước dịch file: đơn vị góc, thư mục mesh -->
  <visual>...</visual>          <!-- cách hiển thị: ánh sáng, camera -->
  <default>...</default>        <!-- giá trị mặc định dùng chung -->
  <worldbody>...</worldbody>    <!-- THẾ GIỚI VẬT LÝ: robot, sàn, đèn -->
  <asset>...</asset>            <!-- tài nguyên: mesh, texture, material -->
  <actuator>...</actuator>      <!-- động cơ -->
</mujoco>
```

Ba khối quan trọng nhất là **`<worldbody>`** (robot ở đâu, hình dạng gì), **`<asset>`** (file 3D nào được nạp) và **`<actuator>`** (khớp nào có động cơ). Ta đi lần lượt.

---

## 1. `<mujoco>` — thẻ gốc

```xml
<mujoco model="so101_basics">
```

Mọi file MJCF bắt đầu bằng thẻ này. Thuộc tính `model` chỉ là **tên gọi**, hiện trên tiêu đề cửa sổ viewer. Không ảnh hưởng vật lý.

---

## 2. `<compiler>` — quy ước đọc file

```xml
<compiler angle="radian" meshdir="assets" autolimits="true"/>
```

Thẻ này **không mô tả robot**, nó dặn MuJoCo cách *hiểu* các con số trong file. Rất dễ bị bỏ qua nhưng sai một thuộc tính là robot méo hoàn toàn.

| Thuộc tính | Ý nghĩa |
|---|---|
| `angle="radian"` | Mọi góc trong file tính bằng **radian**. Nếu đổi thành `"degree"` thì `range="-1.69 1.69"` sẽ bị hiểu là ±1.69 **độ** — khớp gần như không nhúc nhích. |
| `meshdir="assets"` | Tìm file `.stl` trong thư mục `assets/`. Nhờ nó mà `<mesh file="base_so101_v2.stl"/>` viết gọn được, khỏi ghi đường dẫn dài. |
| `autolimits="true"` | Hễ khớp có `range` thì tự động coi là bị giới hạn, khỏi phải ghi thêm `limited="true"`. |

> **Đơn vị trong MuJoCo:** mét, kilôgam, giây. Không có thuộc tính nào để đổi. Vì vậy `pos="0 0 0.0624"` nghĩa là 6.24 **cm**, và `mass="0.147"` là 147 **gram**. SO-101 nặng tổng cộng 0.632 kg — con số hợp lý cho một cánh tay in 3D để bàn.

---

## 3. `<worldbody>` — thế giới vật lý

Đây là trái tim của file. Mọi vật thể tồn tại trong mô phỏng đều nằm trong đây.

```xml
<worldbody>
  <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
  <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  <body name="base" pos="0 0 0">
      ...robot...
  </body>
</worldbody>
```

`<worldbody>` chính là **body gốc** đại diện cho thế giới. Nó bất động và có khối lượng vô hạn — bạn không thể đẩy thế giới đi. Sàn nhà (`floor`) gắn trực tiếp vào đây nên nó cũng đứng yên vĩnh viễn.

### 3.1. `<body>` — một khâu cứng

`<body>` là **một mảnh vật chất cứng** (rigid body). Đây là khái niệm trung tâm của MuJoCo.

```xml
<body name="upper_arm" pos="-0.0304 -0.0183 -0.0542" quat="0.5 -0.5 -0.5 -0.5">
```

| Thuộc tính | Ý nghĩa |
|---|---|
| `name` | Tên để tra cứu trong Python: `mj_name2id(model, mjOBJ_BODY, "upper_arm")` |
| `pos` | Vị trí `(x, y, z)` mét |
| `quat` | Hướng xoay, dạng quaternion `(w, x, y, z)` |

**Điểm mấu chốt dễ nhầm nhất:** `pos` và `quat` là **tương đối so với body cha**, không phải toạ độ thế giới. `upper_arm` có `pos="-0.0304 ..."` nghĩa là "lệch 3 cm so với `shoulder`", chứ không phải "ở toạ độ âm trong không gian".

Chạy thử để thấy rõ sự khác biệt — cột `xpos` dưới đây là toạ độ **thế giới** do MuJoCo tự tính ra từ chuỗi các `pos` tương đối:

```
base                parent=world       mass=0.147   xpos=[0.000,  0.000, 0.000]
shoulder            parent=base        mass=0.100   xpos=[0.039, -0.000, 0.062]
upper_arm           parent=shoulder    mass=0.103   xpos=[0.069, -0.018, 0.117]
lower_arm           parent=upper_arm   mass=0.104   xpos=[-0.010,-0.018, 0.201]
wrist               parent=lower_arm   mass=0.079   xpos=[0.125, -0.018, 0.206]
gripper             parent=wrist       mass=0.087   xpos=[0.178, -0.000, 0.177]
moving_jaw          parent=gripper     mass=0.012   xpos=[0.209,  0.018, 0.184]
```

`model.body_pos` giữ giá trị **tương đối** (thứ bạn viết trong XML, cố định); `data.xpos` giữ toạ độ **thế giới** (thứ MuJoCo tính lại mỗi bước, thay đổi khi khớp quay).

### 3.2. Body lồng nhau = chuỗi động học

Các `<body>` lồng trong nhau tạo thành **cây động học** (kinematic tree). SO-101 là một chuỗi thẳng:

```
world
└── base                    (đế, gắn chặt xuống sàn)
    └── shoulder            ← joint: shoulder_pan
        └── upper_arm       ← joint: shoulder_lift
            └── lower_arm   ← joint: elbow_flex
                └── wrist   ← joint: wrist_flex
                    └── gripper          ← joint: wrist_roll
                        └── moving_jaw   ← joint: gripper
```

Trong XML, quan hệ cha–con thể hiện bằng **thụt lề lồng nhau**:

```xml
<body name="shoulder" ...>
    <joint name="shoulder_pan" .../>
    <geom .../>
    <body name="upper_arm" ...>      <!-- con của shoulder -->
        <joint name="shoulder_lift" .../>
        ...
    </body>
</body>
```

**Hệ quả vật lý:** xoay `shoulder_pan` thì *toàn bộ* cánh tay xoay theo, vì mọi body phía dưới đều là con cháu của `shoulder`. Ngược lại, xoay `gripper` chẳng ảnh hưởng gì tới `base`. Chuyển động truyền **từ cha xuống con**, không bao giờ ngược lại.

Chú ý `base` **không có `<joint>`**. Body không có joint thì bị **hàn chặt** vào cha của nó — ở đây là `world`. Đó là lý do đế robot đứng yên. Muốn robot rơi tự do, ta thêm `<freejoint/>` vào `base`.

### 3.3. `<joint>` — khớp nối

`<joint>` định nghĩa **body này được phép chuyển động thế nào so với cha**.

```xml
<joint axis="0 0 1" name="elbow_flex" type="hinge" range="-1.69 1.69" class="sts3215"/>
```

| Thuộc tính | Ý nghĩa |
|---|---|
| `type="hinge"` | Khớp bản lề — quay quanh một trục, giống bản lề cửa. 1 bậc tự do. |
| `axis="0 0 1"` | Trục quay, biểu diễn **trong hệ toạ độ của body**. `0 0 1` là trục Z. |
| `range="-1.69 1.69"` | Giới hạn góc quay (radian, do `angle="radian"`). Khoảng ±97°. |
| `class="sts3215"` | Kế thừa các thuộc tính vật lý từ `<default>` — xem mục 5. |

Cả 6 khớp SO-101 đều dùng `axis="0 0 1"`. Nghe lạ, vì thực tế chúng quay theo nhiều hướng khác nhau. Lý do: mỗi body đã được **xoay sẵn** bằng thuộc tính `quat`, nên trục Z *cục bộ* của mỗi body đã chỉ đúng hướng mong muốn rồi. Đây là quy ước chuẩn của công cụ sinh file tự động.

Các loại `type` thường gặp:

| `type` | Mô tả | Số DOF |
|---|---|---|
| `hinge` | Quay quanh 1 trục (mặc định) | 1 |
| `slide` | Trượt dọc 1 trục | 1 |
| `ball` | Khớp cầu, quay tự do | 3 |
| `free` | Bay tự do trong không gian | 6 |

Giới hạn 6 khớp của SO-101:

| Khớp | `range` (rad) | Tương đương |
|---|---|---|
| `shoulder_pan` | ±1.92 | ±110° |
| `shoulder_lift` | ±1.75 | ±100° |
| `elbow_flex` | ±1.69 | ±97° |
| `wrist_flex` | ±1.66 | ±95° |
| `wrist_roll` | −2.74 … +2.84 | −157° … +163° |
| `gripper` | −0.17 … +1.75 | −10° … +100° (đóng/mở kẹp) |

### 3.4. `<geom>` — hình khối

`<geom>` (geometric primitive) là **hình dạng** của body. Nó phục vụ hai mục đích tách biệt: **hiển thị** và **va chạm**.

```xml
<geom type="mesh" class="visual" pos="-0.065 0.012 0.018" quat="0 1 0 0"
      mesh="upper_arm_so101_v1" material="upper_arm_so101_v1_material"/>
```

| Thuộc tính | Ý nghĩa |
|---|---|
| `type` | Loại hình: `mesh`, `box`, `sphere`, `cylinder`, `capsule`, `plane` |
| `mesh` | Nếu `type="mesh"`, trỏ tới tên mesh khai báo trong `<asset>` |
| `pos`, `quat` | Vị trí/hướng của hình khối **so với body chứa nó** |
| `material` | Màu sắc, độ bóng — khai báo trong `<asset>` |
| `class` | Kế thừa từ `<default>` |

Một body có thể chứa **nhiều geom**. Chẳng hạn `shoulder` gồm 3 mảnh: vỏ động cơ, giá đỡ, khớp xoay. Chúng gắn cứng với nhau thành một khâu duy nhất.

Sàn nhà dùng một loại geom đặc biệt:

```xml
<geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
```

`type="plane"` là mặt phẳng **vô hạn**. Với plane, `size` không phải kích thước thật: hai số đầu bằng `0` nghĩa là trải vô tận, số thứ ba (`0.05`) là bước ô cờ của hoạ tiết.

### 3.5. Cặp `visual` / `collision` — tại sao mỗi mảnh xuất hiện hai lần?

Đọc `model.xml` bạn sẽ thấy mỗi bộ phận được khai báo **hai lần liên tiếp**, chỉ khác `class`:

```xml
<geom type="mesh" class="visual"    mesh="upper_arm_so101_v1" .../>
<geom type="mesh" class="collision" mesh="upper_arm_so101_v1" .../>
```

Đây **không phải lỗi lặp**. Đó là quy ước chuẩn trong robotics, tách bạch hai vai trò:

* **`visual`** — chỉ để **nhìn**. Trong `<default>` nó mang `contype="0" conaffinity="0"`, nghĩa là **hoàn toàn trong suốt với va chạm**: xuyên qua mọi vật, không cản gì cả. Đổi lại, nó được phép chi tiết tuỳ thích.
* **`collision`** — chỉ để **tính va chạm**. Thuộc `group="3"`, mặc định **bị ẩn** trong viewer nên bạn không thấy nó.

Vì sao phải tách? Vì tính va chạm giữa các mesh phức tạp rất **chậm**. Tách đôi cho phép hiển thị mượt mà mà vẫn mô phỏng nhanh — thường hình học collision được đơn giản hoá thành vài hình hộp/trụ thô.

Đếm số geom thực tế trong model sau khi biên dịch:

```
group=0  contype=1  conaffinity=1  →  1 geom   (sàn nhà)
group=2  contype=0  conaffinity=0  →  17 geom  (visual — không va chạm)
group=3  contype=1  conaffinity=1  →  13 geom  (collision — có va chạm)
```

Trong viewer, nhấn phím **`0`–`4`** để bật/tắt từng group. Nhấn `3` sẽ hiện lớp collision đang ẩn.

> **Chi tiết tinh tế:** class `sts3215` cũng khai báo `contype="0"`, nhưng geom nào ghi `class="collision"` thì lấy theo `collision` chứ không lấy `sts3215`. Một geom chỉ nhận **một** class duy nhất — không cộng gộp nhiều class. Bảng trên xác nhận: 13 geom collision vẫn giữ `contype=1`.

### 3.6. `<inertial>` — khối lượng và quán tính

```xml
<inertial pos="-0.0898 -0.0084 0.0184" mass="0.103"
          fullinertia="4.08e-05 1.47e-04 1.42e-04 -1.98e-05 -4.03e-08 8.97e-09"/>
```

| Thuộc tính | Ý nghĩa |
|---|---|
| `mass` | Khối lượng (kg). `0.103` = 103 gram. |
| `pos` | Vị trí **trọng tâm** so với gốc body |
| `fullinertia` | 6 phần tử của ma trận quán tính: `Ixx Iyy Izz Ixy Ixz Iyz` |

Quán tính quyết định vật **khó quay** đến mức nào — tương tự khối lượng quyết định vật khó đẩy đến mức nào. Những con số này lấy từ phần mềm CAD, không cần tự tính tay.

Nếu **bỏ trống** `<inertial>`, MuJoCo tự suy ra từ các geom, giả định vật đặc và đồng chất. Tiện cho bài tập, nhưng kém chính xác cho robot thật — vỏ in 3D thì rỗng, còn động cơ thì đặc và nặng.

### 3.7. `<site>` — điểm đánh dấu

```xml
<site group="3" name="gripperframe" pos="-0.0079 -0.0002 -0.0981" quat="0.707 0 0.707 0"/>
```

`<site>` là một **điểm mốc vô hình**: không khối lượng, không va chạm, không ảnh hưởng vật lý gì cả. Nó chỉ để **đánh dấu vị trí** cho bạn tra cứu sau này.

Site `gripperframe` đánh dấu **điểm làm việc của kẹp** (end-effector). Khi lập trình động học ngược (inverse kinematics) — "muốn đầu kẹp tới toạ độ này thì các khớp phải quay bao nhiêu?" — bạn đọc vị trí của nó:

```python
sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
print(data.site_xpos[sid])   # toạ độ thế giới của đầu kẹp
```

Site cũng là nơi gắn cảm biến (`<sensor>`) trong các bài nâng cao.

### 3.8. `<light>` — nguồn sáng

```xml
<light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
```

Đèn chiếu từ độ cao 3.5 m, hướng thẳng xuống (`dir="0 0 -1"`). `directional="true"` nghĩa là đèn kiểu **mặt trời**: tia sáng song song, cường độ không giảm theo khoảng cách. Chỉ ảnh hưởng đồ hoạ, không ảnh hưởng vật lý.

---

## 4. `<asset>` — kho tài nguyên

`<asset>` chứa các tài nguyên **được tham chiếu bằng tên** từ nơi khác. Nó không đặt gì vào thế giới cả — chỉ là kho khai báo.

```xml
<asset>
  <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" .../>
  <texture type="2d" name="groundplane" builtin="checker" .../>
  <material name="groundplane" texture="groundplane" texrepeat="5 5" reflectance="0.2"/>
  <mesh file="upper_arm_so101_v1.stl"/>
  <material name="upper_arm_so101_v1_material" rgba="1 0.82 0.12 1"/>
</asset>
```

### `<mesh>` — nạp file 3D

```xml
<mesh file="upper_arm_so101_v1.stl"/>
```

Nạp file STL từ thư mục `meshdir`. Không ghi `name` thì MuJoCo lấy **tên file bỏ đuôi** làm tên — nên geom tham chiếu được bằng `mesh="upper_arm_so101_v1"`.

Model này nạp 13 mesh, nhưng có tới 17 geom visual. Chênh lệch vì mesh **dùng lại được**: `sts3215_03a_v1.stl` (vỏ động cơ) xuất hiện 5 lần ở 5 khớp khác nhau. Nạp một lần, vẽ nhiều chỗ — tiết kiệm bộ nhớ.

### `<material>` — màu sắc

```xml
<material name="upper_arm_so101_v1_material" rgba="1 0.82 0.12 1"/>
```

`rgba` gồm 4 số trong khoảng 0–1: đỏ, lục, lam, độ đục. `1 0.82 0.12 1` là **vàng cam** — màu nhựa in 3D đặc trưng của SO-101. Các bộ phận động cơ dùng `rgba="0.1 0.1 0.1 1"` (đen).

Số cuối là alpha: `1` đục hoàn toàn, `0.5` bán trong suốt. Đặt alpha thấp cho geom collision là mẹo hay để nhìn xuyên qua khi debug.

### `<texture>` — hoạ tiết

```xml
<texture type="2d" name="groundplane" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"/>
```

`builtin="checker"` sinh hoạ tiết **ô cờ** ngay trong bộ nhớ, khỏi cần file ảnh. Ô cờ trên sàn không chỉ để đẹp — nó giúp mắt bạn cảm nhận được chiều sâu và chuyển động của robot.

`type="skybox"` với `builtin="gradient"` tạo nền trời chuyển màu xanh → đen.

---

## 5. `<default>` — tránh lặp lại

`<default>` là cơ chế **kế thừa thuộc tính**, giúp file gọn đi rất nhiều.

```xml
<default>
  <default class="sts3215">
    <joint damping="0.60" frictionloss="0.052" armature="0.028"/>
    <position kp="998.22" kv="2.731" forcerange="-2.94 2.94"/>
  </default>
</default>
```

Khai báo trên nói: "Bất cứ `<joint>` nào ghi `class="sts3215"` đều tự động nhận `damping=0.60`, `frictionloss=0.052`, `armature=0.028`."

Nhờ vậy 6 khớp chỉ cần viết ngắn gọn:

```xml
<joint axis="0 0 1" name="elbow_flex" type="hinge" range="-1.69 1.69" class="sts3215"/>
```

thay vì lặp lại ba thuộc tính vật lý ở cả 6 chỗ. Sửa một dòng trong `<default>` là cả 6 khớp đổi theo.

**Quy tắc ưu tiên:** thuộc tính ghi **trực tiếp trên thẻ** luôn thắng giá trị từ `<default>`. Ví dụ class `sts3215` đặt `forcerange="-2.94 2.94"`, nhưng actuator ghi đè `forcerange="-3.35 3.35"` ngay trên thẻ. Kết quả biên dịch thực tế:

```
forcerange = [-3.35, 3.35]     ← giá trị trên thẻ thắng
kp         = 998.22            ← lấy từ class sts3215
```

Ba thuộc tính vật lý của khớp, giải thích ngắn:

| Thuộc tính | Ý nghĩa |
|---|---|
| `damping` | Cản nhớt — lực cản tỉ lệ vận tốc, giúp khớp không dao động mãi |
| `frictionloss` | Ma sát khô — lực cản cố định, phải vượt qua nó khớp mới bắt đầu nhúc nhích |
| `armature` | Quán tính rotor động cơ. Rất quan trọng cho **ổn định số**: thiếu nó, servo `kp` lớn dễ làm mô phỏng nổ. |

---

## 6. `<actuator>` — động cơ

Body và joint mô tả *cấu trúc*. `<actuator>` là thứ khiến robot **chuyển động chủ động**.

```xml
<actuator>
  <position class="sts3215" name="elbow_flex" joint="elbow_flex"
            forcerange="-3.35 3.35" ctrlrange="-1.69 1.69"/>
</actuator>
```

| Thuộc tính | Ý nghĩa |
|---|---|
| `<position>` | Loại actuator: điều khiển **vị trí** (servo) |
| `joint` | Khớp mà nó truyền động |
| `ctrlrange` | Khoảng giá trị hợp lệ của `data.ctrl` — nên khớp với `range` của joint |
| `forcerange` | Moment tối đa (N·m). Servo STS3215 thật cho ~3.35 N·m. |

**Điểm quan trọng nhất cần nắm:** với `<position>`, `data.ctrl[i]` **không phải moment xoắn** mà là **vị trí mục tiêu** tính bằng radian. MuJoCo tự chạy bộ điều khiển PD bên trong:

$$\tau = k_p(\text{ctrl} - q) - k_v\dot{q}$$

với `kp = 998.22`, `kv = 2.731` kế thừa từ class `sts3215`, và `τ` bị chặn trong `forcerange`.

Hệ quả thực tế: robot **không bám lệnh tuyệt đối**. Nếu `ctrl = 1.0` thì khớp dừng ở đâu đó gần 1.0 chứ không đúng bằng — vì cần một sai số khác 0 để sinh ra moment chống lại trọng lực. Đây là **độ võng tĩnh** của bộ PD, hoàn toàn đúng vật lý, không phải lỗi.

Ba loại actuator thường gặp:

| Thẻ | `data.ctrl` mang ý nghĩa | Dùng khi |
|---|---|---|
| `<position>` | Vị trí mục tiêu (rad hoặc m) | Servo, cánh tay robot |
| `<motor>` | Moment/lực trực tiếp (N·m) | Điều khiển moment, học tăng cường |
| `<velocity>` | Vận tốc mục tiêu | Bánh xe, băng chuyền |

`model.nu` = số actuator = 6, đúng bằng độ dài `data.ctrl`.

---

## 7. `<visual>` — thiết lập hiển thị

```xml
<visual>
  <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
  <rgba haze="0.15 0.25 0.35 1"/>
  <global azimuth="160" elevation="-20"/>
</visual>
```

Thẻ này thuần **thẩm mỹ**, không đụng gì tới vật lý. `<headlight>` là đèn gắn trên camera (luôn chiếu theo hướng bạn nhìn); `<global azimuth/elevation>` đặt góc camera ban đầu khi mở viewer.

Đừng nhầm `<visual>` (thiết lập render toàn cục) với `class="visual"` (class geom ở mục 3.5) — hai thứ khác hẳn nhau, chỉ trùng tên.

---

# PHẦN B — Chạy mô phỏng bằng Python

## 8. Hai đối tượng cốt lõi: `mjModel` và `mjData`

Đây là khái niệm quan trọng nhất phía Python.

| | `mjModel` | `mjData` |
|---|---|---|
| Bản chất | Phần **tĩnh** | Phần **động** |
| Chứa gì | Cây động học, khối lượng, mesh, giới hạn khớp, actuator | `qpos`, `qvel`, `ctrl`, lực, `time` |
| Thay đổi khi chạy? | Không | Có, mỗi timestep |
| Tạo bằng | `MjModel.from_xml_path("model.xml")` | `MjData(model)` |

```python
model = mujoco.MjModel.from_xml_path("model.xml")   # biên dịch XML một lần
data  = mujoco.MjData(model)                        # cấp phát bộ nhớ trạng thái
```

Ẩn dụ dễ nhớ: `mjModel` là **bản thiết kế** robot; `mjData` là **robot thật đang chuyển động** tại một thời điểm. Toàn bộ `model.xml` bạn vừa đọc ở Phần A chính là thứ được nén vào `mjModel`.

## 9. Không gian trạng thái

Chạy `simulate.py`, chương trình in ra:

```
nq (số toạ độ vị trí) = 6
nv (số bậc tự do)     = 6
nu (số actuator)      = 6
```

* **`nq`** — độ dài `data.qpos`, vector vị trí khớp
* **`nv`** — độ dài `data.qvel`, vector vận tốc khớp
* **`nu`** — độ dài `data.ctrl`, vector tín hiệu điều khiển

Ở đây `nq == nv` vì SO-101 chỉ có khớp `hinge`, mỗi khớp đúng 1 DOF. Điều này **không phải lúc nào cũng đúng**: một `<freejoint>` chiếm 7 ô trong `qpos` (3 vị trí + 4 quaternion) nhưng chỉ 6 ô trong `qvel` (3 tịnh tiến + 3 vận tốc góc) — quaternion cần 4 số để biểu diễn nhưng chỉ có 3 bậc tự do quay.

## 10. Vòng lặp mô phỏng

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

`launch_passive` nghĩa là **bạn** giữ quyền điều khiển vòng lặp (viewer chỉ hiển thị thụ động). Đối lập với `launch`, khi MuJoCo tự chạy vòng lặp riêng.

Đoạn `time.sleep` cuối vòng lặp giữ nhịp **thời gian thực**: một bước 2 ms cũng tốn đúng 2 ms ngoài đời. Bỏ nó đi thì mô phỏng chạy nhanh hết mức CPU cho phép — điều bạn muốn khi huấn luyện RL, nhưng không muốn khi đang quan sát bằng mắt.

Model này **không khai báo `<option>`**, nên MuJoCo dùng mặc định: `timestep = 0.002` s và `gravity = (0, 0, −9.81)`.

---

## Bài tập

**Bài 1 — Đọc trạng thái.** In `data.qpos` và `data.time` mỗi 100 bước. Xác nhận `data.time` tăng đúng bội số của `model.opt.timestep`.

**Bài 2 — Sai số bám.** In `data.ctrl[i] - data.qpos[i]` cho cả 6 khớp. Khớp nào sai số lớn nhất? Giải thích bằng lập luận trọng lực và moment tải (gợi ý: khớp nào phải đỡ nhiều khối lượng phía sau nó nhất?).

**Bài 3 — Toạ độ tương đối vs tuyệt đối.** In `model.body_pos[i]` và `data.xpos[i]` cho cả 7 body. Đặt `data.qpos[0] = 1.5` rồi gọi `mj_forward` và in lại. Mảng nào đổi, mảng nào không? Vì sao?

**Bài 4 — Nhìn thấy lớp collision.** Mở viewer, nhấn phím `2` rồi `3`. Mô tả sự khác nhau giữa hai lớp geom.

**Bài 5 — Độ cứng bộ điều khiển.** Sửa `kp` của class `sts3215` từ `998.22` xuống `100`, rồi lên `5000`. `kp` nhỏ thì cánh tay võng và trễ; `kp` lớn thì bám sát hơn nhưng có thể dao động hoặc mất ổn định số.

**Bài 6 — Giới hạn khớp.** Bỏ đoạn kẹp `min(max(target, lo), hi)` trong `simulate.py` và tăng biên độ sin lên `2.0`. Khớp có vượt quá `range` không? MuJoCo xử lý giới hạn khớp như ràng buộc **mềm**, không phải tường cứng tuyệt đối.

**Bài 7 — Trọng lực.** Thêm `<option gravity="0 0 0"/>` ngay sau `<compiler>`. Sai số bám ở Bài 2 thay đổi thế nào? Kết quả chứng minh điều gì về nguồn gốc sai số?

**Bài 8 — Đơn vị góc.** Đổi `<compiler angle="radian"/>` thành `"degree"` rồi chạy lại. Giải thích tại sao robot gần như đứng im. Nhớ đổi lại sau khi thử.

**Bài 9 — Tắt viewer.** Bỏ `viewer` và `time.sleep`, chạy 10000 bước rồi đo bằng `time.perf_counter()`. Mô phỏng nhanh gấp bao nhiêu lần thời gian thực? Đây là lý do huấn luyện RL trong mô phỏng khả thi.

---

## Bảng tra nhanh các thẻ

| Thẻ | Thuộc | Vai trò |
|---|---|---|
| `<mujoco>` | gốc | Bọc toàn bộ file |
| `<compiler>` | gốc | Quy ước đọc file: đơn vị góc, thư mục mesh |
| `<visual>` | gốc | Thiết lập hiển thị toàn cục |
| `<default>` | gốc | Giá trị mặc định dùng chung, tránh lặp |
| `<worldbody>` | gốc | Thế giới vật lý |
| `<asset>` | gốc | Kho mesh, texture, material |
| `<actuator>` | gốc | Danh sách động cơ |
| `<body>` | worldbody | Một khâu cứng, lồng nhau thành cây động học |
| `<joint>` | body | Khớp nối — body này cử động thế nào so với cha |
| `<geom>` | body / worldbody | Hình khối: hiển thị và/hoặc va chạm |
| `<inertial>` | body | Khối lượng, trọng tâm, ma trận quán tính |
| `<site>` | body | Điểm mốc vô hình để tra cứu vị trí |
| `<light>` | worldbody | Nguồn sáng |
| `<mesh>` | asset | Nạp file STL |
| `<material>` | asset | Màu sắc, độ bóng |
| `<texture>` | asset | Hoạ tiết ô cờ, nền trời |
| `<position>` | actuator | Servo điều khiển vị trí |

---

## Nguồn

Model và mesh lấy từ [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (`Simulation/SO101`, bản hiệu chuẩn `so101_new_calib`). File `model.xml` ở đây là bản gộp `scene.xml` + `so101_new_calib.xml` + `joints_properties.xml` thành một file duy nhất cho gọn.

Tài liệu MJCF đầy đủ: [mujoco.readthedocs.io/en/stable/XMLreference.html](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
