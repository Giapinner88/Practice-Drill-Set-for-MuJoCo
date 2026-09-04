# BUỔI 1 — MUJOCO FUNDAMENTALS & ROBOT MODELING WITH SO-101

**Thời lượng:** 120 phút

## 1. Mục tiêu buổi học

Sau buổi 1, người học cần:

- Hiểu MuJoCo là gì và vai trò của nó trong robotics simulation.
- Cài đặt và chạy được MuJoCo.
- Hiểu cấu trúc cơ bản của MJCF.
- Hiểu cách một robot vật lý được biểu diễn bằng:
  - `body`
  - `joint`
  - `geom`
  - `actuator`
- Hiểu cấu tạo cơ bản của robot SO-101.
- Mapping được:
  - link thật → `body`
  - khớp thật → `joint`
  - motor → `actuator`
- Hiểu các biến cơ bản:
  - `qpos`
  - `qvel`
  - `ctrl`
- Load và chạy được SO-101 trong MuJoCo.
- Điều khiển thủ công các joint thông qua Viewer.
- Hình thành mental model:

```text
Physical robot
      ↓
    MJCF
      ↓
 MjModel + MjData
      ↓
Simulation
      ↓
 qpos / qvel
```

Buổi này **không đi sâu dynamics và control**.

---

# Timeline

## 0–15 phút — Giới thiệu MuJoCo và cài đặt

### 0–5 phút — MuJoCo là gì?

Giới thiệu ngắn:

- MuJoCo là physics engine dành cho hệ cơ học nhiều vật thể.
- Được sử dụng nhiều trong:
  - Robotics.
  - Control.
  - Reinforcement Learning.
  - Manipulation.
  - Legged robotics.

Phân biệt ba khái niệm:

```text
MuJoCo
→ physics engine

MJCF
→ format để mô tả hệ vật lý

Viewer
→ visualization + debugging
```

Giới thiệu pipeline tổng quát:

```text
Robot model
    ↓
Physics engine
    ↓
State
    ↓
Visualization
```

Không giải thích dynamics ở đây.

Chỉ đặt câu hỏi:

> Nếu chúng ta có một robot thật, làm thế nào để MuJoCo hiểu được robot đó?

Đây là câu hỏi chính của buổi 1.

---

### 5–10 phút — Cài đặt MuJoCo

```bash
pip install mujoco
```

Kiểm tra:

```python
import mujoco

print(mujoco.__version__)
```

Có thể giới thiệu cấu trúc project đơn giản:

```text
mujoco_workshop/
├── models/
│   └── so101.xml
│
└── main.py
```

---

### 10–15 phút — Chạy model đầu tiên

Load model:

```python
model = mujoco.MjModel.from_xml_path("models/so101.xml")
data = mujoco.MjData(model)
```

Mở Viewer.

Mục tiêu của phần này:

> Cho người học nhìn thấy robot chạy được trước khi bắt đầu đọc XML.

Không phân tích sâu code.

---

# 15–35 phút — Từ robot vật lý đến MJCF

## 15–20 phút — Robot được cấu tạo từ gì?

Giới thiệu các khái niệm vật lý:

```text
Robot
├── Links
├── Joints
└── Actuators
```

Ví dụ trực quan:

```text
Link
→ rigid component

Joint
→ relative motion between links

Actuator
→ provides force / torque / motion
```

Từ đây mapping sang MuJoCo:

| Robot thật | MuJoCo         |
| ----------- | -------------- |
| Link        | `<body>`     |
| Joint       | `<joint>`    |
| Shape       | `<geom>`     |
| Motor       | `<actuator>` |

Đây là mapping quan trọng nhất của buổi học.

---

## 20–25 phút — Kinematic tree

Giới thiệu robot dưới dạng cây:

```text
base
 └── link_1
      └── link_2
           └── link_3
```

Và tương ứng trong MJCF:

```xml
<body name="base">

    <body name="link_1">

        <body name="link_2">

        </body>

    </body>

</body>
```

Nhấn mạnh:

> Parent-child relationship trong MJCF chính là cấu trúc vật lý của robot.

Không cần DH hoặc transformation matrix ở buổi này.

---

## 25–35 phút — Các tag MJCF quan trọng nhất

Chỉ tập trung vào những tag sẽ dùng xuyên workshop.

### `<worldbody>`

Chứa toàn bộ hệ vật lý.

```xml
<worldbody>
</worldbody>
```

---

### `<body>`

Rigid body.

```xml
<body name="upper_arm" pos="0 0 0.1">
```

Các thuộc tính cần biết:

- `name`
- `pos`
- `quat`

---

### `<joint>`

Định nghĩa degree of freedom.

```xml
<joint
    name="elbow"
    type="hinge"
    axis="0 1 0"
    range="-1.5 1.5"
/>
```

Giải thích:

- `type`
- `axis`
- `range`

Các joint chính:

```text
hinge
slide
ball
free
```

Không đi sâu.

---

### `<geom>`

Geometry của rigid body.

```xml
<geom type="box" size="0.02 0.1 0.02"/>
```

Hai vai trò:

```text
geometry
   ├── visualization
   └── collision
```

---

### `<actuator>`

Tác động lên joint.

```xml
<actuator>

    <position
        joint="elbow"
        kp="50"
    />

</actuator>
```

Mental model:

```text
Actuator
    ↓
 Joint
    ↓
 Body motion
```

---

# 35–55 phút — Case study: SO-101

## 35–40 phút — Giới thiệu SO-101

Giới thiệu robot ở mức hệ thống:

```text
Base
 ↓
Shoulder
 ↓
Upper arm
 ↓
Elbow
 ↓
Forearm
 ↓
Wrist
 ↓
Gripper
```

Cho người học nhìn robot thật / CAD / model.

Yêu cầu xác định:

- đâu là link;
- đâu là joint;
- đâu là actuator.

---

## 40–45 phút — Degree of Freedom

Giới thiệu configuration:

$$
q =
\begin{bmatrix}
q_1 &
q_2 &
\cdots &
q_n
\end{bmatrix}^{T}
$$

Không cần đi sâu toán.

Chỉ giải thích:

```text
q1 → joint 1
q2 → joint 2
...
```

Một bộ giá trị \(q\) tương ứng với một tư thế của robot.

---

## 45–55 phút — Mapping SO-101 sang MJCF

Mở trực tiếp file SO-101 XML.

Trace một nhánh từ base đến gripper.

Ví dụ:

```text
base
 └── shoulder
      └── upper_arm
           └── forearm
                └── wrist
                     └── gripper
```

Với mỗi component, tìm:

```text
<body>
<joint>
<geom>
```

Sau đó tìm actuator tương ứng:

```text
physical motor
      ↓
<actuator>
      ↓
<joint>
      ↓
<body>
```

Mục tiêu:

Người học phải có khả năng đọc XML và hiểu:

> Dòng này đang mô tả bộ phận nào của robot?

---

# 55–70 phút — Model, State và Control Input

Đây là đoạn nối quan trọng sang buổi 2 và buổi 3.

## 55–60 phút — MjModel và MjData

```python
model = mujoco.MjModel.from_xml_path(...)
data = mujoco.MjData(model)
```

Giải thích:

### `MjModel`

Chứa thông tin mô hình:

```text
geometry
mass
joints
actuators
simulation parameters
```

Có thể hiểu gần đúng là:

> "Robot này là robot gì?"

---

### `MjData`

Chứa trạng thái hiện tại:

```text
position
velocity
control input
forces
sensor values
```

Có thể hiểu là:

> "Robot đang ở trạng thái nào?"

---

## 60–66 phút — qpos và qvel

Giới thiệu:

```python
data.qpos
data.qvel
```

Ở mức khái niệm:

$$
x =
\begin{bmatrix}
q\\
\dot q
\end{bmatrix}
$$

Trong đó:

- $q$ vị trí các DOF.
- $\dot q$ vận tốc các DOF.

Không mở dynamics.

Chỉ nói:

> Đây chính là state mà chúng ta sẽ dùng rất nhiều ở buổi 2 và buổi 3.

---

## 66–70 phút — ctrl

```python
data.ctrl
```

Giải thích:

```text
ctrl
 ↓
actuator
 ↓
robot
```

Nhấn mạnh:

```text
qpos ≠ command

ctrl = command/input
```

Đây là distinction quan trọng.

---

# 70–80 phút — Simulation loop

## 70–75 phút — mj_step()

Giới thiệu:

```python
mujoco.mj_step(model, data)
```

Chỉ giải thích ở mức black-box:

```text
Current state
     +
control input
     ↓
   MuJoCo
     ↓
next state
```

Hay:

$$
x_t,\;u_t
\longrightarrow
x_{t+1}
$$

Không giải thích bên trong phép biến đổi này.

Đặt hook cho buổi 2:

> Buổi sau chúng ta sẽ mở hộp đen này ra và xem dynamics tạo ra $x_{t+1}$ như thế nào.

---

## 75–80 phút — Simulation pipeline

Vẽ toàn bộ:

```text
MJCF
 ↓
MjModel
 ↓
MjData
 │
 ├── qpos
 ├── qvel
 └── ctrl
 ↓
mj_step()
 ↓
new qpos / qvel
 ↓
Viewer
```

---

# 80–90 phút — MuJoCo Viewer

## 80–85 phút — Làm quen Viewer

Hướng dẫn:

- rotate camera;
- pan;
- zoom;
- chọn body;
- visualize joints;
- quan sát collision geometry;
- pause / resume simulation.

---

## 85–90 phút — Viewer như debugging tool

Không chỉ coi Viewer là render.

Cho người học quan sát:

- body hierarchy;
- joint axis;
- contact;
- coordinate frames nếu cần.

Ý tưởng chính:

> Viewer giúp kiểm tra xem model chúng ta xây có đúng với robot vật lý hay không.

---

# 90–115 phút — Hands-on: Điều khiển SO-101 thủ công

Phần này nên giữ nhiều thời gian nhất.

## 90–95 phút — Xác định actuator

Cho người học tìm:

```text
actuator 0
actuator 1
actuator 2
...
```

và mapping:

```text
actuator index
     ↓
 actuator name
     ↓
 joint
     ↓
 robot link
```

---

## 95–105 phút — Điều khiển từng joint

Thay đổi từng control input.

Ví dụ:

```text
Shoulder
Elbow
Wrist
```

Quan sát:

- joint nào chuyển động;
- link nào đi theo;
- end-effector thay đổi ra sao.

Cho người học thử hai trường hợp:

### Joint gần base

Quan sát ảnh hưởng lên nhiều link phía sau.

### Joint gần gripper

Quan sát ảnh hưởng cục bộ hơn.

Mục đích:

> củng cố hierarchy của robot.

---

## 105–110 phút — Quan sát state

Trong lúc điều khiển:

```python
print(data.qpos)
print(data.qvel)
print(data.ctrl)
```

Yêu cầu người học quan sát:

```text
change ctrl
    ↓
robot moves
    ↓
qpos changes
```

Chưa cần giải thích vì sao tốc độ hay acceleration thay đổi như vậy.

Đó là nội dung buổi 2.

---

## 110–115 phút — Mini challenge

Đưa SO-101 tới một configuration cho trước.

Ví dụ:

```text
Shoulder → 20°
Elbow    → 45°
Wrist    → -30°
```

Người học cần:

1. Xác định actuator.
2. Thay đổi input.
3. Quan sát robot.
4. Kiểm tra `qpos`.

Không cần viết controller.

---

# 115–120 phút — Tổng kết và nối sang buổi 2

## Những gì đã học

```text
Physical Robot
      ↓
Links / Joints / Motors
      ↓
MJCF
      ↓
Body / Joint / Actuator
      ↓
MjModel
      ↓
MjData
      ↓
qpos / qvel / ctrl
      ↓
mj_step()
      ↓
Next State
```

Đặt câu hỏi:

> Tại sao với cùng một control input, robot lại chuyển động như vậy?

Từ đó giới thiệu buổi 2:

```text
BUỔI 1
Model

    ↓

BUỔI 2
Dynamics
```

Ta đã biết:

$$
x_t,\;u_t
\rightarrow
x_{t+1}
$$

Nhưng chưa biết chính xác phép biến đổi đó xảy ra như thế nào.

Buổi 2 sẽ dùng **cart-pole** để nghiên cứu:

- force;
- acceleration;
- inertia;
- gravity;
- nonlinear dynamics;
- equations of motion;
- numerical simulation.

---

# Phân bổ thời gian

| Nội dung             |          Thời gian |
| --------------------- | ------------------: |
| MuJoCo + cài đặt   |            15 phút |
| Robot → MJCF         |            20 phút |
| SO-101 case study     |            20 phút |
| Model / State / Input |            15 phút |
| Simulation loop       |            10 phút |
| Viewer                |            10 phút |
| Hands-on SO-101       |            25 phút |
| Tổng kết            |             5 phút |
| **Tổng**       | **120 phút** |

# Phạm vi cố ý chưa dạy

Buổi 1 không đi sâu vào:

- Equation of motion.
- Mass matrix.
- Coriolis force.
- Gravity vector.
- Jacobian.
- Forward/inverse dynamics.
- PID.
- LQR.
- IK.
- Trajectory tracking.

Các phần này được giữ lại cho buổi 2 và buổi 3.

# Câu hỏi kiểm tra cuối buổi

Người học nên trả lời được:

1. MuJoCo khác MJCF như thế nào?
2. `body`, `joint`, `geom`, `actuator` đại diện cho gì?
3. Một motor của SO-101 được mapping sang MJCF như thế nào?
4. `MjModel` và `MjData` khác nhau ra sao?
5. `qpos`, `qvel`, `ctrl` là gì?
6. `mj_step()` nhận gì và tạo ra gì?
7. Làm thế nào tìm actuator tương ứng với một joint trên SO-101?
8. Làm thế nào điều khiển thủ công robot và quan sát state thay đổi?

Nếu người học làm được phần hands-on và trả lời được những câu này thì đủ nền để bước sang **buổi 2: Cart-pole, nonlinear system và dynamics**.
