# ⚖️ Cân Bằng Con Lắc Ngược với MuJoCo

Dự án này mô phỏng và điều khiển con lắc ngược sử dụng công cụ vật lý MuJoCo. Hệ thống điều khiển gồm 2 giai đoạn:

1. Swing-up (lắc đưa lên) – sử dụng điều khiển năng lượng để đưa con lắc lên thẳng đứng.
2. Ổn định (PID) – khi con lắc gần thẳng đứng, bộ điều khiển PID sẽ giữ thăng bằng.

📁 Tệp tin
Con_lac_nguoc.xml – Mô hình MuJoCo của con lắc ngược.
Can_bang_cln.py – Tập lệnh điều khiển và mô phỏng.
find_stage_1 - Tìm k, kx, kv
find_stage_2 - Tìm kp, ki, kd
Stage_1 - Test giai đoạn Swing-up
Stage_2 - Test giai đoạn ổn định

⚙️ Yêu cầu
Python ≥ 3.8
Các thư viện:

mujoco
mujoco.viewer
numpy
time
math
itertools

Cài đặt bằng pip:

pip install mujoco mujoco.viewer numpy time math itertools

🧠 Nguyên lý điều khiển
Swing-up: tính năng lượng động để tạo mô-men đưa con lắc lên cao.

PID: được kích hoạt khi góc con lắc nằm trong khoảng ±10°, giúp giữ thăng bằng tại vị trí thẳng đứng.
