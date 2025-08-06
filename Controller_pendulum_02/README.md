# ⚖️ Điều Khiển trong Mujoco sử dụng PID và không sử dụng PID

📁 Tệp tin

arm_2D_PID.xml – Mô hình MuJoCo của cánh tay 2 khớp.

arm1_2D_PID.xml – Mô hình MuJoCo của cánh tay 1 khớp.

control_PID.py – Tập lệnh điều khiển cánh tay 2 khớp

control1_PID.py – Tập lệnh điều khiển cánh tay 1 khớp

⚙️ Yêu cầu

Python ≥ 3.8

Cài đặt bằng pip:

pip install mujoco mujoco.viewer numpy time matplotlib.pyplot

📈 Biểu đồ

Sau mô phỏng, chương trình hiển thị biểu đồ:

Đường màu xanh: góc thực tế của khớp

Đường màu cam: góc mục tiêu (quỹ đạo mong muốn)
