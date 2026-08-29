"""
Course 1 - SO-101: Form chuẩn của một chương trình MuJoCo.

Đây là bộ khung tối thiểu mà MỌI chương trình MuJoCo đều có:
    nạp model  ->  tạo data  ->  đặt tư thế  ->  vòng lặp (ctrl, step, sync)

Cách học: sửa 6 con số trong TARGET dưới đây rồi chạy lại, xem robot đổi
tư thế thế nào. Đó là cách nhanh nhất để cảm nhận từng khớp làm gì.

Chạy:  python simulate.py
Thoát: nhấn ESC tại cửa sổ viewer.
"""

import os
import time

import mujoco
import mujoco.viewer

# ==========================================================================
# BẠN SỬA Ở ĐÂY - góc mục tiêu của 6 khớp, đơn vị radian
# ==========================================================================
# Mẹo: 90 độ = 1.57 rad, 45 độ = 0.79 rad, 0 độ = 0.0 rad
#
#          tên khớp        giá trị   khoảng cho phép      khớp này làm gì
TARGET = [
    0.0,   # shoulder_pan    [-1.92, +1.92]   xoay đế trái/phải
    -1.0,  # shoulder_lift   [-1.75, +1.75]   nâng/hạ vai
    1.0,   # elbow_flex      [-1.69, +1.69]   gập khuỷu tay
    0.5,   # wrist_flex      [-1.66, +1.66]   gập cổ tay lên/xuống
    0.0,   # wrist_roll      [-2.74, +2.84]   xoay cổ tay
    0.3,   # gripper         [-0.17, +1.75]   0.0 = kẹp đóng, 1.75 = mở hết
]
# ==========================================================================


# --- 1. mjModel: phần TĨNH -------------------------------------------------
# Biên dịch file XML một lần. Chứa mọi thứ không đổi theo thời gian:
# cây động học, khối lượng, mesh, giới hạn khớp, danh sách actuator.
XML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
model = mujoco.MjModel.from_xml_path(XML_PATH)

# --- 2. mjData: phần ĐỘNG --------------------------------------------------
# Chứa trạng thái thay đổi mỗi bước: qpos (vị trí khớp), qvel (vận tốc),
# ctrl (lệnh điều khiển), data.time (thời gian mô phỏng).
data = mujoco.MjData(model)

# --- 3. Đặt tư thế ban đầu -------------------------------------------------
# Ghi thẳng vào qpos, rồi mj_forward để MuJoCo tính lại vị trí các body
# trong không gian. mj_forward KHÔNG tiến thời gian, chỉ tính lại.
# Không có bước này thì robot bắt đầu ở qpos = 0 rồi mới chạy về TARGET.
#
# Kẹp giá trị vào range của khớp, phòng khi bạn gõ số vượt giới hạn.
for i in range(model.nq):
    lo, hi = model.jnt_range[i]
    data.qpos[i] = min(max(TARGET[i], lo), hi)
mujoco.mj_forward(model, data)

# --- 4. Vòng lặp mô phỏng --------------------------------------------------
# Ba việc lặp đi lặp lại: ra lệnh -> tiến một bước -> vẽ lại.
print("Viewer đang chạy. Nhấn ESC để thoát.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.perf_counter()

        # (a) Ra lệnh. Với actuator <position>, ctrl là VỊ TRÍ MỤC TIÊU
        #     (radian), không phải moment xoắn. MuJoCo tự chạy bộ PD bên trong.
        data.ctrl[:] = TARGET

        # (b) Tiến đúng một timestep (2 ms). data.time tăng lên.
        mujoco.mj_step(model, data)

        # (c) Đẩy trạng thái mới lên cửa sổ đồ hoạ. Không ảnh hưởng vật lý.
        viewer.sync()

        # Giữ nhịp thời gian thực: ngủ phần thời gian còn thừa của timestep.
        # Bỏ dòng này đi thì mô phỏng chạy nhanh hết mức CPU cho phép.
        sleep_time = model.opt.timestep - (time.perf_counter() - step_start)
        if sleep_time > 0:
            time.sleep(sleep_time)
