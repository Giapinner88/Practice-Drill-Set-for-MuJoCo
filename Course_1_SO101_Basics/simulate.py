"""
Course 1 - SO-101: Form chuẩn của một chương trình MuJoCo.

Đây là bộ khung tối thiểu mà MỌI chương trình MuJoCo đều có:
    nạp model  ->  tạo data  ->  vòng lặp (step, sync)

Robot được điều khiển TRỰC TIẾP TRONG VIEWER bằng thanh trượt, không cần
sửa code. Mở panel bên phải, kéo 6 thanh trượt và xem cánh tay cử động.

Chạy:  python simulate.py
Thoát: nhấn ESC tại cửa sổ viewer.
"""

import os
import time

import mujoco
import mujoco.viewer

# --- 1. mjModel: phần TĨNH -------------------------------------------------
# Biên dịch file XML một lần. Chứa mọi thứ không đổi theo thời gian:
# cây động học, khối lượng, mesh, giới hạn khớp, danh sách actuator.
XML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
model = mujoco.MjModel.from_xml_path(XML_PATH)

# --- 2. mjData: phần ĐỘNG --------------------------------------------------
# Chứa trạng thái thay đổi mỗi bước: qpos (vị trí khớp), qvel (vận tốc),
# ctrl (lệnh điều khiển), data.time (thời gian mô phỏng).
data = mujoco.MjData(model)

# --- 3. Nạp tư thế ban đầu -------------------------------------------------
# model.xml có sẵn một keyframe tên "home". mj_resetDataKeyframe đặt cả
# qpos lẫn ctrl theo keyframe đó, nên robot đứng yên ngay từ đầu thay vì
# rơi phịch xuống từ tư thế qpos = 0.
mujoco.mj_resetDataKeyframe(model, data, 0)

print(__doc__)
print("Kéo thanh trượt trong panel bên phải để điều khiển robot.")

# --- 4. Vòng lặp mô phỏng --------------------------------------------------
# Chỉ hai việc lặp đi lặp lại: tiến một bước -> vẽ lại.
#
# Chú ý: KHÔNG có dòng nào gán data.ctrl ở đây. Chính thanh trượt trong
# viewer ghi thẳng vào data.ctrl, nên nếu ta gán đè trong vòng lặp thì
# thanh trượt sẽ mất tác dụng ngay lập tức.
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.perf_counter()

        # (a) Tiến đúng một timestep (2 ms). data.time tăng lên.
        mujoco.mj_step(model, data)

        # (b) Đẩy trạng thái mới lên cửa sổ đồ hoạ. Không ảnh hưởng vật lý.
        viewer.sync()

        # Giữ nhịp thời gian thực: ngủ phần thời gian còn thừa của timestep.
        # Bỏ dòng này đi thì mô phỏng chạy nhanh hết mức CPU cho phép.
        sleep_time = model.opt.timestep - (time.perf_counter() - step_start)
        if sleep_time > 0:
            time.sleep(sleep_time)
