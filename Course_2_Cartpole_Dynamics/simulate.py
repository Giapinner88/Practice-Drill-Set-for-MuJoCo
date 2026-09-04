"""
Course 2 - Cart-pole: form chuẩn của một chương trình MuJoCo.

Cùng bộ khung như Course 1:
    nạp model  ->  tạo data  ->  vòng lặp (step, sync)

Khác biệt: model lần này chỉ có HAI bậc tự do, đủ đơn giản để nhìn thấy
rõ từng tham số vật lý trong model.xml ảnh hưởng lên chuyển động thế nào.

Lực đẩy cart điều khiển bằng THANH TRƯỢT trong viewer (nhấn Tab -> Control),
không cần sửa code.

Chạy:  python simulate.py            # model đúng
       python simulate.py --broken   # model cố tình sai, dùng ở phần debug
Thoát: nhấn ESC tại cửa sổ viewer.
"""

import os
import sys
import time

import mujoco
import mujoco.viewer

# --- 1. mjModel: phần TĨNH -------------------------------------------------
# Mọi thứ không đổi theo thời gian: cây động học, khối lượng, inertia,
# giới hạn khớp, timestep, gravity. Đây là kết quả biên dịch model.xml.
XML_NAME = "model_broken.xml" if "--broken" in sys.argv else "model.xml"
XML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), XML_NAME)
model = mujoco.MjModel.from_xml_path(XML_PATH)

# --- 2. mjData: phần ĐỘNG --------------------------------------------------
# Trạng thái thay đổi mỗi bước: qpos, qvel, ctrl, data.time.
data = mujoco.MjData(model)

# --- 3. Tư thế ban đầu -----------------------------------------------------
# Keyframe "tilted" đặt pole lệch 0.2 rad. Không có nó thì pole đứng đúng
# phương thẳng đứng - một điểm cân bằng KHÔNG BỀN, và về lý thuyết nó sẽ
# đứng yên mãi mãi, chẳng có gì để xem.
mujoco.mj_resetDataKeyframe(model, data, 0)

print(__doc__)
print(f"Model: {XML_NAME}")
print(f"timestep = {model.opt.timestep} s | gravity = {model.opt.gravity}")
print("Nhấn Tab, kéo thanh trượt 'cart_motor' để đẩy cart.")
if "--broken" in sys.argv:
    print("\n>>> Model này có BA lỗi. Bật Rendering -> Joint / Frame / Com")
    print(">>> trong panel trái để tìm. Đáp án ở cuối README.md.")

# --- 4. Vòng lặp mô phỏng --------------------------------------------------
# KHÔNG gán data.ctrl ở đây: thanh trượt trong viewer ghi thẳng vào
# data.ctrl, gán đè trong vòng lặp là thanh trượt mất tác dụng ngay.
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.perf_counter()

        # (a) Tiến đúng một timestep. Toàn bộ vật lý nằm trong dòng này.
        mujoco.mj_step(model, data)

        # (b) Vẽ lại. Không ảnh hưởng vật lý.
        viewer.sync()

        # Giữ nhịp thời gian thực.
        sleep_time = model.opt.timestep - (time.perf_counter() - step_start)
        if sleep_time > 0:
            time.sleep(sleep_time)
