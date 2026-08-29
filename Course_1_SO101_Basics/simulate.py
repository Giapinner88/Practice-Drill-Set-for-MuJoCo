"""
Course 1 - SO-101: Những thứ căn bản nhất trong MuJoCo.

Chạy:  python simulate.py
Thoát: nhấn ESC tại cửa sổ viewer.
"""

import math
import os
import time

import mujoco
import mujoco.viewer

# --------------------------------------------------------------------------
# 1. mjModel - phần TĨNH của mô phỏng
#    Chứa mọi thứ không đổi theo thời gian: cây động học (kinematic tree),
#    khối lượng, quán tính, mesh, giới hạn khớp, danh sách actuator.
#    Được biên dịch (compile) một lần từ file XML.
# --------------------------------------------------------------------------
XML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
model = mujoco.MjModel.from_xml_path(XML_PATH)

# --------------------------------------------------------------------------
# 2. mjData - phần ĐỘNG của mô phỏng
#    Chứa trạng thái thay đổi mỗi bước: qpos (vị trí khớp), qvel (vận tốc),
#    ctrl (tín hiệu điều khiển), qfrc_* (các loại lực), thời gian data.time.
# --------------------------------------------------------------------------
data = mujoco.MjData(model)

# SO-101 có 6 khớp bản lề (hinge), mỗi khớp 1 bậc tự do và 1 actuator vị trí.
JOINT_NAMES = [
    "shoulder_pan",    # xoay đế quanh trục Z
    "shoulder_lift",   # nâng/hạ vai
    "elbow_flex",      # gập khuỷu
    "wrist_flex",      # gập cổ tay
    "wrist_roll",      # xoay cổ tay
    "gripper",         # đóng/mở kẹp
]

print(f"nq (số toạ độ vị trí) = {model.nq}")
print(f"nv (số bậc tự do)     = {model.nv}")
print(f"nu (số actuator)      = {model.nu}")
print(f"timestep              = {model.opt.timestep} s")
print(f"gravity               = {model.opt.gravity}")
print()
for name in JOINT_NAMES:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    lo, hi = model.jnt_range[jid]
    print(f"  {name:<15} range = [{lo:+.3f}, {hi:+.3f}] rad")
print()

# --------------------------------------------------------------------------
# 3. Đặt tư thế ban đầu (home pose)
#    Ghi trực tiếp vào qpos, rồi gọi mj_forward để MuJoCo tính lại toàn bộ
#    các đại lượng dẫn xuất (vị trí body trong không gian, ma trận quán tính...)
#    mà KHÔNG tiến thời gian.
# --------------------------------------------------------------------------
HOME_QPOS = [0.0, -1.0, 1.0, 0.5, 0.0, 0.3]
data.qpos[:] = HOME_QPOS
mujoco.mj_forward(model, data)

# --------------------------------------------------------------------------
# 4. Vòng lặp mô phỏng
#    - data.ctrl[i] là VỊ TRÍ MỤC TIÊU của actuator i (actuator loại <position>,
#      MuJoCo tự tính moment tau = kp*(ctrl - qpos) - kv*qvel).
#    - mujoco.mj_step tiến hệ đi đúng một timestep.
#    - viewer.sync() đẩy trạng thái mới lên cửa sổ đồ hoạ.
# --------------------------------------------------------------------------
print("Khởi chạy viewer. Nhấn ESC để thoát.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.perf_counter()

        t = data.time

        # Bài demo: cho từng khớp dao động sin quanh tư thế home,
        # đủ để thấy mọi bậc tự do chuyển động và bám theo lệnh điều khiển.
        for i in range(model.nu):
            target = HOME_QPOS[i] + 0.4 * math.sin(2.0 * math.pi * 0.2 * t + i)
            # Kẹp lệnh vào ctrlrange để không vượt giới hạn khớp.
            lo, hi = model.actuator_ctrlrange[i]
            data.ctrl[i] = min(max(target, lo), hi)

        mujoco.mj_step(model, data)
        viewer.sync()

        # Giữ nhịp thời gian thực: ngủ phần thời gian còn thừa của timestep.
        sleep_time = model.opt.timestep - (time.perf_counter() - step_start)
        if sleep_time > 0:
            time.sleep(sleep_time)
