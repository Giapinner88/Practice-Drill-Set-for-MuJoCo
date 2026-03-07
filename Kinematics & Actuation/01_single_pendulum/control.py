import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import time
import math

# Load model và data
model = mujoco.MjModel.from_xml_path("/home/giap-ros/Mujoco/Practise/Controller/01_single_pendulum/model.xml")
data = mujoco.MjData(model)

# Tham số điều khiển
A = 2.0      # Biên độ mô-men
f = 0.5      # Tần số (Hz)

# Ghi dữ liệu
times, qpos_log, qvel_log, ctrl_log = [], [], [], []

# Khởi động plot
plt.ion()
fig, ax = plt.subplots(3, 1, figsize=(8, 6))
lines = [ax[i].plot([], [])[0] for i in range(3)]

ax[0].set_title("Joint Position (rad)")
ax[1].set_title("Joint Velocity (rad/s)")
ax[2].set_title("Control Torque (Nm)")
for axis in ax:
    axis.set_xlim(0, 10)
    axis.set_ylim(-10, 10)

# Reset ban đầu
mujoco.mj_resetData(model, data)

# Viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running():
        step_start = time.time()

        # Tính thời gian thực tế đã trôi qua
        t = time.time() - start_time

        # Điều khiển torque dạng sin(t)
        torque = A * math.sin(2 * math.pi * f * t)
        data.ctrl[0] = torque

        # Mô phỏng 1 bước
        mujoco.mj_step(model, data)
        viewer.sync()

        # Lưu dữ liệu
        times.append(t)
        qpos_log.append(data.qpos[0])
        qvel_log.append(data.qvel[0])
        ctrl_log.append(torque)

        # Giới hạn 10s gần nhất
        if t > 10:
            times = times[-300:]
            qpos_log = qpos_log[-300:]
            qvel_log = qvel_log[-300:]
            ctrl_log = ctrl_log[-300:]

        # Cập nhật plot
        lines[0].set_data(times, qpos_log)
        lines[1].set_data(times, qvel_log)
        lines[2].set_data(times, ctrl_log)
        for axis in ax:
            axis.set_xlim(max(0, t - 10), t)
        plt.pause(0.001)

        # Duy trì tốc độ real-time
        time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))
