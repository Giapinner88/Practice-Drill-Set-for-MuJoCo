import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path("Kinematics & Actuation/12_Cartpole/model.xml")
data = mujoco.MjData(model)

# Tải ma trận K đã tính
try:
    K = np.load("lqr_gain.npy")
except FileNotFoundError:
    print("Vui lòng chạy mujoco_linearization.py trước để tạo K.")
    exit()

# Thông số vật lý cho Energy Shaping
mp = 1.0
l = 0.5
g = 9.81
# Năng lượng mục tiêu tại vị trí upright (theta = pi): E = 2*m*g*l
E_target = 2.0 * mp * g * l

# Gains cho Energy Shaping (work for 10N limit)
k_E = 20.0              # Bơm năng lượng mạnh
k_stabilize = 0.8       # Giảm xóc khi gần upright
k_x = 3.0               # Giữ cart ở giữa
k_dx = 1.5

# Telemetry
theta_log = []
dtheta_log = []
ctrl_mode_log = [] # 0: Swing-up, 1: LQR

# Khởi tạo: Con lắc buông thõng lệch một chút
data.qpos[0] = 0.0
data.qpos[1] = 0.1
mujoco.mj_forward(model, data)

in_lqr_mode = False

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Đợi 3 giây rồi mới bắt đầu chạy
    time.sleep(3)
    while viewer.is_running() and data.time < 30.0:
        step_start = time.time()
        x = data.qpos[0]
        theta = data.qpos[1]
        dx = data.qvel[0]
        dtheta = data.qvel[1]
        
        # Đưa theta về đoạn [-pi, pi] để dễ tính toán hàm cos
        theta_wrapped = (theta + np.pi) % (2 * np.pi) - np.pi
        theta_upright_err = (theta - np.pi + np.pi) % (2 * np.pi) - np.pi
        
        # Ghi log
        theta_log.append(theta_wrapped)
        dtheta_log.append(dtheta)
        
        # Energy Shaping Control (no mode switching needed for 10N underactuated)

        if in_lqr_mode:
            # LQR Mode
            state_error = np.array([x, theta_upright_err, dx, dtheta])
            u = -K.dot(state_error)[0]
            ctrl_mode_log.append(1)
        else:
            # Energy Shaping Mode (cho cả swing-up lẫn balance)
            # Quy ước theta=0 là down, theta=pi là up.
            E_current = 0.5 * mp * (l * dtheta) ** 2 + mp * g * l * (1.0 - np.cos(theta_wrapped))
            E_err = E_current - E_target
            
            # Gần upright, dùng damping để ổn định; xa upright, bơm năng lượng
            if np.abs(theta_upright_err) < 0.5:
                # Gần upright: dùng energy shaping + strong damping
                u = -k_E * E_err * dtheta * np.cos(theta_wrapped) - k_stabilize * dtheta - k_x * x - k_dx * dx
            else:
                # Xa upright: pure energy shaping để bơm lên
                u = -k_E * E_err * dtheta * np.cos(theta_wrapped) - k_x * x - k_dx * dx
            ctrl_mode_log.append(0)
            
        # Giới hạn lực (Actuator saturation)
        data.ctrl[0] = np.clip(u, -10.0, 10.0)
        
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))

# --- Plot Phase Portrait ---
plt.figure(figsize=(10, 6))
# Tách các điểm theo chế độ điều khiển để vẽ màu khác nhau
theta_log = np.array(theta_log)
dtheta_log = np.array(dtheta_log)
ctrl_mode_log = np.array(ctrl_mode_log)

# Plot Swing-up pha
plt.scatter(theta_log[ctrl_mode_log == 0], dtheta_log[ctrl_mode_log == 0], 
            s=2, c='blue', label='Swing-up (Energy Shaping + PD)')
# Plot LQR pha
plt.scatter(theta_log[ctrl_mode_log == 1], dtheta_log[ctrl_mode_log == 1], 
            s=2, c='red', label='LQR Stabilization')

# Đánh dấu điểm cân bằng
plt.plot([-np.pi, np.pi], [0, 0], 'k*', markersize=12, label='Unstable Equilibrium')

plt.title('Phase Portrait of the Pendulum')
plt.xlabel(r'$\theta$ (rad) - Wrapped to $[-\pi, \pi]$')
plt.ylabel(r'$\dot{\theta}$ (rad/s)')
plt.grid(True)
plt.legend()
plt.show()